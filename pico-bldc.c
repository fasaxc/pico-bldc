#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/i2c_slave.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/divider.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "hardware/i2c.h"
#include "pico/critical_section.h"
#include "hardware/pio.h"
#include "pwmhigh.pio.h"
#include "pwminvl.pio.h"
#include <stdfix.h>

// GPIO defines
#define PIN_DEBUG 0
#define PIN_PWM_IN 18

#define PIN_MOTOR_A 16
#define PIN_MOTOR_B 15
#define PIN_MOTOR_C 14

#define PIN_BUTT_A 3
#define PIN_BUTT_B 5

// Motor defines
#define MOTOR_NUM_POLES 22


// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
// #define SPI_PORT spi0
// #define PIN_MISO 16
// #define PIN_CS   17
// #define PIN_SCK  18
// #define PIN_MOSI 19

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9
#define I2C_SLAVE_ADDRESS 0x42

#define LUT_LEN 4096
static u_int16_t pwm_lut[LUT_LEN];

static struct
{
    uint8_t addr;
    uint32_t value;
    uint32_t value_out;
    int num_bytes_received;
    bool addr_received;
} i2c_context = {};

enum I2CRegs {
    I2C_REG_CTRL,
    I2C_REG_MOTOR_SPEEDS,
    I2C_REG_COUNT,
};

static critical_section_t i2c_reg_lock;
static uint32_t volatile i2c_registers[I2C_REG_COUNT] = {};

static inline uint32_t i2c_reg_get(enum I2CRegs num) {
    critical_section_enter_blocking(&i2c_reg_lock);
    uint32_t out = i2c_registers[num];
    critical_section_exit(&i2c_reg_lock);
    return out;
}

static inline uint32_t i2c_reg_set(enum I2CRegs num, uint32_t value) {
    critical_section_enter_blocking(&i2c_reg_lock);
    i2c_registers[num] = value;
    critical_section_exit(&i2c_reg_lock);
}

// Our handler is called from the I2C ISR, so it must complete quickly. Blocking calls /
// printing to stdio may interfere with interrupt handling.
static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
    case I2C_SLAVE_RECEIVE: // master has written some data
        uint8_t data = i2c_read_byte_raw(i2c);
        if (i2c_context.addr_received) {
            i2c_context.value = (i2c_context.value << 8) | data;
            i2c_context.num_bytes_received++;
        } else {
            i2c_context.addr = data;
            i2c_context.addr_received = true;
            i2c_context.value = 0;
            i2c_context.num_bytes_received = 0;
            if (i2c_context.addr < I2C_REG_COUNT) {
                i2c_context.value_out = i2c_reg_get(i2c_context.addr);
            } else {
                i2c_context.value_out = 0;
            }
        }
        break;
    case I2C_SLAVE_REQUEST: // master is requesting data, called once per byte.
        i2c_write_byte_raw(i2c, (uint8_t)(i2c_context.value_out>>24));
        i2c_context.value_out <<= 8;
        break;
    case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
        if (i2c_context.addr_received && 
            (i2c_context.addr < I2C_REG_COUNT) &&
            (i2c_context.num_bytes_received == 4)) {
            // Valid address.
            i2c_reg_set(i2c_context.addr, i2c_context.value);
        }
        i2c_context.addr_received = false;
        break;
    default:
        break;
    }
}

// Make VSCode intellisense happy.
//#define VSCODE
#ifdef VSCODE
#define _Accum int
#define _Fract int
#define fix15c(x) x
#error VSCODE hack enabled
#else
#define fix15c(x) x ## k
#endif

#define fix15_t signed accum
static inline fix15_t kbits(uint32_t x) {
    union {
        fix15_t f;
        uint32_t y;
    } u;
    u.y = x;
    return u.f;
} 

static inline void print_fix15(char *msg, fix15_t v) {
    printf("%s=%+f ", msg, (float)v);
}

#define CLAMP_ANGLE(a) ((a+1000) - (int)(a+1000))

#define ANGLE_RING_BUF_SIZE 4
struct motor_cb {
    fix15_t target_velocity;  // Revs/s
    fix15_t angle_offset;     // Calibration offset to align poles.

    uint pwm_slice_a, pwm_slice_b, pwm_slice_c;
    uint pwm_chan_a, pwm_chan_b, pwm_chan_c;

    fix15_t estimated_velocity; // Revs/sec
    //fix15_t estimated_accel;    // Revs/sec sq

    fix15_t output_throttle;    // -1.0 to 1.0
    
    fix15_t last_sensor_pwm_interval;

    struct {
        uint32_t time_us;
        fix15_t angle;  // Range 0.0-1.0
    } measured_angles_ring_buf[ANGLE_RING_BUF_SIZE];
    int angle_buf_idx;

    bool angle_meas_pending;
    bool output_update_pending;
};

void motor_record_pwm_interval(struct motor_cb *cb, uint32_t raw_pio_output);
void motor_record_pwm_high_time(struct motor_cb *cb, uint32_t raw_pio_output);
void motor_record_angle_meas(struct motor_cb *cb, fix15_t angle);

void motor_record_pwm_interval(struct motor_cb *cb, uint32_t raw_pio_output) {
    uint32_t invl = 0xffffffff - raw_pio_output;
    cb->last_sensor_pwm_interval = invl;
}

void motor_record_pwm_high_time(struct motor_cb *cb, uint32_t raw_pio_output) {
    uint32_t invl = cb->last_sensor_pwm_interval;
    invl <<= 12;
    if (invl == 0) {
        return;
    }
    uint32_t high = 0xffffffff - raw_pio_output;
    high <<= 12;
    uint32_t one_clock = invl / 
        (16/*start bits*/ + 4095/*data bits*/ + 8/*end bits*/);
    // Integer division truncates; add half a clock to get
    // round-to-nearest.
    uint32_t half_clock = one_clock / 2; 
    uint32_t clocks = (high+half_clock)/one_clock; 
    //printf("h=%d i=%d ", high, invl);
    fix15_t angle = (fix15_t)clocks - fix15c(16);
    //print_fix15("raw", angle);
    angle >>= 12;
    angle += cb->angle_offset;
    angle = CLAMP_ANGLE(angle);
    motor_record_angle_meas(cb, angle);
}

void motor_record_angle_meas(struct motor_cb *cb, fix15_t angle) {
    uint32_t now = time_us_32();
    int next_idx = cb->angle_buf_idx+1;
    if (next_idx == ANGLE_RING_BUF_SIZE) {
        next_idx = 0;
    }
    cb->measured_angles_ring_buf[next_idx].time_us = now;
    cb->measured_angles_ring_buf[next_idx].angle = angle;
    cb->angle_buf_idx = next_idx;
    cb->angle_meas_pending = true;
    cb->output_update_pending = true;
}

// Using kbits effectively divides by 2^15 so, to get seconds,
// we need to multiply by 2^15 / 1,000,000.
#define USEC_TO_FIX15_S(us) (kbits(us) * fix15c(0.032768))

void motor_process_angle_meas(struct motor_cb *cb) {
    int idx = cb->angle_buf_idx;
    int prev_idx = (idx==0)?(ANGLE_RING_BUF_SIZE-1):(idx - 1);
    fix15_t prev_angle = cb->measured_angles_ring_buf[prev_idx].angle;
    uint32_t prev_time = cb->measured_angles_ring_buf[prev_idx].time_us;
    if (prev_time == 0) {
        // Need to wait for second reading.
        return;
    }
    fix15_t current_angle = cb->measured_angles_ring_buf[idx].angle;
    uint32_t current_time = cb->measured_angles_ring_buf[idx].time_us;
    
    uint32_t delta_t_us = current_time - prev_time;
    fix15_t delta_angle = current_angle - prev_angle;
    if (delta_angle > fix15c(0.5)) {
        delta_angle -= 1;
    } else if (delta_angle < fix15c(-0.5)) {
        delta_angle += 1;
    }

    fix15_t delta_t_s = USEC_TO_FIX15_S(delta_t_us);
    fix15_t est_v = delta_angle / delta_t_s;

    // static int n;
    // if ((n++ % 1) == 0) {
        //  print_fix15("an", current_angle);
        //  print_fix15("dan", delta_angle);
        //  print_fix15("dt", delta_t_s);
        //  print_fix15("ev", est_v);
        //  printf("\n");
    // }  

    cb->estimated_velocity = est_v;

    fix15_t v_error = -(cb->target_velocity - est_v);
    cb->output_throttle += v_error * fix15c(0.02);
    if (cb->output_throttle > fix15c(1)) {
        cb->output_throttle = fix15c(1);
    } else if (cb->output_throttle < fix15c(-1)) {
        cb->output_throttle = fix15c(-1);
    }
}

void motor_set_pwms(struct motor_cb *cb, fix15_t drive_angle) {
    u_int16_t duty_a = 0, duty_b = 0, duty_c = 0;
    uint lut_bin = drive_angle * LUT_LEN; 
    
    // static int n;
    // if ((n++ % 1) == 0)
    //     print_fix15("l", lut_bin);

    fix15_t factor = cb->output_throttle < 0?-cb->output_throttle:cb->output_throttle;
    if (cb->output_throttle != 0) {
        duty_a = pwm_lut[(lut_bin) % LUT_LEN] * factor;
        duty_b = pwm_lut[(lut_bin + (LUT_LEN/3)) % LUT_LEN] * factor;
        duty_c = pwm_lut[(lut_bin + (2*LUT_LEN/3)) % LUT_LEN] * factor;
    }

    // printf("a=%4hd b=%4hd c=%4hd", duty_a, duty_b, duty_c);

    pwm_set_chan_level(cb->pwm_slice_a, cb->pwm_chan_a, duty_a);
    pwm_set_chan_level(cb->pwm_slice_b, cb->pwm_chan_b, duty_b);
    pwm_set_chan_level(cb->pwm_slice_c, cb->pwm_chan_c, duty_c);
}

// Nature of the PWM signal means we're always 1ms behind.
#define READING_DELAY_US 1000
void motor_update_output(struct motor_cb *cb) {
    // Estimate current position of the motor given time since 
    // last reading and calculated velocity.
    int idx = cb->angle_buf_idx;
    fix15_t last_reading_angle = cb->measured_angles_ring_buf[idx].angle;
    uint32_t last_reading_time = cb->measured_angles_ring_buf[idx].time_us;
    uint32_t now = time_us_32();
    uint32_t us_since_reading = now - last_reading_time + READING_DELAY_US;
    fix15_t s_since_reading = USEC_TO_FIX15_S(us_since_reading);
    fix15_t angle_est = last_reading_angle + cb->estimated_velocity * s_since_reading;
    
    static int n;
    angle_est = CLAMP_ANGLE(angle_est);

    // Multiply up by number of pole pairs to get the effective electrical
    // "angle". 
    fix15_t meas_pole_angle = angle_est*(MOTOR_NUM_POLES/2);
    fix15_t drive_angle = meas_pole_angle;
    if (cb->output_throttle < 0) {
        drive_angle += fix15c(0.25);
    } else if (cb->output_throttle > 0) {
        drive_angle -= fix15c(0.25);
    }
    drive_angle = CLAMP_ANGLE(drive_angle);

    //if ((n++ % 1) == 0) {
        // print_fix15("ma", meas_pole_angle);
        // print_fix15("dra", drive_angle);
    //}

    motor_set_pwms(cb, drive_angle);
}

void motor_update(struct motor_cb *cb) {
    if (cb->angle_meas_pending) {
        cb->angle_meas_pending = false;
        motor_process_angle_meas(cb);
    }
    if (cb->output_update_pending) {
        cb->output_update_pending = false;
        motor_update_output(cb);
    }
    //printf("\n");
}

static uint32_t drain_pio_fifo_blocking(PIO pio, uint sm) {
    uint32_t v;
    do {
        v = pio_sm_get_blocking(pio, sm);
    } while (!pio_sm_is_rx_fifo_empty(pio, sm));
    return v;
}

static uint32_t drain_pio_fifo_non_block(PIO pio, uint sm) {
    uint32_t v = 0;
    while (!pio_sm_is_rx_fifo_empty(pio, sm)) { 
        v = pio_sm_get(pio, sm);
    }
    return v;
}

int main()
{
    stdio_init_all();

    // Init "debug" pin, which we toggle for reading by the
    // oscilloscope.
    gpio_init(PIN_DEBUG);
    gpio_set_dir(PIN_DEBUG, GPIO_OUT);
    gpio_put(PIN_DEBUG, 0);

    gpio_init(PIN_BUTT_A);
    gpio_set_dir(PIN_BUTT_A, GPIO_IN);
    gpio_pull_up(PIN_BUTT_A);

    gpio_init(PIN_BUTT_B);
    gpio_set_dir(PIN_BUTT_B, GPIO_IN);
    gpio_pull_up(PIN_BUTT_B);
    
    // Configure motor PWM outputs.
    // PWM_TOP = 6200 gives roughly 20kHz.
#define PWM_TOP 6200
    pwm_config pwm_c = pwm_get_default_config();
    pwm_config_set_wrap(&pwm_c, PWM_TOP);
    pwm_config_set_clkdiv_int(&pwm_c, 1);
    pwm_config_set_phase_correct(&pwm_c, 1);

    gpio_set_function(PIN_MOTOR_A, GPIO_FUNC_PWM);
    uint pwm_slice_a = pwm_gpio_to_slice_num(PIN_MOTOR_A);
    uint pwm_chan_a = pwm_gpio_to_channel(PIN_MOTOR_A);
    pwm_init(pwm_slice_a, &pwm_c, false);

    gpio_set_function(PIN_MOTOR_B, GPIO_FUNC_PWM);
    uint pwm_slice_b = pwm_gpio_to_slice_num(PIN_MOTOR_B);
    uint pwm_chan_b = pwm_gpio_to_channel(PIN_MOTOR_B);
    pwm_init(pwm_slice_b, &pwm_c, false);

    gpio_set_function(PIN_MOTOR_C, GPIO_FUNC_PWM);
    uint pwm_slice_c = pwm_gpio_to_slice_num(PIN_MOTOR_C);
    uint pwm_chan_c = pwm_gpio_to_channel(PIN_MOTOR_C);
    pwm_init(pwm_slice_c, &pwm_c, false);

    // Enable all PWMs together so that they start in phase.
    pwm_set_mask_enabled((1<<pwm_slice_a) | 
                         (1<<pwm_slice_b) |  
                         (1<<pwm_slice_c));
    
    // Configure PIO programs to measure PWM interval and high time
    // from the motor position sensor.
    PIO pio = pio0;
    uint offset_invl = pio_add_program(pio, &pwminvl_program);
    uint sm_invl = pio_claim_unused_sm(pio, true);
    pwminvl_program_init(pio, sm_invl, offset_invl, PIN_PWM_IN);

    uint offset_high = pio_add_program(pio, &pwmhigh_program);
    uint sm_high = pio_claim_unused_sm(pio, true);
    pwmhigh_program_init(pio, sm_high, offset_high, PIN_PWM_IN);
    
    // Init LUT for sine.
#define PWM_OFFSET 0.032f
    for (int i=0; i<LUT_LEN; i++) {
        float angle = i*M_TWOPI/LUT_LEN;
        float sinef = sinf(angle);
        float offset = (sinef + 1.0f + PWM_OFFSET)/2;
        float scaled = offset * PWM_TOP / (1.0f + PWM_OFFSET);
        float clamped = scaled > PWM_TOP ? PWM_TOP : scaled;
        pwm_lut[i] = (uint16_t)(clamped);
    }

    // I2C Initialisation.
    critical_section_init(&i2c_reg_lock);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    i2c_init(I2C_PORT, 100*1000);
    i2c_slave_init(I2C_PORT, I2C_SLAVE_ADDRESS, &i2c_slave_handler);

    while (!pio_sm_get_rx_fifo_level(pio, sm_invl)) {
        // Wait for the first interval measurement to arrive so
        // we'll definitely load it on the first loop below.
    }

    while (gpio_get(PIN_BUTT_A)) {
        // Wait for the (active low) button to be pressed.
    }

    // Determine angle offset of the motor.  Do a slow round of open loop 
    // control to capture the rotor and then read the position.
    const uint fp_bits = 12; // Fixed point bits to use for division
    uint32_t pole_angle = 0;
    uint32_t angle_offset = 2048;
    uint32_t meas_pole_angle = 0;
    int32_t last_wheel_angle;
    for (pole_angle = 0; pole_angle < 256; pole_angle++) {
        uint32_t high, invl, one_clock, half_clock;
        // Load the PWM high time from the PIO.
        do {
            high = 0xffffffff - pio_sm_get_blocking(pio, sm_high);
            gpio_put(PIN_DEBUG, 1);
        } while (pio_sm_get_rx_fifo_level(pio, sm_high));

        // Load any updated PWM interval from the PIO.  The interval
        // is updated every other PWM cycle so we don't wait for it.
        while(pio_sm_get_rx_fifo_level(pio, sm_invl)) {
            invl = 0xffffffff - pio_sm_get_blocking(pio, sm_invl);
            one_clock = (invl << fp_bits) / (16+4095+8);
            half_clock = one_clock / 2;
        }

        // Convert to fixed point for better precision.
        high = high << fp_bits;
        // Convert to angle (in 4096ths of a circle).
        int32_t wheel_angle = (((high + half_clock) / one_clock) - 16) & 0xfff;
        last_wheel_angle = wheel_angle;

        // Using 0-4095 for our angle range.
        // Convert wheel angle to angle relative to pole of 
        // magnet.
        meas_pole_angle = (wheel_angle * MOTOR_NUM_POLES/2) % 4096;

        if ((pole_angle % 16) == 0) {
            printf("%04d %04d\n", pole_angle, wheel_angle);
        }

        u_int16_t duty_a = pwm_lut[(pole_angle) % LUT_LEN];
        u_int16_t duty_b = pwm_lut[(pole_angle + (LUT_LEN/3)) % LUT_LEN];
        u_int16_t duty_c = pwm_lut[(pole_angle + (2*LUT_LEN/3)) % LUT_LEN];

        pwm_set_chan_level(pwm_slice_a, pwm_chan_a, duty_a);
        pwm_set_chan_level(pwm_slice_b, pwm_chan_b, duty_b);
        pwm_set_chan_level(pwm_slice_c, pwm_chan_c, duty_c);
    
        gpio_put(PIN_DEBUG, 0);
    }

    angle_offset = (pole_angle - meas_pole_angle) % 4096;

    pwm_set_chan_level(pwm_slice_a, pwm_chan_a, 0);
    pwm_set_chan_level(pwm_slice_b, pwm_chan_b, 0);
    pwm_set_chan_level(pwm_slice_c, pwm_chan_c, 0);
    printf("Pole angle: %04d meas_pole_angle: %04d angle_offset: %04d\n", 
        pole_angle, meas_pole_angle, angle_offset);

    // Start closed-loop control.
    struct motor_cb m = {
        .target_velocity = fix15c(0.1),
        .angle_offset = ((fix15_t)angle_offset)/4096/(MOTOR_NUM_POLES/2),
        .pwm_slice_a = pwm_slice_a,
        .pwm_slice_b = pwm_slice_b,
        .pwm_slice_c = pwm_slice_c,
        .pwm_chan_a = pwm_chan_a,
        .pwm_chan_b = pwm_chan_b,
        .pwm_chan_c = pwm_chan_c,
    };

    while (true) {
        uint32_t raw_pio_high = drain_pio_fifo_blocking(pio, sm_high);
        uint32_t raw_pio_invl = drain_pio_fifo_non_block(pio, sm_invl);
        
        if (raw_pio_invl != 0) {
            motor_record_pwm_interval(&m, raw_pio_invl);
        }
        gpio_put(PIN_DEBUG, 1);
        motor_record_pwm_high_time(&m, raw_pio_high);

        if (i2c_reg_get(I2C_REG_CTRL) & 1) {
            // I2C control is enabled.
            uint32_t motor_speeds = i2c_reg_get(I2C_REG_MOTOR_SPEEDS);
            int32_t m1 = motor_speeds & 0xff;
            if (m1 > 127) {
                m1 -= 256;
            }
            m.target_velocity = ((fix15_t)m1)/10;
        }

        motor_update(&m);

        // Read buttons (which are pull-downs) and adjust target speed accordingly.
        if (!gpio_get(PIN_BUTT_A) && m.target_velocity > -20) {
            m.target_velocity -= fix15c(0.005);
        }
        if (!gpio_get(PIN_BUTT_B) && m.target_velocity < 20) {
            m.target_velocity += fix15c(0.005);
        }

        gpio_put(PIN_DEBUG, 0);
    }

    return 0;
}
