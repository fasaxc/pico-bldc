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
#define LUT_LEN 4096
#define PWM_OFFSET 0.032f
    u_int16_t pwm_lut[LUT_LEN];
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
    int32_t target_speed = 100;
    int32_t throttle = 0x8ff;
    uint64_t last_time = time_us_64();
    uint32_t n = 0;
#define num_angles 3
    int angle_idx = 0;
    int32_t delta_angles[num_angles] = {};
    while (true) {
        uint32_t high, invl, one_clock, half_clock;
        // Load the PWM high time from the PIO.
        do {
            high = 0xffffffff - pio_sm_get_blocking(pio, sm_high);
            gpio_put(PIN_DEBUG, 1);
        } while (pio_sm_get_rx_fifo_level(pio, sm_high));
        uint64_t loop_start = time_us_64();

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
        int32_t wheel_angle = (((high + half_clock) / one_clock) - 16) % 4096;

        if (i2c_reg_get(I2C_REG_CTRL) & 1) {
            // I2C control is enabled.
            uint32_t motor_speeds = i2c_reg_get(I2C_REG_MOTOR_SPEEDS);
            int32_t m1 = motor_speeds & 0xff;
            if (m1 > 127) {
                m1 -= 256;
            }
            target_speed = m1 << 5;
        }

        // Calculate speed.
        uint64_t delta_t_us = loop_start - last_time;
        int32_t this_delta_angle = wheel_angle - last_wheel_angle;
        if (this_delta_angle >= 2048) {
            this_delta_angle -= 4096;
        } else if (this_delta_angle < -2048) {
            this_delta_angle += 4096;
        }
        delta_angles[angle_idx] = this_delta_angle;
        angle_idx++;
        if (angle_idx==num_angles) {
            angle_idx = 0;
        }

        last_wheel_angle = wheel_angle;
        last_time = loop_start;

        // Measured values are up +/- 40.  Scale that into range with target_speed.
        int32_t delta_angle = 0;
        for (int i=0; i < num_angles; i++) {
            delta_angle += delta_angles[i];
        }
        const int angle_scale_fac = 50;
        delta_angle *= angle_scale_fac/num_angles;

        // The wheel angle is sampled at the start of the interval but we only
        // receive it at the end.  Compensate for that by adding on the delta.
        wheel_angle += delta_angle/angle_scale_fac;
        const int interps = 8;
        int32_t wheel_angle_inc = (delta_angle/angle_scale_fac)/interps;

        for (int i=0; i<interps; i++) {
            gpio_put(PIN_DEBUG, 1);
            wheel_angle &= 0xfff;

            // Using 0-4095 for our angle range.
            // Convert wheel angle to angle relative to pole of 
            // magnet.
            uint32_t meas_pole_angle = (wheel_angle * MOTOR_NUM_POLES/2) % 4096;
            pole_angle=(meas_pole_angle + 4096 + angle_offset + (target_speed>0?1024:(1024*3))) % 4096;

            if (i == 0) {
                if (target_speed > 0) {
                    throttle += target_speed - delta_angle;
                } else if (target_speed < 0) {
                    throttle += delta_angle - target_speed;
                } else {
                    throttle = 0;
                }
                
                if (throttle > 4095) throttle=4095;
                if (throttle < 0) throttle=0;
                
                if ((n % 64) == 0) {
                    printf("t=%04d v=%04d\n", target_speed, delta_angle);
                }
                n++;
            }

            u_int16_t duty_a = pwm_lut[(pole_angle) % LUT_LEN];
            u_int16_t duty_b = pwm_lut[(pole_angle + (LUT_LEN/3)) % LUT_LEN];
            u_int16_t duty_c = pwm_lut[(pole_angle + (2*LUT_LEN/3)) % LUT_LEN];

            duty_a = (duty_a * throttle) >> 12;
            duty_b = (duty_b * throttle) >> 12;
            duty_c = (duty_c * throttle) >> 12;

            pwm_set_chan_level(pwm_slice_a, pwm_chan_a, duty_a);
            pwm_set_chan_level(pwm_slice_b, pwm_chan_b, duty_b);
            pwm_set_chan_level(pwm_slice_c, pwm_chan_c, duty_c);
            
            // printf("%04d %04d %04d %04d %04d %04d\n", 
            //  wheel_angle, meas_pole_angle, pole_angle, duty_a, duty_b, duty_c);

            gpio_put(PIN_DEBUG, 0);

            wheel_angle += wheel_angle_inc;
            if (i<(interps-1)) {
                uint64_t target_time = loop_start + ((i+1)*delta_t_us/interps);
                uint64_t now = time_us_64();
                if (target_time > now) { 
                    sleep_us(target_time - now);
                }
            }
        }

        // Read buttons (which are pull-downs) and adjust target speed accordingly.
        if (!gpio_get(PIN_BUTT_A) && target_speed > -4095) {
            target_speed--;
        }
        if (!gpio_get(PIN_BUTT_B) && target_speed < 4095) {
            target_speed++;
        }
    }

    return 0;
}
