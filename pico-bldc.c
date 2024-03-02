#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/i2c_slave.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/divider.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "hardware/i2c.h"
#include "pico/critical_section.h"
#include "hardware/pio.h"

#include "motor.h" // Must be last; includes stdfix.h, conflicts with SDK

// GPIO defines
#define PIN_DEBUG 6

#define PIN_PWM0_IN 4
#define PIN_PWM1_IN 5
#define PIN_PWM2_IN 26
#define PIN_PWM3_IN 22

#define PIN_MOTOR_NRESET 28
#define PIN_MOTOR_NFAULT 27

#define PIN_MOT0_A 10
#define PIN_MOT0_B 12
#define PIN_MOT0_C 11

#define PIN_MOT1_A 13
#define PIN_MOT1_B 14
#define PIN_MOT1_C 15

#define PIN_MOT2_A 16
#define PIN_MOT2_B 17
#define PIN_MOT2_C 18

#define PIN_MOT3_A 19
#define PIN_MOT3_B 20
#define PIN_MOT3_C 21

#define PIN_BUTT_A 7
#define PIN_BUTT_B 8

// I2C defines
#define I2C_PORT i2c0
#define I2C_SDA 0   
#define I2C_SCL 1
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

    printf("Pico-BLDC booting...\n");

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

    // Shared motor reset/fault pins
    gpio_init(PIN_MOTOR_NFAULT);
    gpio_set_dir(PIN_MOTOR_NFAULT, GPIO_IN);
    gpio_pull_up(PIN_MOTOR_NFAULT);

    gpio_init(PIN_MOTOR_NRESET);
    gpio_set_dir(PIN_MOTOR_NRESET, GPIO_OUT);
    gpio_put(PIN_MOTOR_NRESET, 1); // high = run

    // I2C Initialisation.
    critical_section_init(&i2c_reg_lock);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    i2c_init(I2C_PORT, 100*1000);
    i2c_slave_init(I2C_PORT, I2C_SLAVE_ADDRESS, &i2c_slave_handler);

    // Initialise global state for all motors.  This:
    // - Loads PIO programs.
    // - Sets up the motor sine() LUT.
    motor_global_init(pio0);

    uint32_t last_print = 0;
    while (gpio_get(PIN_BUTT_A)) {
        // Wait for the (active low) button to be pressed.
        uint32_t now = time_us_32();
        if ((now - last_print) > 1000000) {
            printf("Waiting for button...\n");
            last_print = now;
        } 
    }

    // Start closed-loop control.
    struct motor_cb m[4] = {};

    // FIXME Temporary code to drive all motor pins low.
    for (int pin = 10; pin <= 21; pin++) {
        gpio_init(pin);
        gpio_set_dir(pin, 1);
        gpio_put(pin, 0);
    }

    motor_init(&m[0], PIN_MOT0_A, PIN_MOT0_B, PIN_MOT0_C, PIN_PWM0_IN);
    motor_init(&m[1], PIN_MOT1_A, PIN_MOT1_B, PIN_MOT1_C, PIN_PWM1_IN);
    motor_init(&m[2], PIN_MOT2_A, PIN_MOT2_B, PIN_MOT2_C, PIN_PWM2_IN);
    motor_init(&m[3], PIN_MOT3_A, PIN_MOT3_B, PIN_MOT3_C, PIN_PWM3_IN);
    motor_enable_pwms();
    
    motor_calibrate(&m[0]);
    motor_calibrate(&m[1]);
    motor_calibrate(&m[2]);
    motor_calibrate(&m[3]);

    int n = 0;
    uint num_faults = 0;
    uint32_t last_fault_report = 0;
    uint32_t last_speed_report = 0;
    while (true) {
        if (i2c_reg_get(I2C_REG_CTRL) & 1) {
            // I2C control is enabled.
            uint32_t motor_speeds = i2c_reg_get(I2C_REG_MOTOR_SPEEDS);
            int32_t m1 = motor_speeds & 0xff;
            if (m1 > 127) {
                m1 -= 256;
            }
            m[2].target_velocity = ((fix15_t)m1)/10;
        }

        uint32_t start_time = time_us_32();
        while ((time_us_32()-start_time) < 1000) {
            for (int m_out_idx = 0; m_out_idx<4; m_out_idx++) {
                // Poll all four motors to see if they have updated sensor
                // inputs.  If there's no update, this returns immediately
                // so effectively, we prioritise sensor updates if there 
                // are any, then default to updating outputs.
                for (int m_upd_idx = 0; m_upd_idx<4; m_upd_idx++) {
                    motor_update(&m[m_upd_idx]);
                }
                // Then update one motor per loop.
                motor_update_output(&m[m_out_idx]);
            }
            gpio_put(PIN_DEBUG, n++&1);
        }

        // Read buttons (which are pull-downs) and adjust target speed accordingly.
        if (!gpio_get(PIN_BUTT_A) && m[2].target_velocity > -20) {
            m[0].target_velocity -= fix15c(0.005);
            m[1].target_velocity -= fix15c(0.005);
            m[2].target_velocity -= fix15c(0.005);
            m[3].target_velocity -= fix15c(0.005);
            print_fix15("v", m[2].target_velocity);
            printf("\n");
        }
        if (!gpio_get(PIN_BUTT_B) && m[2].target_velocity < 20) {
            m[0].target_velocity += fix15c(0.005);
            m[1].target_velocity += fix15c(0.005);
            m[2].target_velocity += fix15c(0.005);
            m[3].target_velocity += fix15c(0.005);
            print_fix15("v", m[2].target_velocity);
            printf("\n");
        }
        if (!gpio_get(PIN_MOTOR_NFAULT)) {
            num_faults++;
            if ((time_us_32() - last_fault_report) > 1000000) {
                printf("Motor fault (count=%d)!\n", num_faults);
                last_fault_report = time_us_32();
            }
        }
        if ((time_us_32() - last_speed_report) > 1000000) {
            print_fix15("pv", m[2].est_pole_v/11);
            printf("\n");
            last_speed_report = time_us_32();
        }
    }

    return 0;
}
