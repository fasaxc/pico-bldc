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

#include "motor.h" // Must be last; includes stdfix.h, conflicts with SDK

// GPIO defines
#define PIN_DEBUG 0
#define PIN_PWM_IN 18

#define PIN_MOTOR_A 16
#define PIN_MOTOR_B 15
#define PIN_MOTOR_C 14

#define PIN_BUTT_A 3
#define PIN_BUTT_B 5

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

    while (gpio_get(PIN_BUTT_A)) {
        // Wait for the (active low) button to be pressed.
    }

    // Start closed-loop control.
    struct motor_cb m = {};
    motor_init(&m, PIN_MOTOR_A, PIN_MOTOR_B, PIN_MOTOR_C, PIN_PWM_IN);
    motor_calibrate(&m);

    while (true) {
        uint32_t raw_pio_high = drain_pio_fifo_blocking(pio0, m.sm_high);
        uint32_t reading_time = time_us_32();
        gpio_put(PIN_DEBUG, 1);
        uint32_t raw_pio_invl = drain_pio_fifo_non_block(pio0, m.sm_invl);
        
        // Note flipped order vs the reads; want to feed in the interval
        // first.
        if (raw_pio_invl != 0) {
            motor_record_pwm_interval(&m, raw_pio_invl);
        }
        if (raw_pio_high != 0) {
            motor_record_pwm_high_time(&m, raw_pio_high);
        }

        if (i2c_reg_get(I2C_REG_CTRL) & 1) {
            // I2C control is enabled.
            uint32_t motor_speeds = i2c_reg_get(I2C_REG_MOTOR_SPEEDS);
            int32_t m1 = motor_speeds & 0xff;
            if (m1 > 127) {
                m1 -= 256;
            }
            m.target_velocity = ((fix15_t)m1)/10;
        }

        while ((time_us_32()-reading_time) < 900) {
            m.output_update_pending = true; // Force it for now.
            motor_update(&m);
        }

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
