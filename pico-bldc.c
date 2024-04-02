#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/i2c_slave.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/divider.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "hardware/i2c.h"
#include "pico/multicore.h"
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

// I2C defines, first the port where we act as peripheral.
#define I2C_PERIPH_PORT i2c0
#define I2C_PERIPH_SDA 0   
#define I2C_PERIPH_SCL 1
#define I2C_PERIPH_ADDR 0x42

// I2C defines for the controler port, where we read the INA219.
#define I2C_CONT_PORT i2c1
#define I2C_CONT_SDA 2
#define I2C_CONT_SCL 3

#define I2C_INA219_ADDR 0x40
#define INA219_REG_CONF    0
#define INA219_REG_SHUNT_V 1
#define INA219_REG_BUS_V   2
#define INA219_REG_POWER   3
#define INA219_REG_CURRENT 4
#define INA219_REG_CALIB   5


typedef uint16_t i2c_reg_t;

static struct
{
    uint8_t addr;
    i2c_reg_t value;
    i2c_reg_t value_out;
    int num_bytes_received;
    bool addr_received;
    uint32_t volatile last_i2c_write_time;
} i2c_context = {};

enum I2CRegs {
    I2C_REG_CTRL,
    I2C_REG_STATUS,
    I2C_REG_WDOG_TIMEOUT_MS,
    I2C_REG_FAULT_COUNT,
    
    I2C_REG_MOT0_V,
    I2C_REG_MOT1_V,
    I2C_REG_MOT2_V,
    I2C_REG_MOT3_V,

    I2C_REG_MOT0_CALIB,
    I2C_REG_MOT1_CALIB,
    I2C_REG_MOT2_CALIB,
    I2C_REG_MOT3_CALIB,

    I2C_REG_BATT_V,  // LSB = 4mV
    I2C_REG_CURRENT, // LSB depend on calibration
    I2C_REG_POWER,   // LSB = 20 * I2C_REG_CURRENT LSB

    I2C_REG_TEMPERATURE,  // LSB = 0.01C

    I2C_REG_MOT0_TRAVEL,
    I2C_REG_MOT1_TRAVEL,
    I2C_REG_MOT2_TRAVEL,
    I2C_REG_MOT3_TRAVEL,

    I2C_REG_COUNT,
};

#define I2C_REG_CTRL_EN    (1<<0)
#define I2C_REG_CTRL_RUN   (1<<1)
#define I2C_REG_CTRL_CALIB (1<<2)
#define I2C_REG_CTRL_RESET (1<<3)
#define I2C_REG_CTRL_WDEN  (1<<4)

#define I2C_REG_STATUS_FAULT            (1<<0)
#define I2C_REG_STATUS_CALIBRATION_DONE (1<<1)
#define I2C_REG_STATUS_WDOG_EXPIRED     (1<<2)

static critical_section_t i2c_reg_lock;
static i2c_reg_t volatile i2c_registers[I2C_REG_COUNT] = {};

static inline i2c_reg_t i2c_reg_get(enum I2CRegs num) {
    critical_section_enter_blocking(&i2c_reg_lock);
    uint32_t out = i2c_registers[num];
    critical_section_exit(&i2c_reg_lock);
    return out;
}

static inline i2c_reg_t i2c_reg_get_and_clear_mask(enum I2CRegs num, i2c_reg_t mask) {
    critical_section_enter_blocking(&i2c_reg_lock);
    uint32_t out = i2c_registers[num];
    i2c_registers[num] &= ~mask;
    critical_section_exit(&i2c_reg_lock);
    return out & mask;
}

static inline void i2c_reg_set(enum I2CRegs num, i2c_reg_t value) {
    critical_section_enter_blocking(&i2c_reg_lock);
    i2c_registers[num] = value;
    critical_section_exit(&i2c_reg_lock);
}

static inline void i2c_reg_set_mask(enum I2CRegs num, i2c_reg_t mask) {
    critical_section_enter_blocking(&i2c_reg_lock);
    i2c_registers[num] |= mask;
    critical_section_exit(&i2c_reg_lock);
}

// Our handler is called from the I2C ISR, so it must complete quickly. Blocking calls /
// printing to stdio may interfere with interrupt handling.
static void i2c_periph_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
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
        i2c_write_byte_raw(i2c, (uint8_t)(i2c_context.value_out>>(sizeof(i2c_reg_t)*8 - 8)));
        i2c_context.value_out <<= 8;
        break;
    case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
        if (i2c_context.addr_received && 
            (i2c_context.addr < I2C_REG_COUNT) &&
            (i2c_context.num_bytes_received == sizeof(i2c_reg_t))) {
            // Valid address, received all bytes to write.
            switch (i2c_context.addr) {
            case I2C_REG_STATUS:
                // Special handling: write clears the bits that were written.
                i2c_reg_get_and_clear_mask(i2c_context.addr, i2c_context.value);
                break;
            default:
                i2c_reg_set(i2c_context.addr, i2c_context.value);
                break;
            }
            i2c_context.last_i2c_write_time = time_us_32();
        }
        i2c_context.addr_received = false;
        break;
    default:
        break;
    }
}

void core1_entry();

int main()
{
    stdio_init_all();

    printf("Pico-BLDC booting...\n");
    multicore_launch_core1(core1_entry);

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
    i2c_reg_set(I2C_REG_WDOG_TIMEOUT_MS, 2000);
    gpio_set_function(I2C_PERIPH_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_PERIPH_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_PERIPH_SDA);
    gpio_pull_up(I2C_PERIPH_SCL);
    i2c_init(I2C_PERIPH_PORT, 100*1000);
    i2c_slave_init(I2C_PERIPH_PORT, I2C_PERIPH_ADDR, &i2c_periph_handler);

    // Initialise global state for all motors.  This:
    // - Loads PIO programs.
    // - Sets up the motor sine() LUT.
    motor_global_init(pio0);

    uint32_t last_print = 0;
    while (1) {
        // Wait for the (active low) button to be pressed.
        if (!gpio_get(PIN_BUTT_A)) {
            break;
        }
        // Or for Enter on the UART.
        int chr = getchar_timeout_us(0);
        if (chr == '\r') {
            break;
        } else if (chr >= 0) {
            printf("Unexpected char: %x\n", chr);
        }
        // Or for I2C control to be enabled.
        if (i2c_reg_get(I2C_REG_CTRL) & I2C_REG_CTRL_EN) {
            printf("I2C enabled\n");
            break;
        }

        uint32_t now = time_us_32();
        if ((now - last_print) > 1000000) {
            printf("Waiting for Enter, button A, or I2C_REG_CTRL bit 0...\n");
            last_print = now;
        } 
    }

    // Start closed-loop control.
 #define NUM_MOTORS 4
    struct motor_cb m[NUM_MOTORS] = {};

    while (1) {
        if (gpio_get(PIN_MOTOR_NFAULT)) {
            break;
        }
        uint32_t now = time_us_32();
        if ((now - last_print) > 1000000) {
            printf("Motor fault; is motor power on?\n");
            last_print = now;
        } 
    }

    printf("Initialising motor control blocks...\n");
    motor_init(&m[0], PIN_MOT0_A, PIN_MOT0_B, PIN_MOT0_C, PIN_PWM0_IN);
    motor_init(&m[1], PIN_MOT1_A, PIN_MOT1_B, PIN_MOT1_C, PIN_PWM1_IN);
    motor_init(&m[2], PIN_MOT2_A, PIN_MOT2_B, PIN_MOT2_C, PIN_PWM2_IN);
    motor_init(&m[3], PIN_MOT3_A, PIN_MOT3_B, PIN_MOT3_C, PIN_PWM3_IN);

    printf("Syncing PWMs...\n");
    motor_enable_pwms();
    
    // If I2C control is _disabled_ or the I2C "do calibration" flag is 
    // set.
    if (!(i2c_reg_get(I2C_REG_CTRL) & I2C_REG_CTRL_EN) || 
         (i2c_reg_get(I2C_REG_CTRL) & I2C_REG_CTRL_CALIB)) {
calibrate:
        i2c_reg_get_and_clear_mask(I2C_REG_CTRL, I2C_REG_CTRL_CALIB);
        gpio_put(PIN_MOTOR_NRESET, 1); // Enable motors 
        for (int i = 0; i < NUM_MOTORS; i++) {
            printf("Calibrating motor %d...\n", i);
            motor_calibrate(&m[i]);
            uint16_t calibration_data = motor_get_calibration(&m[i]);
            i2c_reg_set(I2C_REG_MOT0_CALIB+i, calibration_data);
        }
        i2c_reg_set_mask(I2C_REG_STATUS, I2C_REG_STATUS_CALIBRATION_DONE);
        printf("Calibration done.\n");

        // Calibration takes a while, make sure we don't trigger the watchdog 
        // immediately.
        i2c_context.last_i2c_write_time = time_us_32();
    }

    int n = 0;
    uint num_faults = 0;
    uint32_t last_fault_report = 0;
    uint32_t last_speed_report = 0;
    while (true) {
        i2c_reg_t i2c_ctrl = i2c_reg_get(I2C_REG_CTRL);
        if (i2c_ctrl & I2C_REG_CTRL_EN) {
            // I2C control is enabled.
            if (i2c_reg_get_and_clear_mask(I2C_REG_CTRL, I2C_REG_CTRL_CALIB)) {
                // Told to do a calibration.
                printf("I2C says to do calibration...\n");
                goto calibrate;
            }
            for (int i = 0; i< NUM_MOTORS; i++) {
                motor_restore_calibration(&m[i], i2c_reg_get(I2C_REG_MOT0_CALIB+i));
                i2c_reg_set(I2C_REG_MOT0_TRAVEL+i, m[i].distance_traveled<<8);
            }

            if ((i2c_ctrl & I2C_REG_CTRL_WDEN) && (i2c_ctrl & I2C_REG_CTRL_RUN)) {
                // Watchdog is enabled. Check it.

                // Must read last_write_time first since an interrupt could
                // update it at any point.
                uint32_t lwt = i2c_context.last_i2c_write_time;
                uint32_t now = time_us_32();
                uint32_t us_since_last_i2c_write = now - lwt;

                uint32_t timeout = (uint32_t)i2c_reg_get(I2C_REG_WDOG_TIMEOUT_MS) * 1000;
                if (us_since_last_i2c_write > timeout) {
                    puts("Watchdog expired!!!\n");
                    i2c_ctrl &= (~I2C_REG_CTRL_RUN);
                    i2c_reg_get_and_clear_mask(I2C_REG_CTRL, I2C_REG_CTRL_RUN);
                    i2c_ctrl |= I2C_REG_CTRL_RESET;
                    i2c_reg_set_mask(I2C_REG_STATUS, I2C_REG_STATUS_WDOG_EXPIRED);
                }
            }

            if (i2c_ctrl & I2C_REG_CTRL_RESET) {
                puts("Reset motor speeds.\n");
                i2c_reg_get_and_clear_mask(I2C_REG_CTRL, I2C_REG_CTRL_RESET);
                for (int i = 0; i < NUM_MOTORS; i++) {
                    i2c_reg_set(I2C_REG_MOT0_V+i, 0);
                }
            }

            if (i2c_ctrl & I2C_REG_CTRL_RUN) {
                // Motor power enabled.
                for (int i = 0; i < NUM_MOTORS; i++) {
                    motor_set_v(&m[i], i2c_reg_get(I2C_REG_MOT0_V+i));
                }
                gpio_put(PIN_MOTOR_NRESET, 1); // high = run
            } else {
                // Motor power disabled.
                for (int i = 0; i < NUM_MOTORS; i++) {
                    motor_set_v(&m[i], 0);
                }
                gpio_put(PIN_MOTOR_NRESET, 0); // low = hold in reset.
            }
        } else {
            // Read buttons (which are pull-downs) and adjust target speed accordingly.
            int chr = getchar_timeout_us(0);
            if ((!gpio_get(PIN_BUTT_A) || chr == '.') && m[3].target_velocity > -20) {
                m[0].target_velocity -= fix15c(0.1);
                m[1].target_velocity -= fix15c(0.2);
                m[2].target_velocity -= fix15c(0.3);
                m[3].target_velocity -= fix15c(0.4);
                print_fix15("v", m[3].target_velocity);
                printf("\n");
            }
            if ((!gpio_get(PIN_BUTT_B) || chr == ',') && m[3].target_velocity < 20) {
                m[0].target_velocity += fix15c(0.1);
                m[1].target_velocity += fix15c(0.2);
                m[2].target_velocity += fix15c(0.3);
                m[3].target_velocity += fix15c(0.4);
                print_fix15("v", m[3].target_velocity);
                printf("\n");
            }
            gpio_put(PIN_MOTOR_NRESET, 1); // high = run
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

        if (!gpio_get(PIN_MOTOR_NFAULT)) {
            num_faults++;
            i2c_reg_set(I2C_REG_FAULT_COUNT, (i2c_reg_t)num_faults);
            if ((time_us_32() - last_fault_report) > 1000000) {
                printf("Motor fault (count=%d)!\n", num_faults);
                last_fault_report = time_us_32();
            }
            i2c_reg_set_mask(I2C_REG_STATUS, I2C_REG_STATUS_FAULT);
        } else {
            i2c_reg_get_and_clear_mask(I2C_REG_STATUS, I2C_REG_STATUS_FAULT);
        }
        if ((time_us_32() - last_speed_report) > 1000000) {
            //print_fix15("pv", m[3].est_pole_v/11);
            //printf("\n");
            last_speed_report = time_us_32();
        }
    }

    return 0;
}

static int i2c_write_16(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint16_t value) {
    uint8_t buf[3] = {reg, (uint8_t)(value >> 8), (uint8_t)value};
    int count = i2c_write_blocking(i2c, addr, buf, 3, false);
    if (count < 0) {
        puts("Couldn't write to I2C, please check wiring!");
        return count;
    }
    return 0;
}

static int i2c_read_16(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint16_t *result) {
    uint8_t buf[2];
    buf[0] = reg;
    // Write address on the bus, holding the trasactino open.
    int count = i2c_write_blocking(i2c, addr, buf, 1, true);
    if (count < 0) {
        puts("Couldn't write to I2C, please check wiring!");
        return count;
    }
    // Read back two bytes.
    count = i2c_read_blocking(i2c, addr, buf, 2, false);
    if (count < 0) {
        puts("Couldn't read I2C, please check wiring!");
        return count;
    }
    // I2C is generally big-endian.
    *result = ((uint16_t)buf[0])<<8 | buf[1];
    return 0;
}

float read_onboard_temperature() {
    // From the Pico examples...
    /* 12-bit conversion, assume max value == ADC_VREF == 3.3 V */
    const float conversionFactor = 3.3f / (1 << 12);
    float adc = (float)adc_read() * conversionFactor;
    float tempC = 27.0f - (adc - 0.706f) / 0.001721f;
    return tempC;
}

void core1_entry() {
    printf("Second core booting...\n");

    // Set up our I2C controller port, which we use to 
    // read the INA219.
    gpio_set_function(I2C_CONT_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_CONT_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_CONT_SDA);
    gpio_pull_up(I2C_CONT_SCL);
    i2c_init(I2C_CONT_PORT, 100*1000);

    // Init the ADC, which we use to read the internal temperature.
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);

    float temp_c = read_onboard_temperature();

#define INA219_R_SHUNT 0.05
#define INA219_MAX_EXPECTED_CURRENT 6.0
#define INA219_CURRENT_LSB  (INA219_MAX_EXPECTED_CURRENT / 32768)
    uint16_t ina_cal = (uint16_t)(0.04096 / (INA219_CURRENT_LSB * INA219_R_SHUNT));
    uint8_t buf[3] = {INA219_REG_CALIB, ina_cal>>8, ina_cal};
    bool done_calib = false;
    uint32_t last_print = 0;
    while (true) {
        if (!done_calib) {
            uint16_t conf = 0x3b9f;
            int err = i2c_write_16(I2C_CONT_PORT, I2C_INA219_ADDR, 
                INA219_REG_CONF, conf);
            if (err) {
                puts("Couldn't write to INA219, please check wiring!");
                continue;
            } 
            printf("Writing INA219 calibration word: %d\n", ina_cal);
            err = i2c_write_16(I2C_CONT_PORT, I2C_INA219_ADDR, 
                INA219_REG_CALIB, ina_cal);
            if (err) {
                puts("Couldn't write to INA219, please check wiring!");
                continue;
            } 
            done_calib = true;
        }

        int16_t shunt_v;
        int err = i2c_read_16(I2C_CONT_PORT, I2C_INA219_ADDR, INA219_REG_SHUNT_V, &shunt_v);
        if (err) {
            continue;
        }
        int16_t batt_v;
        err = i2c_read_16(I2C_CONT_PORT, I2C_INA219_ADDR, INA219_REG_BUS_V, &batt_v);
        if (err) {
            continue;
        }
        batt_v = batt_v >> 3; // Ignore control bits.
        i2c_reg_set(I2C_REG_BATT_V, batt_v);

        int16_t current;
        err = i2c_read_16(I2C_CONT_PORT, I2C_INA219_ADDR, INA219_REG_CURRENT, &current);
        if (err) {
            continue;
        }
        i2c_reg_set(I2C_REG_CURRENT, current);

        int16_t power;
        err = i2c_read_16(I2C_CONT_PORT, I2C_INA219_ADDR, INA219_REG_POWER, &power);
        if (err) {
            continue;
        }
        i2c_reg_set(I2C_REG_POWER, power);

        temp_c = read_onboard_temperature() * 0.05 + temp_c * 0.94;
        i2c_reg_set(I2C_REG_TEMPERATURE, (i2c_reg_t)(temp_c * 100));

        uint32_t now = time_us_32();
        if ((now - last_print) >= 1000000) {
            printf("Temp: %.1fC bus_v: %.2fV shunt_v: %.3fV current: %.3fA power: %.2fW\n", 
                temp_c, batt_v * 0.004f, shunt_v * 0.00001f, current * 0.0001831054688f , power * 0.003662109375f);
            last_print = now;    
        }
        sleep_ms(10);
    }
}