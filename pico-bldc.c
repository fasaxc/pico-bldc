#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/divider.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
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
// #define I2C_PORT i2c0
// #define I2C_SDA 8
// #define I2C_SCL 9


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

    while (!pio_sm_get_rx_fifo_level(pio, sm_invl)) {
        // Wait for the first interval measurement to arrive so
        // we'll definitely load it on the first loop below.
    }

    while (gpio_get(PIN_BUTT_A)) {
    }

    // Determine angle offset of the motor.  Do a slow round of open loop 
    // control to capture the rotor and then read the position.
    const uint fp_bits = 12; // Fixed point bits to use for division
    uint32_t pole_angle = 0;
    uint32_t angle_offset = 2048;
    uint32_t meas_pole_angle = 0;
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
        uint32_t wheel_angle = ((high + half_clock) / one_clock) - 16;
        if (wheel_angle > 4095) {
            wheel_angle = 0;
        }

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
    uint32_t throttle = 0x8ff;
    while (true) {
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
        uint32_t wheel_angle = ((high + half_clock) / one_clock) - 16;
        if (wheel_angle > 4095) {
            wheel_angle = 0;
        }

        // Using 0-4095 for our angle range.
        // Convert wheel angle to angle relative to pole of 
        // magnet.
        uint32_t meas_pole_angle = (wheel_angle * MOTOR_NUM_POLES/2) % 4096;
        pole_angle=meas_pole_angle + 4096 + angle_offset + 1024;
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
        
        // Read buttons (which are pull-downs) and adjust angle offset accordingly.
        if (!gpio_get(PIN_BUTT_A) && throttle > 0) {
            throttle--;
        }
        if (!gpio_get(PIN_BUTT_B) && throttle < 0xfff) {
            throttle++;
        }

        gpio_put(PIN_DEBUG, 0);
    }

    // // SPI initialisation. This example will use SPI at 1MHz.
    // spi_init(SPI_PORT, 1000*1000);
    // gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    // gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    // gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    // gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    // // Chip select is active-low, so we'll initialise it to a driven-high state
    // gpio_set_dir(PIN_CS, GPIO_OUT);
    // gpio_put(PIN_CS, 1);
    

    // // I2C Initialisation. Using it at 400Khz.
    // i2c_init(I2C_PORT, 400*1000);
    
    // gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    // gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    // gpio_pull_up(I2C_SDA);
    // gpio_pull_up(I2C_SCL);


    return 0;
}
