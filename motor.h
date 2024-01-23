#ifndef _MOTOR_H_
#define _MOTOR_H_

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

    PIO pio;
    uint sm_high;
    uint sm_invl;

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

void motor_global_init(PIO p);
void motor_init(struct motor_cb *cb, uint pin_a, uint pin_b, uint pin_C, uint pin_pwm_in);
void motor_calibrate(struct motor_cb *cb);
void motor_record_pwm_interval(struct motor_cb *cb, uint32_t raw_pio_output);
void motor_record_pwm_high_time(struct motor_cb *cb, uint32_t raw_pio_output);
void motor_record_angle_meas(struct motor_cb *cb, fix15_t angle);
void motor_process_angle_meas(struct motor_cb *cb);
void motor_set_pwms(struct motor_cb *cb, fix15_t drive_angle);
void motor_update_output(struct motor_cb *cb);
void motor_update(struct motor_cb *cb);

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

#endif