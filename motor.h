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
#include "pwm.pio.h"

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

static inline uint32_t bitsk(fix15_t x) {
    union {
        fix15_t f;
        uint32_t y;
    } u;
    u.f = x;
    return u.y;
} 

static inline void print_fix15(char *msg, fix15_t v) {
    printf("%s=%+f ", msg, (float)v);
}

static inline fix15_t clamp_angle(fix15_t a) {
    uint32_t au32 = *(uint32_t*)(&a);
    au32 &= 0x7fff;
    return *(fix15_t*)(&au32);
}

#define ANGLE_RING_BUF_SIZE 4
struct motor_cb {
    fix15_t target_velocity;  // Revs/s
    fix15_t angle_offset;     // Calibration offset to align poles.

    PIO pio;
    uint sm_pwm;

    uint pwm_slice_a, pwm_slice_b, pwm_slice_c;
    uint pwm_chan_a, pwm_chan_b, pwm_chan_c;

    fix15_t distance_traveled; // 1.0 = 1 rotation.
    fix15_t est_pole_v;
    fix15_t last_pole_angle;
    uint32_t last_angle_upd_time;
    fix15_t drive_angle_offset;
    //fix15_t estimated_accel;    // Revs/sec sq

    fix15_t output_throttle;    // -1.0 to 1.0

    struct {
        uint32_t time_us;
        fix15_t angle;  // Range 0.0-1.0
    } measured_angles_ring_buf[ANGLE_RING_BUF_SIZE];
    int angle_buf_idx;

    volatile uint32_t next_raw_high;
    volatile uint32_t next_timestamp;

    bool angle_meas_pending;
    bool output_update_pending;
};

void motor_global_init(PIO p);
void motor_enable_pwms();
void motor_init(struct motor_cb *cb, uint pin_a, uint pin_b, uint pin_C, uint pin_pwm_in);
void motor_calibrate(struct motor_cb *cb);
uint16_t motor_get_calibration(struct motor_cb *cb);
void motor_restore_calibration(struct motor_cb *cb, uint16_t);
void motor_set_v(struct motor_cb *cb, int16_t);
void motor_record_pwm_reading(struct motor_cb *cb, uint32_t raw_pio_output, uint32_t timestamp);
void motor_record_angle_meas(struct motor_cb *cb, fix15_t angle, uint32_t timestamp);
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