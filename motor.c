#include "motor.h"
#include "hardware/dma.h"
#include "hardware/timer.h"

#define MOTOR_NUM_POLES 22

// PWM_TOP = 3100 gives roughly 40kHz.
#define PWM_TOP 3100
#define LUT_LEN 4096
static u_int16_t pwm_lut[LUT_LEN];

static PIO pio;
static uint pio_offset_pwm;

static uint32_t pwm_slice_mask;

void motor_global_init(PIO p) {
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

    // Load PIO programs to measure PWM interval and high time
    // from the motor position sensor.
    pio = p;
    pio_offset_pwm = pio_add_program(pio, &pwm_program);
}

void motor_enable_pwms() {
    printf("Enabling PWM slices: %x\n", pwm_slice_mask);
    pwm_set_mask_enabled(pwm_slice_mask);
}

void motor_init(struct motor_cb *cb, uint pin_a, uint pin_b, uint pin_c, uint pin_pwm_in) {
    // Configure motor PWM outputs.
    pwm_config pwm_c = pwm_get_default_config();
    pwm_config_set_wrap(&pwm_c, PWM_TOP);
    pwm_config_set_clkdiv_int(&pwm_c, 1);
    pwm_config_set_phase_correct(&pwm_c, 1);

    gpio_set_function(pin_a, GPIO_FUNC_PWM);
    cb->pwm_slice_a = pwm_gpio_to_slice_num(pin_a);
    cb->pwm_chan_a = pwm_gpio_to_channel(pin_a);
    pwm_init(cb->pwm_slice_a, &pwm_c, false);

    gpio_set_function(pin_b, GPIO_FUNC_PWM);
    cb->pwm_slice_b = pwm_gpio_to_slice_num(pin_b);
    cb->pwm_chan_b = pwm_gpio_to_channel(pin_b);
    pwm_init(cb->pwm_slice_b, &pwm_c, false);

    gpio_set_function(pin_c, GPIO_FUNC_PWM);
    cb->pwm_slice_c = pwm_gpio_to_slice_num(pin_c);
    cb->pwm_chan_c = pwm_gpio_to_channel(pin_c);
    pwm_init(cb->pwm_slice_c, &pwm_c, false);

    // Record which PWMs we're using so they can be enabled in sync.
    pwm_slice_mask |= ((1<<cb->pwm_slice_a) | 
                       (1<<cb->pwm_slice_b) |  
                       (1<<cb->pwm_slice_c));

    // Set up the PIO state machines.  We can use the same programs
    // for every motor instance.
    cb->sm_pwm = pio_claim_unused_sm(pio, true);
    printf("Init PIO on SM %d pin %d\n", cb->sm_pwm, pin_pwm_in);
    pwm_program_init(pio, cb->sm_pwm, pio_offset_pwm, pin_pwm_in);
    
    // Set up a pair of DMAs, the first to copy HIGH times out of the PIO,
    // the second to write a timestamp.  Chain them to each other so that 
    // they ping-pong.
    int pio_chan = dma_claim_unused_channel(true);
    int time_chan = dma_claim_unused_channel(true);

    {
        dma_channel_config c = dma_channel_get_default_config(pio_chan);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_read_increment(&c, false);
        channel_config_set_write_increment(&c, false);
        int dreq;
        if (pio == pio0) {
            dreq = DREQ_PIO0_RX0 + cb->sm_pwm;
        } else {
            dreq = DREQ_PIO1_RX0 + cb->sm_pwm;
        }
        channel_config_set_dreq(&c, dreq); 
        channel_config_set_chain_to(&c, time_chan);    

        dma_channel_configure(
            pio_chan,               // Channel to be configured
            &c,                     // The configuration we just created
            NULL,                   // Write address
            &pio->rxf[cb->sm_pwm], // Read address
            1,             // Number of transfers.
            false          // Don't start yet.
        );
    }
    {
        dma_channel_config c = dma_channel_get_default_config(time_chan);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_read_increment(&c, false);
        channel_config_set_write_increment(&c, false);
        channel_config_set_chain_to(&c, pio_chan);

        dma_channel_configure(
            time_chan,              // Channel to be configured
            &c,                     // The configuration we just created
            &cb->next_timestamp,    // Write address
            &timer_hw->timerawl,    // Read address
            1,             // Number of transfers.
            false          // Don't start yet.
        );
    }

    // Trigger the PIO DMA.
    dma_channel_set_write_addr(pio_chan, &cb->next_raw_high, true);
}

void motor_calibrate(struct motor_cb *cb) {
    // TODO Port the calibration code to the same fixed-point approach as the main loop.

    // Get some values to make sure the PWM signal is working...
    printf("Starting calibration...\n");
    for (int i=0; i<2; i++) {
        printf("Wait on PWM reading...\n");
        uint32_t v = pio_sm_get_blocking(pio, cb->sm_pwm);
        uint32_t high = 0xffff - (v>>16);
        uint32_t invl = 0xffff - (v & 0xffff);
        printf("%08x -> high=%06d invl=%06d\n", v, high, invl);
    }

    // Determine angle offset of the motor.  Do a slow round of open loop 
    // control to capture the rotor and then read the position.
    const uint fp_bits = 12; // Fixed point bits to use for division
    uint32_t pole_angle = 0;
    uint32_t angle_offset = 2048;
    uint32_t meas_pole_angle = 0;
    for (pole_angle = 0; pole_angle < 256; pole_angle++) {
        // Load the PWM high time from the PIO.
        uint32_t raw = drain_pio_fifo_blocking(pio, cb->sm_pwm);
        uint32_t high = 0xffff - (raw>>16);
        uint32_t invl = 0xffff - (raw & 0xffff);
        uint32_t one_clock = (invl << fp_bits) / (16+4095+8);
        uint32_t half_clock = one_clock / 2;

        // Convert to fixed point for better precision.
        high = high << fp_bits;
        // Convert to angle (in 4096ths of a circle).
        int32_t wheel_angle = (((high + half_clock) / one_clock) - 16) & 0xfff;
        
        // Using 0-4095 for our angle range.
        // Convert wheel angle to angle relative to pole of 
        // magnet.
        meas_pole_angle = (wheel_angle * MOTOR_NUM_POLES/2) % 4096;

        if ((pole_angle % 8) == 0) {
            printf("%04d %04d\n", pole_angle, wheel_angle);
        }

        u_int16_t duty_a = pwm_lut[(pole_angle) % LUT_LEN];
        u_int16_t duty_b = pwm_lut[(pole_angle + (LUT_LEN/3)) % LUT_LEN];
        u_int16_t duty_c = pwm_lut[(pole_angle + (2*LUT_LEN/3)) % LUT_LEN];

        pwm_set_chan_level(cb->pwm_slice_a, cb->pwm_chan_a, duty_a/2);
        pwm_set_chan_level(cb->pwm_slice_b, cb->pwm_chan_b, duty_b/2);
        pwm_set_chan_level(cb->pwm_slice_c, cb->pwm_chan_c, duty_c/2);
    }

    angle_offset = (pole_angle - meas_pole_angle) % 4096;

    pwm_set_chan_level(cb->pwm_slice_a, cb->pwm_chan_a, 0);
    pwm_set_chan_level(cb->pwm_slice_b, cb->pwm_chan_b, 0);
    pwm_set_chan_level(cb->pwm_slice_c, cb->pwm_chan_c, 0);
    
    printf("Pole angle: %04d meas_pole_angle: %04d angle_offset: %04d\n", 
        pole_angle, meas_pole_angle, angle_offset);

    cb->angle_offset = ((fix15_t)angle_offset)/4096/(MOTOR_NUM_POLES/2);
    print_fix15("Angle offset", cb->angle_offset);
}

uint16_t motor_get_calibration(struct motor_cb *cb) {
    uint32_t angle_offset_bits = bitsk(cb->angle_offset);
    // Offset should be <1, so it should fit in the bottom 15 bits.
    return (uint16_t)angle_offset_bits;
}

void motor_restore_calibration(struct motor_cb *cb, uint16_t c) {
    int16_t c_signed = (int16_t)c;
    int32_t c_ext = c;
    uint32_t angle_offset_bits = (uint32_t)(c_ext);
    cb->angle_offset = kbits(angle_offset_bits);
}

void motor_set_v(struct motor_cb *cb, int16_t v) {
    // If we just put the bits of an int16 into the low bits
    // of a fix15_t then we'd get a range of -1.0 to 1.0.  Shift
    // by 5 gives a range of -32 to 32 RPS.
    int32_t v_ext = v << 5;
    cb->target_velocity = kbits((uint32_t)v_ext);
}

void motor_record_pwm_reading(struct motor_cb *cb, uint32_t raw_pio_output, uint32_t timestamp) {
    uint32_t high = 0xffff - (raw_pio_output>>16);
    high <<= 12;
    uint32_t invl = 0xffff - (raw_pio_output & 0xffff);
    if ((invl < 30000) || (invl > 45000)) {
        // 38000 is typical.
        printf("Bad invl! %d / %d\n", high, invl);
        return;
    }
    invl <<= 12;
    uint32_t one_clock = invl / 
        (16/*start bits*/ + 4095/*data bits*/ + 8/*end bits*/);
    // Integer division truncates; add half a clock to get
    // round-to-nearest.
    uint32_t half_clock = one_clock / 2; 
    uint32_t clocks = (high+half_clock)/one_clock; 
    if (clocks < 13) {
        // 16 should be the minimum, less than means "error".
        printf("PWM error! %d / %d\n", high, invl);
        return;
    }
    //printf("h=%d i=%d ", high, invl);
    fix15_t angle = (fix15_t)clocks - fix15c(16);
    //print_fix15("raw", angle);
    angle >>= 12;
    angle += cb->angle_offset;
    angle = clamp_angle(angle);
    motor_record_angle_meas(cb, angle, timestamp);
}

void motor_record_angle_meas(struct motor_cb *cb, fix15_t angle, uint32_t now) {
    int next_idx = cb->angle_buf_idx+1;
    if (next_idx == ANGLE_RING_BUF_SIZE) {
        next_idx = 0;
    }
    cb->measured_angles_ring_buf[next_idx].time_us = now;
    cb->measured_angles_ring_buf[next_idx].angle = angle;
    cb->angle_buf_idx = next_idx;
    cb->angle_meas_pending = true;
}

// Using kbits effectively divides by 2^15 so, to get seconds,
// we need to multiply by 2^15 / 1,000,000.
#define USEC_TO_FIX15_S(us) (kbits(us) * fix15c(0.032768))

void motor_process_angle_meas(struct motor_cb *cb) {
    int idx = cb->angle_buf_idx;
    int tap = 1;
    int prev_idx = (idx-tap)&(ANGLE_RING_BUF_SIZE-1);
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
    cb->distance_traveled += delta_angle;

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

    fix15_t v_error = -(cb->target_velocity - est_v);
    cb->output_throttle += v_error * fix15c(0.02);
#define MAX_THROTTLE fix15c(1)
    if (cb->output_throttle > MAX_THROTTLE) {
        cb->output_throttle = MAX_THROTTLE;
    } else if (cb->output_throttle < -MAX_THROTTLE) {
        cb->output_throttle = -MAX_THROTTLE;
    }

    cb->last_pole_angle = current_angle * (MOTOR_NUM_POLES/2);
    cb->last_angle_upd_time = current_time;
    cb->est_pole_v = est_v * (MOTOR_NUM_POLES/2);
    if (cb->output_throttle < 0) {
        cb->drive_angle_offset = fix15c(0.25);
    } else if (cb->output_throttle > 0) {
        cb->drive_angle_offset = fix15c(-0.25);
    } else {
        cb->drive_angle_offset = 0;
    }
}

void motor_update(struct motor_cb *cb) {
    int idx = cb->angle_buf_idx;
    uint32_t last_reading_time = cb->measured_angles_ring_buf[idx].time_us;
    uint32_t next_raw_pwm = cb->next_raw_high;
    uint32_t next_timestamp = cb->next_timestamp;
    if (next_timestamp == last_reading_time) {
        return;
    }
    motor_record_pwm_reading(cb, next_raw_pwm, next_timestamp);
    motor_process_angle_meas(cb);
}

// Nature of the PWM signal means we're always 1ms behind.
#define READING_DELAY_US 1000

void motor_update_output(struct motor_cb *cb) {
    uint32_t now = time_us_32();
    uint32_t us_since_reading = now - cb->last_angle_upd_time + READING_DELAY_US;
    fix15_t s_since_reading = USEC_TO_FIX15_S(us_since_reading);
    fix15_t pole_angle_est = cb->last_pole_angle + (cb->est_pole_v * s_since_reading);
    //print_fix15("s", s_since_reading);
    //print_fix15("pv", cb->est_pole_v);
    //print_fix15("npa", pole_angle_est);
    fix15_t drive_angle = pole_angle_est+cb->drive_angle_offset;
    drive_angle = clamp_angle(drive_angle);
    motor_set_pwms(cb, drive_angle);
    //motor_update_output_old(cb);
    //printf("\n");
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