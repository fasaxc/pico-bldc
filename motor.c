#include "motor.h"
#include "hardware/dma.h"
#include "hardware/timer.h"

#define MOTOR_NUM_POLES 22

// PWM_TOP = 6200 gives roughly 20kHz.
#define PWM_TOP 6200
#define LUT_LEN 4096
static u_int16_t pwm_lut[LUT_LEN];

static PIO pio;
static uint pio_offset_high;
static uint pio_offset_invl;

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
    pio_offset_invl = pio_add_program(pio, &pwminvl_program);
    pio_offset_high = pio_add_program(pio, &pwmhigh_program);
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

    // Enable all PWMs together so that they start in phase.
    pwm_set_mask_enabled((1<<cb->pwm_slice_a) | 
                         (1<<cb->pwm_slice_b) |  
                         (1<<cb->pwm_slice_c));

    // Set up the PIO state machines.  We can use the same programs
    // for every motor instance.
    cb->sm_invl = pio_claim_unused_sm(pio, true);
    printf("Init interval PIO on SM %d pin %d\n", cb->sm_invl, pin_pwm_in);
    pwminvl_program_init(pio, cb->sm_invl, pio_offset_invl, pin_pwm_in);
    cb->sm_high = pio_claim_unused_sm(pio, true);
    printf("Init high time PIO on SM %d pin %d\n", cb->sm_high, pin_pwm_in);
    pwmhigh_program_init(pio, cb->sm_high, pio_offset_high, pin_pwm_in);

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
            dreq = DREQ_PIO0_RX0 + cb->sm_high;
        } else {
            dreq = DREQ_PIO1_RX0 + cb->sm_high;
        }
        channel_config_set_dreq(&c, dreq); 
        channel_config_set_chain_to(&c, time_chan);    

        dma_channel_configure(
            pio_chan,               // Channel to be configured
            &c,                     // The configuration we just created
            NULL,                   // Write address
            &pio->rxf[cb->sm_high], // Read address
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
        printf("Wait on interval...\n");
        uint32_t v = pio_sm_get_blocking(pio, cb->sm_invl);
        printf("Raw value: %d\n", v);
        printf("Wait on high time...\n");
        v = pio_sm_get_blocking(pio, cb->sm_high);
        printf("Raw value: %d\n", v);
    }
    while (!pio_sm_get_rx_fifo_level(pio, cb->sm_invl)) {
        // Wait for the next interval measurement to arrive so
        // we'll definitely load it on the first loop below.
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
        high = 0xffffffff - drain_pio_fifo_blocking(pio, cb->sm_high);

        // Load any updated PWM interval from the PIO.  The interval
        // is updated every other PWM cycle so we don't wait for it.
        invl = 0xffffffff - drain_pio_fifo_non_block(pio, cb->sm_invl);
        cb->last_sensor_pwm_interval = invl;
        one_clock = (invl << fp_bits) / (16+4095+8);
        half_clock = one_clock / 2;

        // Convert to fixed point for better precision.
        high = high << fp_bits;
        // Convert to angle (in 4096ths of a circle).
        int32_t wheel_angle = (((high + half_clock) / one_clock) - 16) & 0xfff;
        
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

        pwm_set_chan_level(cb->pwm_slice_a, cb->pwm_chan_a, duty_a);
        pwm_set_chan_level(cb->pwm_slice_b, cb->pwm_chan_b, duty_b);
        pwm_set_chan_level(cb->pwm_slice_c, cb->pwm_chan_c, duty_c);
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

void motor_record_pwm_interval(struct motor_cb *cb, uint32_t raw_pio_output) {
    uint32_t invl = 0xffffffff - raw_pio_output;
    cb->last_sensor_pwm_interval = invl;
}

void motor_record_pwm_high_time(struct motor_cb *cb, uint32_t raw_pio_output, uint32_t timestamp) {
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
    int idx = cb->angle_buf_idx;
    uint32_t last_reading_time = cb->measured_angles_ring_buf[idx].time_us;
    uint32_t next_raw_high = cb->next_raw_high;
    uint32_t next_timestamp = cb->next_timestamp;
    if (next_timestamp == last_reading_time) {
        return;
    }

    uint32_t raw_pio_invl = drain_pio_fifo_non_block(pio0, cb->sm_invl);
    if (raw_pio_invl) { 
        motor_record_pwm_interval(cb, raw_pio_invl);
    }
    motor_record_pwm_high_time(cb, next_raw_high, next_timestamp);
    motor_process_angle_meas(cb);
}