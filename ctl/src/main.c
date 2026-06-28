/* =========================================================
 * ISR TIMING ESTIMATE (RP2350 - Pico 2 / Micro2 class board)
 * =========================================================
 *
 * Assumptions:
 * - Cortex-M33 (dual-core capable, single core used here)
 * - ISR executed from SRAM (__ramfunc)
 * - IRQ_DIRECT_CONNECT (zero Zephyr ISR wrapper overhead)
 * - CMSIS-DSP Q15 optimized build
 * - NO logging / NO printf in ISR
 * - Core clock: 150 MHz (DEFAULT), up to 200–300 MHz possible
 *
 * ---------------------------------------------------------
 * Operation breakdown (cycles)
 * ---------------------------------------------------------
 *
 * ISR entry / exit:
 *   BEST:   20 cycles
 *   STD:    30 cycles
 *   WORST:  45 cycles
 *
 * pwm_get_counter():
 *   BEST:    8 cycles
 *   STD:    12 cycles
 *   WORST:  18 cycles
 *
 * delay element (y_prev shift/store):
 *   BEST:    5 cycles
 *   STD:     8 cycles
 *   WORST:  12 cycles
 *
 * biquad filter (CMSIS Q15, 1 stage):
 *   BEST:   20 cycles
 *   STD:    45 cycles
 *   WORST:  70 cycles
 *
 * PID Q15 (arm_pid_q15):
 *   BEST:   45 cycles
 *   STD:    90 cycles
 *   WORST: 140 cycles
 *
 * clamp + error + scaling math:
 *   BEST:   10 cycles
 *   STD:    20 cycles
 *   WORST:  30 cycles
 *
 * pwm_set_gpio_level():
 *   BEST:   15 cycles
 *   STD:    30 cycles
 *   WORST:  50 cycles
 *
 * pwm_clear_irq():
 *   BEST:    5 cycles
 *   STD:    10 cycles
 *   WORST:  15 cycles
 *
 * ---------------------------------------------------------
 * TOTAL ISR COST
 * ---------------------------------------------------------
 *
 * BEST CASE:
 *   ≈ 128 cycles  (~0.85 µs @ 150 MHz)
 *
 * STD CASE:
 *   ≈ 245 cycles  (~1.63 µs @ 150 MHz)
 *
 * WORST CASE:
 *   ≈ 370 cycles  (~2.47 µs @ 150 MHz)
 *
 * ---------------------------------------------------------
 * CONTEXT: PWM PERIOD = 3.33 µs (300 kHz)
 *
 * CPU LOAD @ 150 MHz:
 *   BEST:  ~25%
 *   STD:   ~49%
 *   WORST: ~74%
 *
 * ========================================================= */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/irq.h>

#include <arm_math.h>
#include "hardware/pwm.h"

/* =========================================================
 * GPIO (RP2350 pins)
 * ========================================================= */
#define FB_GPIO       2
#define PWM_OUT_GPIO  15

/* =========================================================
 * CONTROL CONSTANTS
 * ========================================================= */
#define PWM_FREQ_HZ     300000u
#define PWM_PERIOD_NS   (1000000000u / PWM_FREQ_HZ)

/* duty safety limits */
#define DUTY_MIN_FRAC   0.10f   /* 10% minimum duty */
#define DUTY_MAX_FRAC   0.90f   /* 90% maximum duty */

/* =========================================================
 * CONTROL STRUCT
 * ========================================================= */
typedef struct {

    uint slice;

    /* ---------------- PID (Q15 fixed-point controller) ----------------
     * Q15 format: 1.0 == 32768
     * So:
     *   0.1  => 3276
     *   1.0  => 32768
     */
    arm_pid_instance_q15 pid;

    /* setpoint in Q15 (normalized signal range)
     * 16384 ≈ 0.5 in Q15 scale
     */
    q15_t setpoint;

    /* ---------------- LP FILTER (CMSIS biquad Q15) ----------------
     * Direct Form I filter state + coefficients
     */
    arm_biquad_casd_df1_inst_q15 lp;

    q15_t lp_state[4];

    /* coefficients:
     * [b0, b1, b2, a1, a2] in Q15
     */
    q15_t lp_coeffs[5];

    /* previous sample for simple delay element */
    uint16_t y_prev;

} flyback_cntrl_t;

/* global instance used by ISR */
static flyback_cntrl_t cntrl;

/* =========================================================
 * FEEDBACK READ
 * ========================================================= */
static inline q15_t read_feedback(void)
{
    uint16_t c = pwm_get_counter(cntrl.slice);

    /* scale:
     * raw counter ~ 0..65535 -> Q15 range approximation
     */
    return (q15_t)(c << 3);
}

/* =========================================================
 * FILTER INIT
 * ========================================================= */
static void filter_init_from_pwm_freq(void)
{
    uint32_t fc = PWM_FREQ_HZ / 50u; /* cutoff heuristic: PWM/50 */

    float fs = (float)PWM_FREQ_HZ;
    float dt = 1.0f / fs;

    float rc = 1.0f / (2.0f * 3.1415926f * (float)fc);
    float alpha = dt / (rc + dt);

    if (alpha > 0.99f) {
        alpha = 0.99f;
    }

    /* ---------------- Q15 COEFFICIENT CONVERSION ----------------
     * Q15 scaling: multiply float by 32768
     *
     * b0 = alpha
     * b1 = 0
     * b2 = 0
     * a1 = alpha - 1
     * a2 = 0
     */

    cntrl.lp_coeffs[0] = (q15_t)(alpha * 32768.0f);                 /* b0 ≈ alpha */
    cntrl.lp_coeffs[1] = 0;                                         /* b1 = 0 */
    cntrl.lp_coeffs[2] = 0;                                         /* b2 = 0 */
    cntrl.lp_coeffs[3] = (q15_t)((alpha - 1.0f) * 32768.0f);        /* a1 ≈ - (1 - alpha) */
    cntrl.lp_coeffs[4] = 0;                                         /* a2 = 0 */

    arm_biquad_cascade_df1_init_q15(
        &cntrl.lp,
        1,
        cntrl.lp_coeffs,
        cntrl.lp_state,
        1
    );
}

/* =========================================================
 * FILTER APPLY
 * ========================================================= */
static inline q15_t filter(q15_t x)
{
    q15_t in = x;
    q15_t out;

    arm_biquad_cascade_df1_q15(
        &cntrl.lp,
        &in,
        &out,
        1
    );

    return out;
}

/* =========================================================
 * PWM DUTY UPDATE (10–90% clamp)
 * ========================================================= */
static inline void update_pwm(uint32_t duty_ns)
{
    const uint32_t wrap = 1u << 16;

    uint32_t level = (duty_ns * wrap) / PWM_PERIOD_NS;

    /* enforce duty limits */
    const uint32_t min_level = (uint32_t)(DUTY_MIN_FRAC * wrap);
    const uint32_t max_level = (uint32_t)(DUTY_MAX_FRAC * wrap);

    if (level < min_level) level = min_level;
    if (level > max_level) level = max_level;

    pwm_set_gpio_level(PWM_OUT_GPIO, level);
}

/* =========================================================
 * ISR (LOW LATENCY PWM WRAP)
 * ========================================================= */
__ramfunc static void pwm_wrap_isr(const void *arg)
{
    ARG_UNUSED(arg);

    q15_t y = read_feedback();

    /* simple 1-sample delay element */
    q15_t y_delayed = (q15_t)(cntrl.y_prev << 3);
    cntrl.y_prev = y;

    q15_t y_f = filter(y_delayed);

    q15_t err = cntrl.setpoint - y_f;

    q15_t u = arm_pid_q15(&cntrl.pid, err);

    /* PID saturation (Q15 domain) */
    if (u > 25000) u = 25000;
    if (u < -25000) u = -25000;

    /* map Q15 control -> duty cycle */
    int32_t duty =
        ((int32_t)(u + 32768) * PWM_PERIOD_NS) >> 16;

    if (duty < 0) duty = 0;
    if (duty > (int32_t)PWM_PERIOD_NS) duty = PWM_PERIOD_NS;

    update_pwm((uint32_t)duty);

    pwm_clear_irq(cntrl.slice);
}

/* =========================================================
 * PWM INPUT INIT
 * ========================================================= */
static void pwm_input_init(void)
{
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_RISING);

    cntrl.slice = pwm_gpio_to_slice_num(FB_GPIO);

    pwm_init(cntrl.slice, &cfg, true);
    pwm_set_counter(cntrl.slice, 0);

    pwm_clear_irq(cntrl.slice);
    pwm_set_irq_enabled(cntrl.slice, true);

    IRQ_DIRECT_CONNECT(
        PWM_IRQ_WRAP,
        0,
        pwm_wrap_isr,
        0
    );

    irq_enable(PWM_IRQ_WRAP);
}

/* =========================================================
 * MAIN
 * ========================================================= */
int main(void)
{
    printk("Flyback controller (10–90%% featuring clamp, filtering(all in Q15)\n");

    /* PID tuning (Q15 scale: 1.0 = 32768) */
    cntrl.pid.Kp = 1966;   /* ~0.06 proportional gain */
    cntrl.pid.Ki = 196;    /* small integral action */
    cntrl.pid.Kd = 33;     /* light derivative damping */

    cntrl.setpoint = 16384; /* ~0.5 normalized target */
    arm_pid_init_q15(&cntrl.pid, 1);

    filter_init_from_pwm_freq();
    pwm_input_init();

    /* safe startup duty (~50%) */
    pwm_set_gpio_level(PWM_OUT_GPIO, 1u << 15);

    while (1) {
        k_sleep(K_SECONDS(1));
    }
}
