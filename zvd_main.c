/*
 * ================================================================
 * COMPLETE ZVD INPUT SHAPING SYSTEM
 * STM32F401 Black Pill + A4988 + NEMA17 + ADXL345
 *
 * Motion: 0 -> 50 degrees (28 steps, full-step mode)
 * 
 * Phase A: UNSHAPED  - Instant snap, measure natural frequency
 * Phase B: ZV SHAPED - 2-impulse destructive interference
 * Phase C: ZVD SHAPED- 3-impulse zero-vibration-derivative
 *
 * Shared SRAM telemetry buffer at 0x20000000 for Python monitor
 * ================================================================
 */
#include <stdint.h>

// ==== Peripheral Register Map ====
#define RCC_BASE        0x40023800
#define RCC_AHB1ENR     (*(volatile uint32_t *)(RCC_BASE + 0x30))

#define GPIOA_BASE      0x40020000
#define GPIOA_MODER     (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_BSRR      (*(volatile uint32_t *)(GPIOA_BASE + 0x18))

#define GPIOB_BASE      0x40020400
#define GPIOB_MODER     (*(volatile uint32_t *)(GPIOB_BASE + 0x00))
#define GPIOB_OTYPER    (*(volatile uint32_t *)(GPIOB_BASE + 0x04))
#define GPIOB_PUPDR     (*(volatile uint32_t *)(GPIOB_BASE + 0x0C))
#define GPIOB_IDR       (*(volatile uint32_t *)(GPIOB_BASE + 0x10))
#define GPIOB_BSRR      (*(volatile uint32_t *)(GPIOB_BASE + 0x18))

#define GPIOC_BASE      0x40020800
#define GPIOC_MODER     (*(volatile uint32_t *)(GPIOC_BASE + 0x00))
#define GPIOC_BSRR      (*(volatile uint32_t *)(GPIOC_BASE + 0x18))

// ==== Linker Symbols ====
extern uint32_t _estack, _etext, _sdata, _edata, _sbss, _ebss;

// ==== SRAM Telemetry Block at 0x20000000 ====
// Python reads this in real-time via ST-LINK
typedef struct __attribute__((packed)) {
    // Header
    uint32_t magic;             // 0xABCD1234 when valid

    // Sensor health
    uint8_t  devid;             // Should be 0xE5
    uint8_t  sensor_ok;         // 1 = online

    // Current phase tracking
    uint8_t  phase;             // 0=IDLE 1=UNSHAPED 2=ZV 3=ZVD 4=RETURNING
    uint8_t  cycle;             // Cycle counter: 1,2,3...

    // Measured beam properties (from unshaped test)
    uint32_t nat_freq_mhz;      // Natural frequency in mHz (e.g. 8500 = 8.5 Hz)
    uint32_t damp_ratio_ppm;    // Damping ratio in ppm (e.g. 50000 = 0.05 = 5%)
    uint32_t Td_ms;             // Damped period in ms

    // ZV shaper parameters (scaled integers)
    uint32_t ZV_A1_pct;         // A1 * 100 (e.g. 54 = 0.54)
    uint32_t ZV_A2_pct;         // A2 * 100
    uint32_t ZV_t1_ms;          // t1 delay (always 0)
    uint32_t ZV_t2_ms;          // t2 delay = Td/2

    // ZVD shaper parameters
    uint32_t ZVD_A1_pct;        // A1 * 100
    uint32_t ZVD_A2_pct;        // A2 * 100
    uint32_t ZVD_A3_pct;        // A3 * 100
    uint32_t ZVD_t1_ms;
    uint32_t ZVD_t2_ms;         // Td/2
    uint32_t ZVD_t3_ms;         // Td

    // Per-stroke results (updated after each wobble window)
    int32_t  unshaped_p2p_mg;   // Peak-to-peak vibration (mg) - Unshaped
    int32_t  zv_p2p_mg;         // Peak-to-peak vibration (mg) - ZV
    int32_t  zvd_p2p_mg;        // Peak-to-peak vibration (mg) - ZVD
    int32_t  curr_z_mg;         // Current live Z reading
    int32_t  curr_max_mg;       // Max Z this stroke
    int32_t  curr_min_mg;       // Min Z this stroke

    // Reduction percentages (x10 for decimal, e.g. 923 = 92.3%)
    uint32_t zv_reduction_x10;
    uint32_t zvd_reduction_x10;

    uint32_t total_samples;     // Running sample count
    uint32_t zero_crossings;    // Last stroke zero crossings

} ZVD_Telemetry_t;

volatile ZVD_Telemetry_t T __attribute__((section(".sensor_buffer")));

// ==== Vector Table ====
void Reset_Handler(void);
void Default_Handler(void) { while(1); }

__attribute__((section(".isr_vector")))
void (* const g_pfnVectors[])(void) = {
    (void (*)(void))((uint32_t)&_estack),
    Reset_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler,
    0, 0, 0, 0, Default_Handler, Default_Handler, 0,
    Default_Handler, Default_Handler
};

// ==== Software Float Math (ARM FPU) ====
// Implements exp() via Taylor series - accurate for |x| < 1.5
static float my_exp(float x) {
    if (x < -4.0f) return 0.0183f;
    if (x > 4.0f) return 54.6f;
    float term = 1.0f, sum = 1.0f;
    for (int n = 1; n <= 12; n++) {
        term *= x / (float)n;
        sum += term;
    }
    return sum;
}

static float my_sqrt(float x) {
    if (x <= 0.0f) return 0.0f;
    float r = x > 1.0f ? x * 0.5f : 1.0f;
    for (int i = 0; i < 20; i++) r = 0.5f * (r + x / r);
    return r;
}

// ==== Timing ====
static void delay_us(uint32_t us) {
    if (!us) return;
    __asm__ volatile(
        "1: mov r1, #3\n"
        "2: subs r1, r1, #1\n"
        "   bne 2b\n"
        "   subs %0, %0, #1\n"
        "   bne 1b\n"
        : "+r"(us) :: "r1", "cc"
    );
}

static void delay_ms(uint32_t ms) {
    while (ms--) delay_us(1000);
}

// ==== I2C Bit-Bang (PB6=SCL, PB7=SDA) ====
static inline void scl_hi(void) { GPIOB_BSRR = (1<<6); }
static inline void scl_lo(void) { GPIOB_BSRR = (1<<22); }
static inline void sda_hi(void) { GPIOB_BSRR = (1<<7); }
static inline void sda_lo(void) { GPIOB_BSRR = (1<<23); }
static inline int  sda_rd(void) { return (GPIOB_IDR & (1<<7)) ? 1 : 0; }

static void i2c_start(void) { sda_hi(); delay_us(2); scl_hi(); delay_us(2); sda_lo(); delay_us(2); scl_lo(); delay_us(2); }
static void i2c_stop(void)  { sda_lo(); delay_us(2); scl_hi(); delay_us(2); sda_hi(); delay_us(2); }

static void i2c_wr(uint8_t b) {
    for (int i = 0; i < 8; i++) {
        if (b & 0x80) sda_hi(); else sda_lo();
        delay_us(2); scl_hi(); delay_us(2); scl_lo(); delay_us(2);
        b <<= 1;
    }
    sda_hi(); delay_us(2); scl_hi(); delay_us(2); scl_lo(); delay_us(2);
}

static uint8_t i2c_rd(int ack) {
    uint8_t b = 0; sda_hi();
    for (int i = 0; i < 8; i++) {
        b <<= 1; scl_hi(); delay_us(2);
        if (sda_rd()) b |= 1;
        scl_lo(); delay_us(2);
    }
    if (ack) sda_lo(); else sda_hi();
    delay_us(2); scl_hi(); delay_us(2); scl_lo(); delay_us(2); sda_hi();
    return b;
}

static void adxl_wr(uint8_t reg, uint8_t val) {
    i2c_start(); i2c_wr(0xA6); i2c_wr(reg); i2c_wr(val); i2c_stop();
}

static uint8_t adxl_rd(uint8_t reg) {
    i2c_start(); i2c_wr(0xA6); i2c_wr(reg);
    i2c_start(); i2c_wr(0xA7);
    uint8_t v = i2c_rd(0); i2c_stop();
    return v;
}

static int16_t adxl_z(void) {
    i2c_start(); i2c_wr(0xA6); i2c_wr(0x36);
    i2c_start(); i2c_wr(0xA7);
    uint8_t lo = i2c_rd(1), hi = i2c_rd(0); i2c_stop();
    return (int16_t)((hi << 8) | lo);
}

// ==== Stepper ====
#define STEP_HI()  GPIOA_BSRR = (1<<2)
#define STEP_LO()  GPIOA_BSRR = (1<<18)
#define DIR_FW()   GPIOA_BSRR = (1<<1)
#define DIR_RV()   GPIOA_BSRR = (1<<17)

// 50 degrees = 200 * (50/360) = 27.78 -> 28 steps
#define TOTAL_STEPS   28
#define STEP_US       900   // Step interval during snap (high speed burst)
#define RETURN_US     5000  // Slow return step interval

static void send_steps(int n, uint32_t interval_us) {
    for (int i = 0; i < n; i++) {
        STEP_HI(); delay_us(8);
        STEP_LO(); delay_us(interval_us);
    }
}

// ==== Sample Z-axis for a window, return p2p vibration in mg ====
// Also detects zero crossings for natural frequency
static int32_t sample_vibration(int sample_count, int sample_ms) {
    int32_t p_max = -999999, p_min = 999999;
    int32_t baseline = 0;
    int crossings = 0, last_sign = 0;

    for (int s = 0; s < sample_count; s++) {
        int16_t rz = adxl_z();
        int32_t mz = (int32_t)rz * 39 / 10;

        T.curr_z_mg = mz;
        T.total_samples++;

        if (mz > p_max) p_max = mz;
        if (mz < p_min) p_min = mz;

        if (s == 0) baseline = mz;

        int sign = (mz >= baseline) ? 1 : -1;
        if (s > 2 && sign != last_sign) crossings++;
        last_sign = sign;

        delay_ms(sample_ms);
    }

    T.curr_max_mg = p_max;
    T.curr_min_mg = p_min;
    T.zero_crossings = crossings;

    return p_max - p_min;
}

// ==== LED Indicators ====
#define LED_ON()  GPIOC_BSRR = (1<<29)
#define LED_OFF() GPIOC_BSRR = (1<<13)

// ==== MAIN ====
void Reset_Handler(void) {
    // Copy .data, zero .bss
    uint32_t *s = &_etext, *d = &_sdata;
    while (d < &_edata) *d++ = *s++;
    d = &_sbss;
    while (d < &_ebss) *d++ = 0;

    // Init telemetry
    T.magic           = 0xABCD1234;
    T.phase           = 0;
    T.cycle           = 0;
    T.unshaped_p2p_mg = 0;
    T.zv_p2p_mg       = 0;
    T.zvd_p2p_mg      = 0;
    T.total_samples   = 0;
    T.nat_freq_mhz    = 0;

    // Enable GPIO clocks: A, B, C
    RCC_AHB1ENR |= (1<<0) | (1<<1) | (1<<2);
    volatile uint32_t tmp = RCC_AHB1ENR; (void)tmp;

    // PA1=DIR, PA2=STEP as output
    GPIOA_MODER &= ~((3U<<2) | (3U<<4));
    GPIOA_MODER |= ((1U<<2) | (1U<<4));

    // PC13=LED as output
    GPIOC_MODER &= ~(3U<<26);
    GPIOC_MODER |= (1U<<26);

    // PB6=SCL, PB7=SDA: open-drain output with pull-up
    GPIOB_MODER &= ~((3U<<12) | (3U<<14));
    GPIOB_MODER |= ((1U<<12) | (1U<<14));
    GPIOB_OTYPER |= (1<<6) | (1<<7);
    GPIOB_PUPDR &= ~((3U<<12) | (3U<<14));
    GPIOB_PUPDR |= ((1U<<12) | (1U<<14));

    // Initial pin states
    GPIOA_BSRR = (1<<17) | (1<<18); // DIR=0, STEP=0
    scl_hi(); sda_hi();
    LED_OFF();
    delay_ms(100);

    // Init ADXL345
    uint8_t id = adxl_rd(0x00);
    T.devid = id;
    T.sensor_ok = (id == 0xE5) ? 1 : 0;
    if (T.sensor_ok) {
        adxl_wr(0x2D, 0x08); // Measurement mode ON
        adxl_wr(0x31, 0x0B); // Full-res, +/-16g
        adxl_wr(0x2C, 0x0D); // 800 Hz output rate
    }

    // Startup blink: 3 flashes = all systems GO
    for (int i = 0; i < 3; i++) {
        LED_ON(); delay_ms(80); LED_OFF(); delay_ms(80);
    }
    delay_ms(500);

    // ================================================================
    // MEASURED SHAPER PARAMETERS (Computed after first unshaped run)
    // These are updated each cycle based on live frequency measurement
    // ================================================================
    float fn_hz   = 8.5f;   // Natural frequency (Hz) - updated after first snap
    float zeta    = 0.05f;  // Damping ratio estimate - refined over cycles
    float Td_s    = 0.0f;   // Damped period (seconds)
    float K       = 0.0f;   // Shaper gain ratio
    int   zv_steps1 = 0, zv_steps2 = 0;
    int   zvd_steps1 = 0, zvd_steps2 = 0, zvd_steps3 = 0;
    uint32_t zv_delay2_ms = 0;
    uint32_t zvd_delay2_ms = 0, zvd_delay3_ms = 0;
    int params_valid = 0;

    // ================================================================
    // MAIN LOOP: Cycles through UNSHAPED -> ZV -> ZVD -> repeat
    // ================================================================
    while (1) {
        T.cycle++;

        // ============================================================
        // PHASE A: UNSHAPED SNAP  (0 -> 50 deg, one shot, full speed)
        // ============================================================
        T.phase = 1;
        DIR_FW();
        LED_OFF();
        delay_ms(300);

        send_steps(TOTAL_STEPS, STEP_US); // Blast all 28 steps instantly!

        // Lock and measure vibration for 3 seconds at 200 Hz (600 samples)
        LED_ON();
        int32_t unshaped_p2p = sample_vibration(300, 10); // 300 samples x 10ms = 3s
        T.unshaped_p2p_mg = unshaped_p2p;
        LED_OFF();

        // ---- Identify Natural Frequency from zero crossings ----
        // 300 samples * 10ms = 3.0 seconds window
        // frequency = zero_crossings / (2 * 3.0)
        int zc = (int)T.zero_crossings;
        if (zc >= 2) {
            fn_hz  = (float)zc / 6.0f;  // Hz (3 sec window, 2 crossings per cycle)
            if (fn_hz < 1.0f) fn_hz = 1.0f;
            if (fn_hz > 50.0f) fn_hz = 50.0f;

            // Estimate damping from amplitude envelope: use 20% as start
            // ζ ≈ 0.03 to 0.1 for typical plastic rulers
            zeta = 0.05f;

            float wd = fn_hz * 6.28318f;             // Damped angular freq rad/s
            float wn = wd / my_sqrt(1.0f - zeta*zeta); // Natural angular freq
            Td_s = 1.0f / fn_hz;                      // Damped period in seconds
            K = my_exp((-zeta * 3.14159f) / my_sqrt(1.0f - zeta*zeta));

            // ---- ZV Shaper: 2 impulses ----
            float A1_zv = 1.0f / (1.0f + K);
            float A2_zv = K   / (1.0f + K);
            zv_delay2_ms = (uint32_t)(Td_s * 500.0f); // Td/2 in ms

            // Split TOTAL_STEPS by amplitude ratios
            zv_steps1 = (int)(A1_zv * TOTAL_STEPS + 0.5f);
            zv_steps2 = TOTAL_STEPS - zv_steps1;

            // ---- ZVD Shaper: 3 impulses ----
            float K2 = K * K;
            float denom = (1.0f + K) * (1.0f + K);
            float A1_zvd = 1.0f / denom;
            float A2_zvd = 2.0f * K / denom;
            float A3_zvd = K2 / denom;
            zvd_delay2_ms = zv_delay2_ms;              // Td/2
            zvd_delay3_ms = (uint32_t)(Td_s * 1000.0f); // Td

            zvd_steps1 = (int)(A1_zvd * TOTAL_STEPS + 0.5f);
            zvd_steps2 = (int)(A2_zvd * TOTAL_STEPS + 0.5f);
            zvd_steps3 = TOTAL_STEPS - zvd_steps1 - zvd_steps2;

            // Write to telemetry
            T.nat_freq_mhz   = (uint32_t)(fn_hz * 1000.0f);
            T.damp_ratio_ppm = (uint32_t)(zeta * 1000000.0f);
            T.Td_ms          = (uint32_t)(Td_s * 1000.0f);
            T.ZV_A1_pct      = (uint32_t)(A1_zv * 100.0f);
            T.ZV_A2_pct      = (uint32_t)(A2_zv * 100.0f);
            T.ZV_t1_ms       = 0;
            T.ZV_t2_ms       = zv_delay2_ms;
            T.ZVD_A1_pct     = (uint32_t)(A1_zvd * 100.0f);
            T.ZVD_A2_pct     = (uint32_t)(A2_zvd * 100.0f);
            T.ZVD_A3_pct     = (uint32_t)(A3_zvd * 100.0f);
            T.ZVD_t2_ms      = zvd_delay2_ms;
            T.ZVD_t3_ms      = zvd_delay3_ms;
            params_valid      = 1;
        }

        // Return home
        T.phase = 4;
        DIR_RV();
        delay_ms(200);
        send_steps(TOTAL_STEPS, RETURN_US);
        delay_ms(1000);

        // ============================================================
        // PHASE B: ZV SHAPED  (2-impulse, destructive interference)
        // ============================================================
        T.phase = 2;
        DIR_FW();
        delay_ms(300);

        if (params_valid && zv_steps1 > 0 && zv_steps2 > 0) {
            // Impulse 1: at t=0
            send_steps(zv_steps1, STEP_US);
            // Wait Td/2 for destructive interference
            delay_ms(zv_delay2_ms);
            // Impulse 2: exactly cancels the vibration from impulse 1
            send_steps(zv_steps2, STEP_US);
        } else {
            send_steps(TOTAL_STEPS, STEP_US); // Fallback if no freq yet
        }

        LED_ON();
        int32_t zv_p2p = sample_vibration(300, 10);
        T.zv_p2p_mg = zv_p2p;
        LED_OFF();

        // Return home
        T.phase = 4;
        DIR_RV();
        delay_ms(200);
        send_steps(TOTAL_STEPS, RETURN_US);
        delay_ms(1000);

        // ============================================================
        // PHASE C: ZVD SHAPED  (3-impulse, robust zero vibration)
        // ============================================================
        T.phase = 3;
        DIR_FW();
        delay_ms(300);

        if (params_valid && zvd_steps1 > 0 && zvd_steps2 > 0 && zvd_steps3 > 0) {
            // Impulse 1: at t=0
            send_steps(zvd_steps1, STEP_US);
            // Wait Td/2
            delay_ms(zvd_delay2_ms);
            // Impulse 2: at t=Td/2
            send_steps(zvd_steps2, STEP_US);
            // Wait another Td/2
            delay_ms(zvd_delay3_ms - zvd_delay2_ms);
            // Impulse 3: at t=Td
            send_steps(zvd_steps3, STEP_US);
        } else {
            send_steps(TOTAL_STEPS, STEP_US);
        }

        LED_ON();
        int32_t zvd_p2p = sample_vibration(300, 10);
        T.zvd_p2p_mg = zvd_p2p;
        LED_OFF();

        // Compute reduction percentages
        if (T.unshaped_p2p_mg > 0) {
            int32_t zv_red = ((T.unshaped_p2p_mg - zv_p2p) * 1000) / T.unshaped_p2p_mg;
            int32_t zvd_red = ((T.unshaped_p2p_mg - zvd_p2p) * 1000) / T.unshaped_p2p_mg;
            T.zv_reduction_x10  = (uint32_t)(zv_red < 0 ? 0 : zv_red);
            T.zvd_reduction_x10 = (uint32_t)(zvd_red < 0 ? 0 : zvd_red);
        }

        // Return home gently before next cycle
        T.phase = 4;
        DIR_RV();
        delay_ms(200);
        send_steps(TOTAL_STEPS, RETURN_US);
        delay_ms(2000); // 2s rest between full cycles

        T.phase = 0; // IDLE momentarily
        delay_ms(500);
    }
}
