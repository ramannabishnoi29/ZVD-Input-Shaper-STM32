#include <stdint.h>

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

extern uint32_t _estack, _etext, _sdata, _edata, _sbss, _ebss;

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

// Telemetry Buffer at 0x20000000
typedef struct __attribute__((packed)) {
    uint32_t magic;                 // 0xAD345002
    uint32_t stroke_count;          // Increments every stroke: 1, 2, 3...
    uint8_t  devid;                 // 0xE5
    uint8_t  status;                // 1 = Active
    int16_t  curr_raw_z;
    int32_t  curr_mg_z;             // Current Z (mg)
    int32_t  stroke_peak_max_z;     // Max positive Z during stroke (mg)
    int32_t  stroke_peak_min_z;     // Max negative Z during stroke (mg)
    int32_t  stroke_p2p_mg;         // Peak-to-Peak Wobble (mg)
    int32_t  est_freq_hz_x10;       // Estimated Natural Frequency (Hz * 10, e.g. 85 = 8.5 Hz)
    uint32_t total_samples;
} StrokeTelemetry_t;

volatile StrokeTelemetry_t g_telemetry __attribute__((section(".sensor_buffer")));

static void delay_us(uint32_t us) {
    if (us == 0) return;
    __asm__ volatile (
        "1: \n"
        "  mov r1, #3 \n"
        "2: \n"
        "  subs r1, r1, #1 \n"
        "  bne 2b \n"
        "  subs %0, %0, #1 \n"
        "  bne 1b \n"
        : "+r"(us)
        :
        : "r1", "cc"
    );
}

static void delay_ms(uint32_t ms) {
    while (ms--) delay_us(1000);
}

// I2C Driver on PB6/PB7
static inline void scl_high(void) { GPIOB_BSRR = (1 << 6); }
static inline void scl_low(void)  { GPIOB_BSRR = (1 << 22); }
static inline void sda_high(void) { GPIOB_BSRR = (1 << 7); }
static inline void sda_low(void)  { GPIOB_BSRR = (1 << 23); }
static inline int  sda_read(void) { return (GPIOB_IDR & (1 << 7)) ? 1 : 0; }

static void i2c_init(void) {
    RCC_AHB1ENR |= (1 << 1);
    volatile uint32_t d = RCC_AHB1ENR;
    (void)d;
    GPIOB_MODER &= ~((3U << 12) | (3U << 14));
    GPIOB_MODER |= ((1U << 12) | (1U << 14));
    GPIOB_OTYPER |= (1 << 6) | (1 << 7);
    GPIOB_PUPDR &= ~((3U << 12) | (3U << 14));
    GPIOB_PUPDR |= ((1U << 12) | (1U << 14));
    scl_high(); sda_high();
    delay_us(10);
}

static void i2c_start(void) {
    sda_high(); delay_us(2);
    scl_high(); delay_us(2);
    sda_low();  delay_us(2);
    scl_low();  delay_us(2);
}

static void i2c_stop(void) {
    sda_low();  delay_us(2);
    scl_high(); delay_us(2);
    sda_high(); delay_us(2);
}

static int i2c_write(uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        if (byte & 0x80) sda_high(); else sda_low();
        delay_us(2); scl_high(); delay_us(2); scl_low(); delay_us(2);
        byte <<= 1;
    }
    sda_high(); delay_us(2); scl_high(); delay_us(2);
    int ack = !sda_read();
    scl_low(); delay_us(2);
    return ack;
}

static uint8_t i2c_read(int ack) {
    uint8_t byte = 0;
    sda_high();
    for (int i = 0; i < 8; i++) {
        byte <<= 1;
        scl_high(); delay_us(2);
        if (sda_read()) byte |= 1;
        scl_low(); delay_us(2);
    }
    if (ack) sda_low(); else sda_high();
    delay_us(2); scl_high(); delay_us(2); scl_low(); delay_us(2);
    sda_high();
    return byte;
}

static uint8_t adxl_read_reg(uint8_t reg) {
    i2c_start(); i2c_write(0xA6); i2c_write(reg);
    i2c_start(); i2c_write(0xA7);
    uint8_t val = i2c_read(0);
    i2c_stop();
    return val;
}

static void adxl_write_reg(uint8_t reg, uint8_t val) {
    i2c_start(); i2c_write(0xA6); i2c_write(reg); i2c_write(val);
    i2c_stop();
}

static int16_t adxl_read_z(void) {
    i2c_start(); i2c_write(0xA6); i2c_write(0x36); // DATAZ0
    i2c_start(); i2c_write(0xA7);
    uint8_t zl = i2c_read(1);
    uint8_t zh = i2c_read(0);
    i2c_stop();
    return (int16_t)((zh << 8) | zl);
}

void Reset_Handler(void) {
    uint32_t *src = &_etext, *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;
    dst = &_sbss;
    while (dst < &_ebss) *dst++ = 0;

    g_telemetry.magic = 0xAD345002;
    g_telemetry.stroke_count = 0;
    g_telemetry.devid = 0;
    g_telemetry.status = 0;
    g_telemetry.total_samples = 0;

    RCC_AHB1ENR |= (1 << 0) | (1 << 1) | (1 << 2);
    volatile uint32_t dummy = RCC_AHB1ENR;
    (void)dummy;

    GPIOA_MODER &= ~((3U << 2) | (3U << 4));
    GPIOA_MODER |= ((1U << 2) | (1U << 4));

    GPIOC_MODER &= ~(3U << 26);
    GPIOC_MODER |= (1U << 26);

    GPIOA_BSRR = (1 << 17) | (1 << 18);
    GPIOC_BSRR = (1 << 13); // LED OFF

    i2c_init();
    delay_ms(50);

    uint8_t id = adxl_read_reg(0x00);
    g_telemetry.devid = id;
    if (id == 0xE5) {
        g_telemetry.status = 1;
        adxl_write_reg(0x2D, 0x08); // Measurement mode
        adxl_write_reg(0x31, 0x0B); // Full resolution +/-16g
        adxl_write_reg(0x2C, 0x0B); // 200Hz output
    }

    while (1) {
        // ==============================================================
        // 1. FORWARD STROKE (0 to ~40 DEGREES: EXACTLY 22 STEPS)
        // High Speed Instant Snap (Zero-ramp breakaway to maximum whip!)
        // ==============================================================
        GPIOA_BSRR = (1 << 1);  // DIR = Forward (0 -> 40 deg)
        GPIOC_BSRR = (1 << 13); // LED OFF
        delay_ms(150);

        uint32_t delay_val = 1800;
        for (int i = 0; i < 22; i++) {
            if (i < 4) delay_val -= 200; // Breakaway: 1800 -> 1000us
            GPIOA_BSRR = (1 << 2);
            delay_us(8);
            GPIOA_BSRR = (1 << 18);
            delay_us(delay_val);
        }

        // ==============================================================
        // 2. DEAD STOP (BRAKE LOCKED) - REAL-TIME VIBRATION SAMPLING
        // ==============================================================
        GPIOC_BSRR = (1 << 29); // LED ON (Brake Active)
        g_telemetry.stroke_count++;

        int32_t p_max = -999999;
        int32_t p_min = 999999;
        int32_t baseline = 0;
        int zero_crossings = 0;
        int last_sign = 0;

        // Sample for 2.0 seconds at ~100 Hz (200 samples)
        for (int s = 0; s < 200; s++) {
            int16_t rz = adxl_read_z();
            int32_t mz = (int32_t)rz * 39 / 10;
            g_telemetry.curr_raw_z = rz;
            g_telemetry.curr_mg_z = mz;
            g_telemetry.total_samples++;

            if (mz > p_max) p_max = mz;
            if (mz < p_min) p_min = mz;

            if (s == 0) baseline = mz;

            // Zero crossing detection relative to initial baseline
            int sign = (mz > baseline) ? 1 : -1;
            if (s > 0 && sign != last_sign) {
                zero_crossings++;
            }
            last_sign = sign;

            delay_ms(10);
        }

        g_telemetry.stroke_peak_max_z = p_max;
        g_telemetry.stroke_peak_min_z = p_min;
        g_telemetry.stroke_p2p_mg = (p_max - p_min);

        // Natural Frequency = (zero_crossings / 2) / 2.0 seconds = zero_crossings / 4.0
        // Multiply by 10 for fixed-point integer: (zero_crossings * 10) / 4 = zero_crossings * 5 / 2
        g_telemetry.est_freq_hz_x10 = (zero_crossings * 5) / 2;

        // ==============================================================
        // 3. RETURN STROKE (40 DEGREES BACK TO 0)
        // Gentle, smooth return creep (no shaking)
        // ==============================================================
        GPIOC_BSRR = (1 << 13); // LED OFF
        GPIOA_BSRR = (1 << 17); // DIR = Reverse (40 -> 0 deg)
        delay_ms(100);

        for (int i = 0; i < 22; i++) {
            GPIOA_BSRR = (1 << 2);
            delay_us(8);
            GPIOA_BSRR = (1 << 18);
            delay_us(4000); // 4ms smooth return
        }

        // Rest at home position before next stroke
        delay_ms(1200);
    }
}
