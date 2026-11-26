#include <stdint.h>

// Register definitions for STM32F401
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

// Cycle-exact delay
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

// ====================================================================
// ROBUST I2C DRIVER ON PB6 (SCL) and PB7 (SDA)
// ====================================================================
static inline void scl_high(void) { GPIOB_BSRR = (1 << 6); }
static inline void scl_low(void)  { GPIOB_BSRR = (1 << 22); }
static inline void sda_high(void) { GPIOB_BSRR = (1 << 7); }
static inline void sda_low(void)  { GPIOB_BSRR = (1 << 23); }
static inline int  sda_read(void) { return (GPIOB_IDR & (1 << 7)) ? 1 : 0; }

static void i2c_init(void) {
    // Enable GPIOB Clock (bit 1)
    RCC_AHB1ENR |= (1 << 1);
    volatile uint32_t d = RCC_AHB1ENR;
    (void)d;

    // PB6, PB7 as Open-Drain Outputs with internal pull-up
    // MODER: 01 for PB6 (bits 13:12), 01 for PB7 (bits 15:14)
    GPIOB_MODER &= ~((3U << 12) | (3U << 14));
    GPIOB_MODER |= ((1U << 12) | (1U << 14));

    // Open drain: bits 6, 7
    GPIOB_OTYPER |= (1 << 6) | (1 << 7);

    // Pull-up: 01 for PB6, 01 for PB7
    GPIOB_PUPDR &= ~((3U << 12) | (3U << 14));
    GPIOB_PUPDR |= ((1U << 12) | (1U << 14));

    scl_high();
    sda_high();
    delay_us(10);
}

static void i2c_start(void) {
    sda_high(); delay_us(3);
    scl_high(); delay_us(3);
    sda_low();  delay_us(3);
    scl_low();  delay_us(3);
}

static void i2c_stop(void) {
    sda_low();  delay_us(3);
    scl_high(); delay_us(3);
    sda_high(); delay_us(3);
}

static int i2c_write_byte(uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        if (byte & 0x80) sda_high();
        else sda_low();
        delay_us(2);
        scl_high(); delay_us(3);
        scl_low();  delay_us(2);
        byte <<= 1;
    }
    sda_high(); // release SDA for ACK
    delay_us(2);
    scl_high(); delay_us(3);
    int ack = !sda_read();
    scl_low();  delay_us(2);
    return ack;
}

static uint8_t i2c_read_byte(int ack) {
    uint8_t byte = 0;
    sda_high();
    for (int i = 0; i < 8; i++) {
        byte <<= 1;
        scl_high(); delay_us(3);
        if (sda_read()) byte |= 1;
        scl_low(); delay_us(2);
    }
    if (ack) sda_low(); else sda_high();
    delay_us(2);
    scl_high(); delay_us(3);
    scl_low();  delay_us(2);
    sda_high();
    return byte;
}

static uint8_t adxl_read_reg(uint8_t reg) {
    i2c_start();
    i2c_write_byte(0x53 << 1); // ADXL345 Write address (0xA6)
    i2c_write_byte(reg);
    i2c_start();
    i2c_write_byte((0x53 << 1) | 1); // ADXL345 Read address (0xA7)
    uint8_t val = i2c_read_byte(0);  // NACK last byte
    i2c_stop();
    return val;
}

static void adxl_write_reg(uint8_t reg, uint8_t val) {
    i2c_start();
    i2c_write_byte(0x53 << 1);
    i2c_write_byte(reg);
    i2c_write_byte(val);
    i2c_stop();
}

// ====================================================================
// MAIN PROGRAM: SUDDEN SNAP (< 50 DEG) + WOBBLE + ADXL345 SENSING
// ====================================================================
void Reset_Handler(void) {
    uint32_t *src = &_etext, *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;
    dst = &_sbss;
    while (dst < &_ebss) *dst++ = 0;

    // Enable Clocks for GPIOA, GPIOB, GPIOC
    RCC_AHB1ENR |= (1 << 0) | (1 << 1) | (1 << 2);
    volatile uint32_t dummy = RCC_AHB1ENR;
    (void)dummy;

    // Configure PA1 (DIR) and PA2 (STEP) as Outputs
    GPIOA_MODER &= ~((3U << 2) | (3U << 4));
    GPIOA_MODER |= ((1U << 2) | (1U << 4));

    // Configure PC13 (LED)
    GPIOC_MODER &= ~(3U << 26);
    GPIOC_MODER |= (1U << 26);

    GPIOA_BSRR = (1 << 17) | (1 << 18); // STEP=LOW, DIR=LOW
    GPIOC_BSRR = (1 << 13);             // LED OFF

    // Initialize I2C on PB6/PB7
    i2c_init();
    delay_ms(50);

    // Check ADXL345 DEVID (should be 0xE5)
    uint8_t devid = adxl_read_reg(0x00);
    if (devid == 0xE5) {
        // Sensor recognized! Configure for Measurement mode (+-16g)
        adxl_write_reg(0x2D, 0x08); // POWER_CTL: Measurement Mode
        adxl_write_reg(0x31, 0x0B); // DATA_FORMAT: Full Res, +/- 16g
        
        // Confirmation flash (2 quick blinks)
        GPIOC_BSRR = (1 << 29); delay_ms(80); GPIOC_BSRR = (1 << 13); delay_ms(80);
        GPIOC_BSRR = (1 << 29); delay_ms(80); GPIOC_BSRR = (1 << 13); delay_ms(300);
    }

    while (1) {
        // ==============================================================
        // 1. EXPLOSIVE SUDDEN START + SUDDEN STOP
        // Movement: EXACTLY 24 STEPS (43.2 degrees, strictly < 50 deg!)
        // Speed: 850 steps/second with ZERO acceleration ramp!
        // Instant top speed -> Instant dead halt = MAXIMUM WOBBLE!
        // ==============================================================
        GPIOA_BSRR = (1 << 1);          // DIR = Forward
        GPIOC_BSRR = (1 << 13);         // LED OFF during stroke
        delay_ms(300);

        for (int i = 0; i < 24; i++) {
            GPIOA_BSRR = (1 << 2);      // STEP HIGH
            delay_us(8);
            GPIOA_BSRR = (1 << 18);     // STEP LOW
            delay_us(1150);             // 1.15ms per step (Lightning snap in ~27 milliseconds!)
        }

        // ==============================================================
        // 2. SUDDEN BRAKE & OBSERVATION WINDOW
        // Motor locks dead solid. Ruler whips & wobbles violently.
        // LED turns ON during wobble period.
        // ==============================================================
        GPIOC_BSRR = (1 << 29);         // LED ON (Vibration Window)

        // Dwell for 3 seconds to let ruler oscillate and settle
        delay_ms(3000);

        // ==============================================================
        // 3. GENTLE RETURN TO HOME (Zero-shake return)
        // ==============================================================
        GPIOC_BSRR = (1 << 13);         // LED OFF
        GPIOA_BSRR = (1 << 17);         // DIR = Reverse
        delay_ms(150);

        for (int i = 0; i < 24; i++) {
            GPIOA_BSRR = (1 << 2);
            delay_us(8);
            GPIOA_BSRR = (1 << 18);
            delay_us(5000);             // Slow 5ms gentle return (smooth, no vibration)
        }

        // Pause 1.5 seconds at 0 degrees before the next snap
        delay_ms(1500);
    }
}
