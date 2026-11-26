#include <stdint.h>

#define RCC_BASE        0x40023800
#define RCC_AHB1ENR     (*(volatile uint32_t *)(RCC_BASE + 0x30))
#define RCC_APB2ENR     (*(volatile uint32_t *)(RCC_BASE + 0x44))

#define GPIOA_BASE      0x40020000
#define GPIOA_MODER     (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_AFRH      (*(volatile uint32_t *)(GPIOA_BASE + 0x24))
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

#define USART1_BASE     0x40011000
#define USART1_SR       (*(volatile uint32_t *)(USART1_BASE + 0x00))
#define USART1_DR       (*(volatile uint32_t *)(USART1_BASE + 0x04))
#define USART1_BRR      (*(volatile uint32_t *)(USART1_BASE + 0x08))
#define USART1_CR1      (*(volatile uint32_t *)(USART1_BASE + 0x0C))

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

// Data Structure placed directly at 0x20000000 in SRAM
typedef struct __attribute__((packed)) {
    uint32_t magic;         // 0xAD345001
    uint8_t  devid;         // Should be 0xE5
    uint8_t  status;        // 1 = Active / Valid Data
    int16_t  raw_x;         // Raw 16-bit counts
    int16_t  raw_y;
    int16_t  raw_z;
    int32_t  mg_x;          // milli-g (1000 mg = 1.0g)
    int32_t  mg_y;
    int32_t  mg_z;
    int32_t  peak_max_z;    // Maximum Z recorded during wobble
    int32_t  peak_min_z;    // Minimum Z recorded during wobble
    uint32_t sample_count;  // Running counter of samples read
} SensorBuffer_t;

volatile SensorBuffer_t g_sensor __attribute__((section(".sensor_buffer")));

// Accurate Microsecond delay
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

// I2C Bit-Bang Driver on PB6 (SCL) and PB7 (SDA)
static inline void scl_high(void) { GPIOB_BSRR = (1 << 6); }
static inline void scl_low(void)  { GPIOB_BSRR = (1 << 22); }
static inline void sda_high(void) { GPIOB_BSRR = (1 << 7); }
static inline void sda_low(void)  { GPIOB_BSRR = (1 << 23); }
static inline int  sda_read(void) { return (GPIOB_IDR & (1 << 7)) ? 1 : 0; }

static void i2c_init(void) {
    RCC_AHB1ENR |= (1 << 1); // GPIOB Clock
    volatile uint32_t d = RCC_AHB1ENR;
    (void)d;

    GPIOB_MODER &= ~((3U << 12) | (3U << 14));
    GPIOB_MODER |= ((1U << 12) | (1U << 14));
    GPIOB_OTYPER |= (1 << 6) | (1 << 7); // Open drain
    GPIOB_PUPDR &= ~((3U << 12) | (3U << 14));
    GPIOB_PUPDR |= ((1U << 12) | (1U << 14)); // Pull-up

    scl_high();
    sda_high();
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
        if (byte & 0x80) sda_high();
        else sda_low();
        delay_us(2);
        scl_high(); delay_us(2);
        scl_low();  delay_us(2);
        byte <<= 1;
    }
    sda_high();
    delay_us(2);
    scl_high(); delay_us(2);
    int ack = !sda_read();
    scl_low();  delay_us(2);
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
    delay_us(2);
    scl_high(); delay_us(2);
    scl_low();  delay_us(2);
    sda_high();
    return byte;
}

static uint8_t adxl_read_reg(uint8_t reg) {
    i2c_start();
    i2c_write(0xA6); // 0x53 Write
    i2c_write(reg);
    i2c_start();
    i2c_write(0xA7); // 0x53 Read
    uint8_t val = i2c_read(0);
    i2c_stop();
    return val;
}

static void adxl_write_reg(uint8_t reg, uint8_t val) {
    i2c_start();
    i2c_write(0xA6);
    i2c_write(reg);
    i2c_write(val);
    i2c_stop();
}

static void adxl_sample(void) {
    i2c_start();
    i2c_write(0xA6);
    i2c_write(0x32); // DATAX0 register address
    i2c_start();
    i2c_write(0xA7);

    uint8_t xl = i2c_read(1);
    uint8_t xh = i2c_read(1);
    uint8_t yl = i2c_read(1);
    uint8_t yh = i2c_read(1);
    uint8_t zl = i2c_read(1);
    uint8_t zh = i2c_read(0);
    i2c_stop();

    int16_t rx = (int16_t)((xh << 8) | xl);
    int16_t ry = (int16_t)((yh << 8) | yl);
    int16_t rz = (int16_t)((zh << 8) | zl);

    g_sensor.raw_x = rx;
    g_sensor.raw_y = ry;
    g_sensor.raw_z = rz;

    // Full-Res Scale: 3.9 mg / LSB (approx * 39 / 10)
    int32_t mx = (int32_t)rx * 39 / 10;
    int32_t my = (int32_t)ry * 39 / 10;
    int32_t mz = (int32_t)rz * 39 / 10;

    g_sensor.mg_x = mx;
    g_sensor.mg_y = my;
    g_sensor.mg_z = mz;
    g_sensor.sample_count++;

    if (mz > g_sensor.peak_max_z) g_sensor.peak_max_z = mz;
    if (mz < g_sensor.peak_min_z) g_sensor.peak_min_z = mz;
}

// USART1 on PA9 (TX) at 115200 baud
static void uart_init(void) {
    RCC_APB2ENR |= (1 << 4); // USART1 Clock
    // PA9 as Alternate Function AF7
    GPIOA_MODER &= ~(3U << 18);
    GPIOA_MODER |= (2U << 18);
    GPIOA_AFRH &= ~(0xFU << 4);
    GPIOA_AFRH |= (0x7U << 4); // AF7 (USART1_TX)

    USART1_BRR = 0x8B;         // 115200 baud at 16MHz
    USART1_CR1 = (1 << 13) | (1 << 3); // UE | TE
}

static void uart_putc(char c) {
    while (!(USART1_SR & (1 << 7))); // Wait TXE
    USART1_DR = (uint8_t)c;
}

static void uart_print(const char *s) {
    while (*s) uart_putc(*s++);
}

static void uart_print_int(int32_t v) {
    char buf[12];
    int i = 0;
    if (v < 0) { uart_putc('-'); v = -v; }
    if (v == 0) { uart_putc('0'); return; }
    while (v > 0) {
        buf[i++] = (v % 10) + '0';
        v /= 10;
    }
    while (i > 0) uart_putc(buf[--i]);
}

void Reset_Handler(void) {
    uint32_t *src = &_etext, *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;
    dst = &_sbss;
    while (dst < &_ebss) *dst++ = 0;

    // Zero out sensor buffer
    g_sensor.magic = 0xAD345001;
    g_sensor.devid = 0x00;
    g_sensor.status = 0;
    g_sensor.sample_count = 0;
    g_sensor.peak_max_z = -999999;
    g_sensor.peak_min_z = 999999;

    RCC_AHB1ENR |= (1 << 0) | (1 << 1) | (1 << 2);
    volatile uint32_t dummy = RCC_AHB1ENR;
    (void)dummy;

    // PA1 (DIR) and PA2 (STEP)
    GPIOA_MODER &= ~((3U << 2) | (3U << 4));
    GPIOA_MODER |= ((1U << 2) | (1U << 4));

    // PC13 (LED)
    GPIOC_MODER &= ~(3U << 26);
    GPIOC_MODER |= (1U << 26);

    GPIOA_BSRR = (1 << 17) | (1 << 18);
    GPIOC_BSRR = (1 << 13); // LED OFF

    i2c_init();
    uart_init();
    delay_ms(50);

    // Read ADXL345 Device ID
    uint8_t id = adxl_read_reg(0x00);
    g_sensor.devid = id;

    if (id == 0xE5) {
        g_sensor.status = 1;
        adxl_write_reg(0x2D, 0x08); // Measurement mode
        adxl_write_reg(0x31, 0x0B); // Full resolution, +/-16g
        adxl_write_reg(0x2C, 0x0B); // 200 Hz data output rate

        // Confirmation flash
        for (int k = 0; k < 2; k++) {
            GPIOC_BSRR = (1 << 29); delay_ms(60);
            GPIOC_BSRR = (1 << 13); delay_ms(60);
        }
    }

    while (1) {
        // Sample sensor before movement
        adxl_sample();

        // 1. VIOLENT SNAP: 25 STEPS (45 DEGREES, < 50 DEG)
        GPIOA_BSRR = (1 << 1);          // DIR = Forward
        GPIOC_BSRR = (1 << 13);
        delay_ms(150);

        // Reset peak detector for the new stroke
        g_sensor.peak_max_z = -999999;
        g_sensor.peak_min_z = 999999;

        uint32_t delay_val = 2200;
        for (int i = 0; i < 25; i++) {
            if (i < 6) delay_val -= 200;
            GPIOA_BSRR = (1 << 2);
            delay_us(8);
            GPIOA_BSRR = (1 << 18);
            delay_us(delay_val);
        }

        // 2. SUDDEN BRAKE & ACTIVE SAMPLING DURING WOBBLE
        GPIOC_BSRR = (1 << 29); // LED ON

        // Sample sensor rapidly during the 2.5 second wobble
        for (int s = 0; s < 250; s++) {
            adxl_sample();
            delay_ms(10); // 100 Hz sampling
        }

        // Output summary over UART (if connected)
        uart_print("ADXL [mg]: X=");
        uart_print_int(g_sensor.mg_x);
        uart_print(" Y=");
        uart_print_int(g_sensor.mg_y);
        uart_print(" Z=");
        uart_print_int(g_sensor.mg_z);
        uart_print(" | Wobble Peak Z=[");
        uart_print_int(g_sensor.peak_min_z);
        uart_print(" to ");
        uart_print_int(g_sensor.peak_max_z);
        uart_print("]\r\n");

        // 3. GENTLE RETURN
        GPIOC_BSRR = (1 << 13);
        GPIOA_BSRR = (1 << 17); // DIR = Reverse
        delay_ms(100);

        for (int i = 0; i < 25; i++) {
            GPIOA_BSRR = (1 << 2);
            delay_us(8);
            GPIOA_BSRR = (1 << 18);
            delay_us(4000);
        }

        // Idle sample during rest
        for (int s = 0; s < 100; s++) {
            adxl_sample();
            delay_ms(15);
        }
    }
}
