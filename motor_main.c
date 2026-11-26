#include <stdint.h>

// Registers for STM32F401
#define RCC_BASE        0x40023800
#define RCC_AHB1ENR     (*(volatile uint32_t *)(RCC_BASE + 0x30))

#define GPIOA_BASE      0x40020000
#define GPIOA_MODER     (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_BSRR      (*(volatile uint32_t *)(GPIOA_BASE + 0x18))

#define GPIOC_BASE      0x40020800
#define GPIOC_MODER     (*(volatile uint32_t *)(GPIOC_BASE + 0x00))
#define GPIOC_BSRR      (*(volatile uint32_t *)(GPIOC_BASE + 0x18))

// Linker symbols
extern uint32_t _estack;
extern uint32_t _etext;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

void Reset_Handler(void);
void Default_Handler(void);

__attribute__((section(".isr_vector")))
void (* const g_pfnVectors[])(void) = {
    (void (*)(void))((uint32_t)&_estack),
    Reset_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    0, 0, 0, 0,
    Default_Handler,
    Default_Handler,
    0,
    Default_Handler,
    Default_Handler
};

// Delay calibrated for 16MHz HSI clock (~1 microsecond per ~4 iterations)
static void delay_us(volatile uint32_t us) {
    volatile uint32_t count = us * 4;
    while (count--) {
        __asm__("nop");
    }
}

static void delay_ms(volatile uint32_t ms) {
    while (ms--) {
        delay_us(1000);
    }
}

void Reset_Handler(void) {
    uint32_t *src = &_etext;
    uint32_t *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;
    dst = &_sbss;
    while (dst < &_ebss) *dst++ = 0;

    // Enable Clocks for GPIOA (bit 0) and GPIOC (bit 2)
    RCC_AHB1ENR |= (1 << 0) | (1 << 2);
    volatile uint32_t dummy = RCC_AHB1ENR;
    (void)dummy;

    // Configure PA1 (DIR) and PA2 (STEP) as Outputs:
    // PA1: bits 3:2 = 01
    // PA2: bits 5:4 = 01
    GPIOA_MODER &= ~((3U << 2) | (3U << 4));
    GPIOA_MODER |= ((1U << 2) | (1U << 4));

    // Configure PC13 (Onboard LED) as Output:
    GPIOC_MODER &= ~(3U << 26);
    GPIOC_MODER |= (1U << 26);

    // Initial state: STEP=LOW, DIR=LOW, LED=OFF (HIGH)
    GPIOA_BSRR = (1 << 17) | (1 << 18); // Reset PA1 and PA2
    GPIOC_BSRR = (1 << 13);             // LED OFF

    while (1) {
        // --- 1. CLOCKWISE: 200 steps (1 full revolution in full-step mode) ---
        GPIOA_BSRR = (1 << 1);          // DIR = HIGH (PA1)
        GPIOC_BSRR = (1 << 29);         // LED ON during CW rotation
        delay_ms(10);

        for (int i = 0; i < 200; i++) {
            GPIOA_BSRR = (1 << 2);      // STEP = HIGH (PA2)
            delay_us(10);               // 10us HIGH pulse
            GPIOA_BSRR = (1 << 18);     // STEP = LOW (PA2)
            delay_us(2500);             // 2.5ms between steps (200 steps = 0.5 sec)
        }

        GPIOC_BSRR = (1 << 13);         // LED OFF
        delay_ms(500);                  // 500ms pause

        // --- 2. COUNTER-CLOCKWISE: 200 steps (1 full revolution) ---
        GPIOA_BSRR = (1 << 17);         // DIR = LOW (PA1)
        GPIOC_BSRR = (1 << 29);         // LED ON during CCW rotation
        delay_ms(10);

        for (int i = 0; i < 200; i++) {
            GPIOA_BSRR = (1 << 2);      // STEP = HIGH (PA2)
            delay_us(10);               // 10us HIGH pulse
            GPIOA_BSRR = (1 << 18);     // STEP = LOW (PA2)
            delay_us(2500);             // 2.5ms between steps
        }

        GPIOC_BSRR = (1 << 13);         // LED OFF
        delay_ms(500);                  // 500ms pause
    }
}

void Default_Handler(void) {
    while (1);
}
