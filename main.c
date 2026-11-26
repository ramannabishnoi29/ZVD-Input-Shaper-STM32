#include <stdint.h>

// Memory and Register definitions for STM32F401
#define RCC_BASE        0x40023800
#define RCC_AHB1ENR     (*(volatile uint32_t *)(RCC_BASE + 0x30))

#define GPIOC_BASE      0x40020800
#define GPIOC_MODER     (*(volatile uint32_t *)(GPIOC_BASE + 0x00))
#define GPIOC_ODR       (*(volatile uint32_t *)(GPIOC_BASE + 0x14))
#define GPIOC_BSRR      (*(volatile uint32_t *)(GPIOC_BASE + 0x18))

// Forward declarations
void Reset_Handler(void);
void Default_Handler(void);

// Linker script symbols
extern uint32_t _estack;
extern uint32_t _etext;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

// Vector Table
__attribute__((section(".isr_vector")))
void (* const g_pfnVectors[])(void) = {
    (void (*)(void))((uint32_t)&_estack), // Initial Stack Pointer
    Reset_Handler,                        // Reset Handler
    Default_Handler,                      // NMI
    Default_Handler,                      // HardFault
    Default_Handler,                      // MemManage
    Default_Handler,                      // BusFault
    Default_Handler,                      // UsageFault
    0, 0, 0, 0,                           // Reserved
    Default_Handler,                      // SVCall
    Default_Handler,                      // Debug Monitor
    0,                                    // Reserved
    Default_Handler,                      // PendSV
    Default_Handler                       // SysTick
};

static void delay_cycles(volatile uint32_t count) {
    while (count--) {
        __asm__("nop");
    }
}

void Reset_Handler(void) {
    // Copy initialized data from FLASH to RAM
    uint32_t *src = &_etext;
    uint32_t *dst = &_sdata;
    while (dst < &_edata) {
        *dst++ = *src++;
    }

    // Zero fill the bss section
    dst = &_sbss;
    while (dst < &_ebss) {
        *dst++ = 0;
    }

    // 1. Enable Clock for GPIOC (bit 2 of RCC_AHB1ENR)
    RCC_AHB1ENR |= (1 << 2);

    // Read back to ensure clock is enabled
    volatile uint32_t dummy = RCC_AHB1ENR;
    (void)dummy;

    // 2. Configure PC13 as Output:
    // Bits 27:26 of GPIOC_MODER: 01 = General purpose output mode
    GPIOC_MODER &= ~(3U << 26);
    GPIOC_MODER |= (1U << 26);

    // Turn LED OFF initially (Active-LOW: Set bit 13 in BSRR)
    GPIOC_BSRR = (1 << 13);

    // Heartbeat blink loop
    // At default 16 MHz HSI, delay_cycles(200000) is approx ~100ms
    while (1) {
        // Flash 1
        GPIOC_BSRR = (1 << 29); // Reset bit 13 -> LED ON
        delay_cycles(250000);
        GPIOC_BSRR = (1 << 13); // Set bit 13 -> LED OFF
        delay_cycles(250000);

        // Flash 2
        GPIOC_BSRR = (1 << 29); // Reset bit 13 -> LED ON
        delay_cycles(250000);
        GPIOC_BSRR = (1 << 13); // Set bit 13 -> LED OFF
        delay_cycles(1500000);  // Pause ~600-700ms
    }
}

void Default_Handler(void) {
    while (1);
}
