#include <stdint.h>

#define RCC_BASE        0x40023800
#define RCC_AHB1ENR     (*(volatile uint32_t *)(RCC_BASE + 0x30))

#define GPIOA_BASE      0x40020000
#define GPIOA_MODER     (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_BSRR      (*(volatile uint32_t *)(GPIOA_BASE + 0x18))

#define GPIOC_BASE      0x40020800
#define GPIOC_MODER     (*(volatile uint32_t *)(GPIOC_BASE + 0x00))
#define GPIOC_BSRR      (*(volatile uint32_t *)(GPIOC_BASE + 0x18))

extern uint32_t _estack, _etext, _sdata, _edata, _sbss, _ebss;

void Reset_Handler(void);
void Default_Handler(void);

__attribute__((section(".isr_vector")))
void (* const g_pfnVectors[])(void) = {
    (void (*)(void))((uint32_t)&_estack),
    Reset_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler,
    0, 0, 0, 0, Default_Handler, Default_Handler, 0,
    Default_Handler, Default_Handler
};

static void delay_us(volatile uint32_t us) {
    volatile uint32_t count = us * 4;
    while (count--) __asm__("nop");
}

static void delay_ms(volatile uint32_t ms) {
    while (ms--) delay_us(1000);
}

// Function to move steps with a smooth trapezoidal acceleration ramp
static void step_motor(uint32_t total_steps, uint32_t min_delay_us, uint32_t max_delay_us, uint32_t ramp_steps) {
    uint32_t current_delay = max_delay_us;
    uint32_t delay_step = (max_delay_us - min_delay_us) / ramp_steps;

    for (uint32_t i = 0; i < total_steps; i++) {
        // Accelerate
        if (i < ramp_steps) {
            current_delay -= delay_step;
        }
        // Decelerate
        else if (i >= (total_steps - ramp_steps)) {
            current_delay += delay_step;
        }

        // Generate Step Pulse
        GPIOA_BSRR = (1 << 2);   // STEP HIGH
        delay_us(5);
        GPIOA_BSRR = (1 << 18);  // STEP LOW
        delay_us(current_delay);
    }
}

void Reset_Handler(void) {
    uint32_t *src = &_etext, *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;
    dst = &_sbss;
    while (dst < &_ebss) *dst++ = 0;

    // Enable Clocks for GPIOA and GPIOC
    RCC_AHB1ENR |= (1 << 0) | (1 << 2);
    volatile uint32_t dummy = RCC_AHB1ENR;
    (void)dummy;

    // Configure PA1 (DIR) and PA2 (STEP) as Outputs
    GPIOA_MODER &= ~((3U << 2) | (3U << 4));
    GPIOA_MODER |= ((1U << 2) | (1U << 4));

    // Configure PC13 (LED)
    GPIOC_MODER &= ~(3U << 26);
    GPIOC_MODER |= (1U << 26);

    GPIOA_BSRR = (1 << 17) | (1 << 18);
    GPIOC_BSRR = (1 << 13); // LED OFF

    while (1) {
        // --- CW Rotation: 4 full revolutions (800 full-steps), fast & ramped ---
        GPIOA_BSRR = (1 << 1);   // DIR = HIGH (CW)
        GPIOC_BSRR = (1 << 29);  // LED ON
        delay_ms(100);

        // Smooth acceleration from 2000us down to 500us (~1200 RPM top speed!)
        step_motor(800, 500, 2000, 150);

        GPIOC_BSRR = (1 << 13);  // LED OFF
        delay_ms(600);

        // --- CCW Rotation: 4 full revolutions (800 full-steps), fast & ramped ---
        GPIOA_BSRR = (1 << 17);  // DIR = LOW (CCW)
        GPIOC_BSRR = (1 << 29);  // LED ON
        delay_ms(100);

        step_motor(800, 500, 2000, 150);

        GPIOC_BSRR = (1 << 13);  // LED OFF
        delay_ms(600);
    }
}

void Default_Handler(void) { while(1); }
