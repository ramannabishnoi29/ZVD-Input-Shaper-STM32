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
void Default_Handler(void) { while(1); }

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

void Reset_Handler(void) {
    uint32_t *src = &_etext, *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;
    dst = &_sbss;
    while (dst < &_ebss) *dst++ = 0;

    // Enable Clocks for GPIOA (PA1, PA2) and GPIOC (PC13 LED)
    RCC_AHB1ENR |= (1 << 0) | (1 << 2);
    volatile uint32_t dummy = RCC_AHB1ENR;
    (void)dummy;

    GPIOA_MODER &= ~((3U << 2) | (3U << 4));
    GPIOA_MODER |= ((1U << 2) | (1U << 4));

    GPIOC_MODER &= ~(3U << 26);
    GPIOC_MODER |= (1U << 26);

    GPIOA_BSRR = (1 << 17) | (1 << 18);
    GPIOC_BSRR = (1 << 13); // LED OFF

    while (1) {
        // ==============================================================
        // 1. FAST MOTION + INSTANT BRAKE (VIBRATION EXCITATION TEST)
        // ==============================================================
        GPIOA_BSRR = (1 << 1);          // DIR = Forward
        GPIOC_BSRR = (1 << 13);         // LED OFF during motion
        delay_ms(200);

        // Move 100 steps (90 degrees in full-step, or 45 deg in half-step)
        // Ramps up quickly to 800us per step, then HALTS SUDDENLY!
        uint32_t delay_val = 2200;
        for (int i = 0; i < 100; i++) {
            // Quick ramp to high speed over first 30 steps
            if (i < 30) {
                delay_val -= 45; // ramps down to ~850us
            }

            GPIOA_BSRR = (1 << 2);      // STEP HIGH
            delay_us(8);
            GPIOA_BSRR = (1 << 18);     // STEP LOW
            delay_us(delay_val);
        }

        // ==============================================================
        // 2. SUDDEN BRAKE APPLIED HERE!
        // No step pulses sent. Motor coils stay ENERGIZED (locked).
        // ==============================================================
        GPIOC_BSRR = (1 << 29);         // LED ON (indicates BRAKE ACTIVE & VIBRATING)
        
        // Dwell for 3.5 seconds to watch the beam vibrate until damped
        delay_ms(3500);

        // ==============================================================
        // 3. GENTLE RETURN (Slow creep back to start without shaking)
        // ==============================================================
        GPIOC_BSRR = (1 << 13);         // LED OFF
        GPIOA_BSRR = (1 << 17);         // DIR = Reverse
        delay_ms(100);

        for (int i = 0; i < 100; i++) {
            GPIOA_BSRR = (1 << 2);
            delay_us(8);
            GPIOA_BSRR = (1 << 18);
            delay_us(4000);             // Slow, quiet, gentle return (4ms per step)
        }

        // Pause 1.5 seconds at home position before repeating
        delay_ms(1500);
    }
}
