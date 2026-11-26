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

    // Startup visual indication (3 fast blinks so you know it's alive)
    for (int k = 0; k < 3; k++) {
        GPIOC_BSRR = (1 << 29); delay_ms(60);
        GPIOC_BSRR = (1 << 13); delay_ms(60);
    }
    delay_ms(500);

    while (1) {
        // ==============================================================
        // 1. VIOLENT SNAP: 25 STEPS (EXACTLY 45 DEGREES, < 50 DEG)
        // Steeper than steep: 6 breakaway steps to conquer ruler inertia,
        // then full speed snap, then DEAD INSTANT HALT!
        // ==============================================================
        GPIOA_BSRR = (1 << 1);          // DIR = Forward
        GPIOC_BSRR = (1 << 13);         // LED OFF during motion
        delay_ms(200);

        uint32_t delay_val = 2200;

        for (int i = 0; i < 25; i++) {
            // Rapid breakaway ramp over first 6 steps so the heavy ruler doesn't stall
            if (i < 6) {
                delay_val -= 200; // ramps from 2200 down to 1000us
            }

            GPIOA_BSRR = (1 << 2);      // STEP HIGH
            delay_us(8);
            GPIOA_BSRR = (1 << 18);     // STEP LOW
            delay_us(delay_val);
        }

        // ==============================================================
        // 2. DEAD STOP (BRAKE LOCKED) - WOBBLE PERIOD
        // ==============================================================
        GPIOC_BSRR = (1 << 29);         // LED ON (Vibrating)
        delay_ms(3000);                 // Dwell 3.0s to let ruler oscillate

        // ==============================================================
        // 3. GENTLE RETURN TO ZERO (Smooth creep back)
        // ==============================================================
        GPIOC_BSRR = (1 << 13);         // LED OFF
        GPIOA_BSRR = (1 << 17);         // DIR = Reverse
        delay_ms(150);

        for (int i = 0; i < 25; i++) {
            GPIOA_BSRR = (1 << 2);
            delay_us(8);
            GPIOA_BSRR = (1 << 18);
            delay_us(4000);             // Slow 4ms creep back (zero vibration)
        }

        // Rest at home position
        delay_ms(1500);
    }
}
