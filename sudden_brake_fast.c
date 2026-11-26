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

// Cycle-exact 1us delay for Cortex-M4 at 16MHz HSI
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
    while (ms--) {
        delay_us(1000);
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

    // Configure PA1 (DIR) and PA2 (STEP)
    GPIOA_MODER &= ~((3U << 2) | (3U << 4));
    GPIOA_MODER |= ((1U << 2) | (1U << 4));

    // Configure PC13 (LED)
    GPIOC_MODER &= ~(3U << 26);
    GPIOC_MODER |= (1U << 26);

    GPIOA_BSRR = (1 << 17) | (1 << 18);
    GPIOC_BSRR = (1 << 13); // LED OFF

    while (1) {
        // ==========================================================
        // 1. HIGH SPEED SWEEP (300 STEPS = ~1.5 TURNS)
        // ==========================================================
        GPIOA_BSRR = (1 << 1);          // DIR = Forward
        GPIOC_BSRR = (1 << 13);         // LED OFF during motion
        delay_ms(300);

        uint32_t d = 1600; // Start at 1600us

        for (int i = 0; i < 300; i++) {
            // Rapid acceleration curve over first 120 steps down to 380us
            if (i < 120) {
                d -= 10;
            }

            GPIOA_BSRR = (1 << 2);      // STEP HIGH
            delay_us(6);
            GPIOA_BSRR = (1 << 18);     // STEP LOW
            delay_us(d);
        }

        // ==========================================================
        // 2. HARD INSTANT BRAKE!
        // Pulses stop completely. Motor locks solid with 100% torque.
        // ==========================================================
        GPIOC_BSRR = (1 << 29);         // LED ON (Braking & Vibration window)
        
        // Hold locked for 3.5 seconds so you can watch the ruler oscillate
        delay_ms(3500);

        // ==========================================================
        // 3. GENTLE RETURN TO HOME
        // ==========================================================
        GPIOC_BSRR = (1 << 13);         // LED OFF
        GPIOA_BSRR = (1 << 17);         // DIR = Reverse
        delay_ms(150);

        for (int i = 0; i < 300; i++) {
            GPIOA_BSRR = (1 << 2);
            delay_us(6);
            GPIOA_BSRR = (1 << 18);
            delay_us(1500);             // Smooth, steady return
        }

        // Pause 1.5 seconds at base before repeating
        delay_ms(1500);
    }
}
