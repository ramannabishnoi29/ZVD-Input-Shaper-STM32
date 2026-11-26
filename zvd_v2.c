/*
 * ZVD INPUT SHAPING SYSTEM v2 - Fixed timing, fast cycles
 * STM32F401 + ADXL345 + A4988 + NEMA17
 *
 * Fix: Default fn=5Hz so ZV/ZVD works from cycle 1
 * Fix: Faster 5ms sampling (200Hz) for accurate freq detection
 * Fix: 1-second wobble window (was 3s) - snappier cycles
 * Motion: 0 -> 50 degrees (28 full steps, full-speed instant snap)
 */
#include <stdint.h>

#define RCC_BASE    0x40023800
#define RCC_AHB1ENR (*(volatile uint32_t *)(RCC_BASE + 0x30))

#define GPIOA_BASE  0x40020000
#define GPIOA_MODER (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_BSRR  (*(volatile uint32_t *)(GPIOA_BASE + 0x18))

#define GPIOB_BASE  0x40020400
#define GPIOB_MODER (*(volatile uint32_t *)(GPIOB_BASE + 0x00))
#define GPIOB_OTYPER (*(volatile uint32_t *)(GPIOB_BASE + 0x04))
#define GPIOB_PUPDR (*(volatile uint32_t *)(GPIOB_BASE + 0x0C))
#define GPIOB_IDR   (*(volatile uint32_t *)(GPIOB_BASE + 0x10))
#define GPIOB_BSRR  (*(volatile uint32_t *)(GPIOB_BASE + 0x18))

#define GPIOC_BASE  0x40020800
#define GPIOC_MODER (*(volatile uint32_t *)(GPIOC_BASE + 0x00))
#define GPIOC_BSRR  (*(volatile uint32_t *)(GPIOC_BASE + 0x18))

extern uint32_t _estack, _etext, _sdata, _edata, _sbss, _ebss;

// ---- Telemetry block at 0x20000000 ----
typedef struct __attribute__((packed)) {
    uint32_t magic;             // 0xABCD1234
    uint8_t  devid;
    uint8_t  sensor_ok;
    uint8_t  phase;             // 1=UNSHAPED 2=ZV 3=ZVD 4=RETURNING
    uint8_t  cycle;

    uint32_t nat_freq_mhz;      // fn in mHz (5000 = 5.0 Hz)
    uint32_t damp_ppm;          // zeta * 1e6
    uint32_t Td_ms;             // Damped period ms

    uint32_t ZV_A1_pct;         // *100
    uint32_t ZV_A2_pct;
    uint32_t ZV_t2_ms;          // = Td/2

    uint32_t ZVD_A1_pct;
    uint32_t ZVD_A2_pct;
    uint32_t ZVD_A3_pct;
    uint32_t ZVD_t2_ms;         // = Td/2
    uint32_t ZVD_t3_ms;         // = Td

    int32_t  unshaped_p2p_mg;
    int32_t  zv_p2p_mg;
    int32_t  zvd_p2p_mg;

    int32_t  curr_z_mg;
    int32_t  curr_max_mg;
    int32_t  curr_min_mg;

    uint32_t zv_reduction_x10;
    uint32_t zvd_reduction_x10;
    uint32_t total_samples;
    uint32_t zero_crossings;
} Telem_t;

volatile Telem_t T __attribute__((section(".sensor_buffer")));

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

// ---- Float math (Cortex-M4 FPU) ----
static float my_exp(float x) {
    if (x < -4.0f) return 0.0183f;
    if (x >  4.0f) return 54.6f;
    float term = 1.0f, sum = 1.0f;
    for (int n = 1; n <= 12; n++) { term *= x/(float)n; sum += term; }
    return sum;
}

static float my_sqrt(float x) {
    if (x <= 0.0f) return 0.001f;
    float r = 1.0f;
    for (int i = 0; i < 20; i++) r = 0.5f*(r + x/r);
    return r;
}

// ---- Timing ----
static void delay_us(uint32_t us) {
    if (!us) return;
    __asm__ volatile(
        "1: mov r1, #3\n"
        "2: subs r1,r1,#1\n bne 2b\n"
        "   subs %0,%0,#1\n bne 1b\n"
        : "+r"(us) :: "r1","cc");
}
static void delay_ms(uint32_t ms) { while(ms--) delay_us(1000); }

// ---- I2C bit-bang PB6=SCL PB7=SDA ----
static inline void scl_hi(void){GPIOB_BSRR=(1<<6);}
static inline void scl_lo(void){GPIOB_BSRR=(1<<22);}
static inline void sda_hi(void){GPIOB_BSRR=(1<<7);}
static inline void sda_lo(void){GPIOB_BSRR=(1<<23);}
static inline int  sda_rd(void){return(GPIOB_IDR&(1<<7))?1:0;}

static void i2c_start(void){sda_hi();delay_us(2);scl_hi();delay_us(2);sda_lo();delay_us(2);scl_lo();delay_us(2);}
static void i2c_stop(void) {sda_lo();delay_us(2);scl_hi();delay_us(2);sda_hi();delay_us(2);}

static void i2c_wr(uint8_t b){
    for(int i=0;i<8;i++){
        if(b&0x80)sda_hi();else sda_lo();
        delay_us(2);scl_hi();delay_us(2);scl_lo();delay_us(2);b<<=1;
    }
    sda_hi();delay_us(2);scl_hi();delay_us(2);scl_lo();delay_us(2);
}

static uint8_t i2c_rd(int ack){
    uint8_t b=0;sda_hi();
    for(int i=0;i<8;i++){b<<=1;scl_hi();delay_us(2);if(sda_rd())b|=1;scl_lo();delay_us(2);}
    if(ack)sda_lo();else sda_hi();
    delay_us(2);scl_hi();delay_us(2);scl_lo();delay_us(2);sda_hi();
    return b;
}

static void adxl_wr(uint8_t reg,uint8_t val){i2c_start();i2c_wr(0xA6);i2c_wr(reg);i2c_wr(val);i2c_stop();}
static uint8_t adxl_rd(uint8_t reg){i2c_start();i2c_wr(0xA6);i2c_wr(reg);i2c_start();i2c_wr(0xA7);uint8_t v=i2c_rd(0);i2c_stop();return v;}

static int16_t adxl_z(void){
    i2c_start();i2c_wr(0xA6);i2c_wr(0x36);
    i2c_start();i2c_wr(0xA7);
    uint8_t lo=i2c_rd(1),hi=i2c_rd(0);i2c_stop();
    return(int16_t)((hi<<8)|lo);
}

// ---- Stepper ----
#define TOTAL_STEPS  28         // 50 degrees in full-step
#define SNAP_US      700        // 700us between steps = very fast snap
#define RETURN_US    4500

#define DIR_FW()  GPIOA_BSRR=(1<<1)
#define DIR_RV()  GPIOA_BSRR=(1<<17)
#define STEP_HI() GPIOA_BSRR=(1<<2)
#define STEP_LO() GPIOA_BSRR=(1<<18)
#define LED_ON()  GPIOC_BSRR=(1<<29)
#define LED_OFF() GPIOC_BSRR=(1<<13)

static void steps(int n, uint32_t us_interval){
    for(int i=0;i<n;i++){STEP_HI();delay_us(8);STEP_LO();delay_us(us_interval);}
}

// Sample Z for a time window (n samples, interval_ms each)
// Returns p2p vibration in mg, also updates telemetry
static int32_t measure_vibe(int n, uint32_t interval_ms){
    int32_t pmax=-999999, pmin=999999;
    int32_t base=0;
    int zc=0, last_sgn=0;

    for(int s=0;s<n;s++){
        int16_t rz=adxl_z();
        int32_t mz=(int32_t)rz*39/10;
        T.curr_z_mg=mz;
        T.total_samples++;
        if(mz>pmax)pmax=mz;
        if(mz<pmin)pmin=mz;
        if(s==0)base=mz;
        int sgn=(mz>=base)?1:-1;
        if(s>2 && sgn!=last_sgn)zc++;
        last_sgn=sgn;
        delay_ms(interval_ms);
    }
    T.curr_max_mg=pmax;
    T.curr_min_mg=pmin;
    T.zero_crossings=(uint32_t)zc;
    return pmax-pmin;
}

void Reset_Handler(void){
    // Data/BSS init
    uint32_t *s=&_etext,*d=&_sdata;
    while(d<&_edata)*d++=*s++;
    d=&_sbss;while(d<&_ebss)*d++=0;

    // Init telemetry defaults
    T.magic         =0xABCD1234;
    T.phase         =0;
    T.cycle         =0;
    T.total_samples =0;
    // Default: 5.0 Hz natural freq, zeta=0.05 -> works from cycle 1!
    T.nat_freq_mhz  =5000;
    T.damp_ppm      =50000;
    T.Td_ms         =200;

    // Enable GPIO clocks A, B, C
    RCC_AHB1ENR|=(1<<0)|(1<<1)|(1<<2);
    volatile uint32_t tmp=RCC_AHB1ENR;(void)tmp;

    // PA1=DIR, PA2=STEP as push-pull output
    GPIOA_MODER&=~((3U<<2)|(3U<<4));
    GPIOA_MODER|=((1U<<2)|(1U<<4));

    // PC13=LED
    GPIOC_MODER&=~(3U<<26);
    GPIOC_MODER|=(1U<<26);

    // PB6=SCL, PB7=SDA: open-drain + pull-up
    GPIOB_MODER&=~((3U<<12)|(3U<<14));
    GPIOB_MODER|=((1U<<12)|(1U<<14));
    GPIOB_OTYPER|=(1<<6)|(1<<7);
    GPIOB_PUPDR&=~((3U<<12)|(3U<<14));
    GPIOB_PUPDR|=((1U<<12)|(1U<<14));

    GPIOA_BSRR=(1<<17)|(1<<18);
    scl_hi();sda_hi();
    LED_OFF();
    delay_ms(100);

    // Init ADXL345
    uint8_t id=adxl_rd(0x00);
    T.devid=id;
    T.sensor_ok=(id==0xE5)?1:0;
    if(T.sensor_ok){
        adxl_wr(0x2D,0x08); // Measure mode
        adxl_wr(0x31,0x0B); // Full-res ±16g
        adxl_wr(0x2C,0x0D); // 800 Hz ODR
    }

    // 2 fast blinks = ready
    for(int i=0;i<2;i++){LED_ON();delay_ms(100);LED_OFF();delay_ms(100);}
    delay_ms(400);

    // ---- Shaper state (starts with default 5 Hz) ----
    float fn_hz    = 5.0f;
    float zeta     = 0.05f;
    int   params_valid = 1;  // Valid from start with defaults

    while(1){
        T.cycle++;

        // Compute shaper params from current fn_hz and zeta
        float Td_s  = 1.0f / fn_hz;
        float wd    = fn_hz * 6.28318f;
        (void)wd;
        float zp    = zeta * 3.14159f / my_sqrt(1.0f - zeta*zeta);
        float K     = my_exp(-zp);
        float K2    = K*K;

        // ZV: 2 impulses
        float za1 = 1.0f/(1.0f+K);
        float za2 = K/(1.0f+K);
        uint32_t zv_delay_ms = (uint32_t)(Td_s*500.0f); // Td/2 in ms
        int zv_s1 = (int)(za1*TOTAL_STEPS+0.5f);
        int zv_s2 = TOTAL_STEPS - zv_s1;

        // ZVD: 3 impulses
        float denom = (1.0f+K)*(1.0f+K);
        float da1 = 1.0f/denom;
        float da2 = 2.0f*K/denom;
        float da3 = K2/denom;
        uint32_t zvd_d2_ms = zv_delay_ms;          // Td/2
        uint32_t zvd_d3_ms = (uint32_t)(Td_s*1000.0f); // Td
        int zvd_s1 = (int)(da1*TOTAL_STEPS+0.5f);
        int zvd_s2 = (int)(da2*TOTAL_STEPS+0.5f);
        int zvd_s3 = TOTAL_STEPS - zvd_s1 - zvd_s2;
        if(zvd_s3<1) zvd_s3=1;

        // Update telemetry params
        T.nat_freq_mhz  =(uint32_t)(fn_hz*1000.0f);
        T.damp_ppm      =(uint32_t)(zeta*1000000.0f);
        T.Td_ms         =(uint32_t)(Td_s*1000.0f);
        T.ZV_A1_pct     =(uint32_t)(za1*100.0f);
        T.ZV_A2_pct     =(uint32_t)(za2*100.0f);
        T.ZV_t2_ms      =zv_delay_ms;
        T.ZVD_A1_pct    =(uint32_t)(da1*100.0f);
        T.ZVD_A2_pct    =(uint32_t)(da2*100.0f);
        T.ZVD_A3_pct    =(uint32_t)(da3*100.0f);
        T.ZVD_t2_ms     =zvd_d2_ms;
        T.ZVD_t3_ms     =zvd_d3_ms;

        // ============================================================
        // PHASE A: UNSHAPED — Instant full snap, measure vibration
        // ============================================================
        T.phase=1;
        DIR_FW(); LED_OFF();
        delay_ms(200);

        steps(TOTAL_STEPS, SNAP_US); // 28 steps in one shot

        LED_ON();
        // Sample 200 points at 5ms = 1 second window
        int32_t us_p2p = measure_vibe(200, 5);
        T.unshaped_p2p_mg = us_p2p;
        LED_OFF();

        // Update fn from zero crossings (1-second window)
        int zc=(int)T.zero_crossings;
        if(zc>=2){
            float new_fn = (float)zc / 2.0f; // Hz (1s window)
            if(new_fn>=1.0f && new_fn<=40.0f){
                fn_hz = 0.6f*fn_hz + 0.4f*new_fn; // Smooth update
            }
        }

        // Return home
        T.phase=4;
        DIR_RV(); delay_ms(150);
        steps(TOTAL_STEPS, RETURN_US);
        delay_ms(500);

        // ============================================================
        // PHASE B: ZV SHAPED — 2-impulse destructive interference
        // ============================================================
        T.phase=2;
        DIR_FW(); LED_OFF();
        delay_ms(200);

        // Impulse 1 at t=0
        steps(zv_s1, SNAP_US);
        // Wait Td/2 — this is the magic timing!
        delay_ms(zv_delay_ms);
        // Impulse 2 cancels the vibration from impulse 1
        steps(zv_s2, SNAP_US);

        LED_ON();
        int32_t zv_p2p = measure_vibe(200, 5);
        T.zv_p2p_mg = zv_p2p;
        LED_OFF();

        // Return home
        T.phase=4;
        DIR_RV(); delay_ms(150);
        steps(TOTAL_STEPS, RETURN_US);
        delay_ms(500);

        // ============================================================
        // PHASE C: ZVD SHAPED — 3-impulse, robust zero vibration
        // ============================================================
        T.phase=3;
        DIR_FW(); LED_OFF();
        delay_ms(200);

        // Impulse 1 at t=0
        steps(zvd_s1, SNAP_US);
        // Wait Td/2
        delay_ms(zvd_d2_ms);
        // Impulse 2 at t=Td/2
        steps(zvd_s2, SNAP_US);
        // Wait another Td/2
        delay_ms(zvd_d3_ms - zvd_d2_ms);
        // Impulse 3 at t=Td
        steps(zvd_s3, SNAP_US);

        LED_ON();
        int32_t zvd_p2p = measure_vibe(200, 5);
        T.zvd_p2p_mg = zvd_p2p;
        LED_OFF();

        // Reduction percentages
        if(T.unshaped_p2p_mg>50){
            int32_t zr=((T.unshaped_p2p_mg-zv_p2p)*1000)/T.unshaped_p2p_mg;
            int32_t zdr=((T.unshaped_p2p_mg-zvd_p2p)*1000)/T.unshaped_p2p_mg;
            T.zv_reduction_x10  =(uint32_t)(zr<0?0:zr);
            T.zvd_reduction_x10 =(uint32_t)(zdr<0?0:zdr);
        }

        // Return home and rest
        T.phase=4;
        DIR_RV(); delay_ms(150);
        steps(TOTAL_STEPS, RETURN_US);
        T.phase=0;
        delay_ms(1000);
    }
}
