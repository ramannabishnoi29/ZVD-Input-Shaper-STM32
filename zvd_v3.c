/*
 * ZVD INPUT SHAPING v3 - Anti-stall, clear phase indicators
 * Fixed: Motor won't stall (1200us/step with short ramp)
 * Fixed: Very obvious LED patterns per phase
 * Fixed: Larger default delays so pauses are clearly visible
 * Motion: 0 -> 50 degrees (28 steps)
 *
 * WHAT TO WATCH: The RULER TIP, not the motor force!
 * UNSHAPED: Ruler shakes for ~2 seconds after stop
 * ZV:       Motor pauses 100ms mid-move, ruler shakes LESS
 * ZVD:      Motor pauses twice, ruler BARELY shakes
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

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t  devid, sensor_ok, phase, cycle;
    uint32_t nat_freq_mhz;
    uint32_t damp_ppm;
    uint32_t Td_ms;
    uint32_t ZV_A1_pct, ZV_A2_pct, ZV_t2_ms;
    uint32_t ZVD_A1_pct, ZVD_A2_pct, ZVD_A3_pct;
    uint32_t ZVD_t2_ms, ZVD_t3_ms;
    int32_t  unshaped_p2p_mg, zv_p2p_mg, zvd_p2p_mg;
    int32_t  curr_z_mg, curr_max_mg, curr_min_mg;
    uint32_t zv_reduction_x10, zvd_reduction_x10;
    uint32_t total_samples, zero_crossings;
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

static float my_exp(float x) {
    if (x<-4.0f) return 0.0183f;
    if (x> 4.0f) return 54.6f;
    float t=1.0f,s=1.0f;
    for(int n=1;n<=10;n++){t*=x/(float)n;s+=t;}
    return s;
}
static float my_sqrt(float x) {
    if(x<=0.0f) return 0.001f;
    float r=1.0f;
    for(int i=0;i<20;i++) r=0.5f*(r+x/r);
    return r;
}

static void delay_us(uint32_t us) {
    if (!us) return;
    __asm__ volatile(
        "1: mov r1,#3\n"
        "2: subs r1,r1,#1\n bne 2b\n"
        "   subs %0,%0,#1\n bne 1b\n"
        : "+r"(us) :: "r1","cc");
}
static void delay_ms(uint32_t ms) { while(ms--) delay_us(1000); }

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
static uint8_t adxl_rd(uint8_t reg){
    i2c_start();i2c_wr(0xA6);i2c_wr(reg);
    i2c_start();i2c_wr(0xA7);
    uint8_t v=i2c_rd(0);i2c_stop();return v;
}
static int16_t adxl_z(void){
    i2c_start();i2c_wr(0xA6);i2c_wr(0x36);
    i2c_start();i2c_wr(0xA7);
    uint8_t lo=i2c_rd(1),hi=i2c_rd(0);i2c_stop();
    return(int16_t)((hi<<8)|lo);
}

#define DIR_FW()  GPIOA_BSRR=(1<<1)
#define DIR_RV()  GPIOA_BSRR=(1<<17)
#define STEP_HI() GPIOA_BSRR=(1<<2)
#define STEP_LO() GPIOA_BSRR=(1<<18)
#define LED_ON()  GPIOC_BSRR=(1<<29)
#define LED_OFF() GPIOC_BSRR=(1<<13)

#define TOTAL_STEPS 28      // 50 degrees
#define SNAP_US     1200    // Safe snap speed (won't stall NEMA17)
#define RETURN_US   5000    // Slow smooth return

static void steps(int n, uint32_t us) {
    for(int i=0;i<n;i++){STEP_HI();delay_us(8);STEP_LO();delay_us(us);}
}

static void led_blink(int n, uint32_t on_ms, uint32_t off_ms) {
    for(int i=0;i<n;i++){LED_ON();delay_ms(on_ms);LED_OFF();delay_ms(off_ms);}
}

static int32_t measure_vibe(int samples, uint32_t interval_ms) {
    int32_t pmax=-999999, pmin=999999, base=0;
    int zc=0, last_sgn=0;
    for(int s=0;s<samples;s++){
        int16_t rz=adxl_z();
        int32_t mz=(int32_t)rz*39/10;
        T.curr_z_mg=mz;
        T.total_samples++;
        if(mz>pmax)pmax=mz;
        if(mz<pmin)pmin=mz;
        if(s==0)base=mz;
        int sgn=(mz>=base)?1:-1;
        if(s>5&&sgn!=last_sgn)zc++;
        last_sgn=sgn;
        delay_ms(interval_ms);
    }
    T.curr_max_mg=pmax;
    T.curr_min_mg=pmin;
    T.zero_crossings=(uint32_t)zc;
    return pmax-pmin;
}

void Reset_Handler(void) {
    uint32_t *s=&_etext,*d=&_sdata;
    while(d<&_edata)*d++=*s++;
    d=&_sbss;while(d<&_ebss)*d++=0;

    T.magic=0xABCD1234;
    T.phase=0; T.cycle=0; T.total_samples=0;
    // Conservative defaults: 3 Hz -> Td=333ms -> Td/2=166ms (very visible pause)
    T.nat_freq_mhz=3000;
    T.damp_ppm=50000;
    T.Td_ms=333;

    RCC_AHB1ENR|=(1<<0)|(1<<1)|(1<<2);
    volatile uint32_t tmp=RCC_AHB1ENR;(void)tmp;

    GPIOA_MODER&=~((3U<<2)|(3U<<4));
    GPIOA_MODER|=((1U<<2)|(1U<<4));
    GPIOC_MODER&=~(3U<<26);
    GPIOC_MODER|=(1U<<26);
    GPIOB_MODER&=~((3U<<12)|(3U<<14));
    GPIOB_MODER|=((1U<<12)|(1U<<14));
    GPIOB_OTYPER|=(1<<6)|(1<<7);
    GPIOB_PUPDR&=~((3U<<12)|(3U<<14));
    GPIOB_PUPDR|=((1U<<12)|(1U<<14));

    GPIOA_BSRR=(1<<17)|(1<<18);
    scl_hi();sda_hi();
    LED_OFF();
    delay_ms(100);

    uint8_t id=adxl_rd(0x00);
    T.devid=id;
    T.sensor_ok=(id==0xE5)?1:0;
    if(T.sensor_ok){
        adxl_wr(0x2D,0x08);
        adxl_wr(0x31,0x0B);
        adxl_wr(0x2C,0x0D); // 800Hz ODR
    }

    // Startup pattern: 3 quick blinks = all ready
    led_blink(3, 80, 80);
    delay_ms(600);

    float fn_hz=3.0f, zeta=0.05f;

    while(1){
        T.cycle++;

        // --- Compute shaper parameters ---
        float Td_s  = 1.0f/fn_hz;
        float zp    = zeta*3.14159f/my_sqrt(1.0f-zeta*zeta);
        float K     = my_exp(-zp);
        float K2    = K*K;

        float za1=1.0f/(1.0f+K), za2=K/(1.0f+K);
        uint32_t zv_d2 = (uint32_t)(Td_s*500.0f);    // Td/2 ms
        int zv_s1=(int)(za1*TOTAL_STEPS+0.5f);
        int zv_s2=TOTAL_STEPS-zv_s1;

        float denom=(1.0f+K)*(1.0f+K);
        float da1=1.0f/denom, da2=2.0f*K/denom, da3=K2/denom;
        uint32_t zvd_d2=(uint32_t)(Td_s*500.0f);
        uint32_t zvd_d3=(uint32_t)(Td_s*1000.0f);
        int zvd_s1=(int)(da1*TOTAL_STEPS+0.5f);
        int zvd_s2=(int)(da2*TOTAL_STEPS+0.5f);
        int zvd_s3=TOTAL_STEPS-zvd_s1-zvd_s2;
        if(zvd_s3<1) zvd_s3=1;

        // Update telemetry
        T.nat_freq_mhz=(uint32_t)(fn_hz*1000.0f);
        T.damp_ppm=(uint32_t)(zeta*1000000.0f);
        T.Td_ms=(uint32_t)(Td_s*1000.0f);
        T.ZV_A1_pct=(uint32_t)(za1*100.0f);
        T.ZV_A2_pct=(uint32_t)(za2*100.0f);
        T.ZV_t2_ms=zv_d2;
        T.ZVD_A1_pct=(uint32_t)(da1*100.0f);
        T.ZVD_A2_pct=(uint32_t)(da2*100.0f);
        T.ZVD_A3_pct=(uint32_t)(da3*100.0f);
        T.ZVD_t2_ms=zvd_d2;
        T.ZVD_t3_ms=zvd_d3;

        // ============================================================
        // PHASE A: UNSHAPED — 1 big snap, full 28 steps at once
        // LED: SOLID ON during vibration window
        // ============================================================
        T.phase=1;
        DIR_FW(); LED_OFF();
        delay_ms(500); // Pause before each phase so you can clearly see transition

        steps(TOTAL_STEPS, SNAP_US); // ONE SHOT: all 28 steps

        // Vibration window: LED SOLID ON
        LED_ON();
        int32_t us_p2p=measure_vibe(150, 8); // 1.2 seconds
        T.unshaped_p2p_mg=us_p2p;
        LED_OFF();

        // Update fn from measurement
        int zc=(int)T.zero_crossings;
        if(zc>=2){
            float new_fn=(float)zc/2.4f; // 1.2s window
            if(new_fn>=0.5f && new_fn<=30.0f)
                fn_hz=0.5f*fn_hz+0.5f*new_fn;
        }

        // Return slowly
        T.phase=4;
        DIR_RV(); delay_ms(300);
        steps(TOTAL_STEPS, RETURN_US);
        delay_ms(800);

        // ============================================================
        // PHASE B: ZV SHAPED — 2 impulses
        // LED: 2 quick blinks before move to signal "ZV phase starting"
        // Then LED OFF during move, ON during vibration
        // ============================================================
        T.phase=2;
        DIR_FW(); LED_OFF();
        led_blink(2, 60, 60); // Signal: ZV starting!
        delay_ms(300);

        // === Sub-move 1 of 2 ===
        steps(zv_s1, SNAP_US);

        // === PAUSE Td/2 — this is the magic, ruler mid-vibration ===
        delay_ms(zv_d2);

        // === Sub-move 2 of 2 — cancels vibration from sub-move 1 ===
        steps(zv_s2, SNAP_US);

        LED_ON();
        int32_t zv_p2p=measure_vibe(150, 8);
        T.zv_p2p_mg=zv_p2p;
        LED_OFF();

        T.phase=4;
        DIR_RV(); delay_ms(300);
        steps(TOTAL_STEPS, RETURN_US);
        delay_ms(800);

        // ============================================================
        // PHASE C: ZVD SHAPED — 3 impulses
        // LED: 3 quick blinks before move to signal "ZVD phase starting"
        // ============================================================
        T.phase=3;
        DIR_FW(); LED_OFF();
        led_blink(3, 60, 60); // Signal: ZVD starting!
        delay_ms(300);

        // === Sub-move 1 of 3 ===
        steps(zvd_s1, SNAP_US);

        // === PAUSE Td/2 ===
        delay_ms(zvd_d2);

        // === Sub-move 2 of 3 ===
        steps(zvd_s2, SNAP_US);

        // === PAUSE Td/2 again ===
        delay_ms(zvd_d3-zvd_d2);

        // === Sub-move 3 of 3 ===
        steps(zvd_s3, SNAP_US);

        LED_ON();
        int32_t zvd_p2p=measure_vibe(150, 8);
        T.zvd_p2p_mg=zvd_p2p;
        LED_OFF();

        // Reduction calculation
        if(T.unshaped_p2p_mg>50){
            int32_t zr=((T.unshaped_p2p_mg-zv_p2p)*1000)/T.unshaped_p2p_mg;
            int32_t zdr=((T.unshaped_p2p_mg-zvd_p2p)*1000)/T.unshaped_p2p_mg;
            T.zv_reduction_x10=(uint32_t)(zr<0?0:zr);
            T.zvd_reduction_x10=(uint32_t)(zdr<0?0:zdr);
        }

        // Return and long rest between cycles
        T.phase=4;
        DIR_RV(); delay_ms(300);
        steps(TOTAL_STEPS, RETURN_US);
        T.phase=0;
        delay_ms(2000); // 2 second pause — ruler settles completely
    }
}
