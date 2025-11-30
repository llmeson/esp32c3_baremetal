#include <stdint.h>
#include "WDT_FEED.h"

/* --- 1. DEFINICIÓN DE REGISTROS --- */
#define BIT(n) (1U << (n))
#define REG32(addr) (*(volatile uint32_t *)(addr))

#define DR_REG_GPIO_BASE        0x60004000UL
#define DR_REG_IO_MUX_BASE      0x60009000UL
#define DR_REG_SYSTEM_BASE      0x600C0000UL
#define DR_REG_APB_SARADC_BASE  0x60040000UL

// GPIO
#define GPIO_OUT_W1TS_REG       (DR_REG_GPIO_BASE + 0x0008)
#define GPIO_OUT_W1TC_REG       (DR_REG_GPIO_BASE + 0x000C)
#define GPIO_ENABLE_W1TS_REG    (DR_REG_GPIO_BASE + 0x0024)
#define GPIO_ENABLE_W1TC_REG    (DR_REG_GPIO_BASE + 0x0028)

// IO MUX
#define IO_MUX_GPIO0_REG        (DR_REG_IO_MUX_BASE + 0x0004)
#define IO_MUX_GPIO3_REG        (DR_REG_IO_MUX_BASE + 0x0010)
#define IO_MUX_GPIO4_REG        (DR_REG_IO_MUX_BASE + 0x0014)
#define IO_MUX_GPIO5_REG        (DR_REG_IO_MUX_BASE + 0x0018)
#define IO_MUX_GPIO6_REG        (DR_REG_IO_MUX_BASE + 0x001C)
#define IO_MUX_GPIO7_REG        (DR_REG_IO_MUX_BASE + 0x0020)

// SYSTEM
#define SYSTEM_PERIP_CLK_EN0_REG (DR_REG_SYSTEM_BASE + 0x0010)
#define SYSTEM_PERIP_RST_EN0_REG (DR_REG_SYSTEM_BASE + 0x0018)
#define SYSTEM_APB_SARADC_CLK_EN BIT(28)
#define SYSTEM_APB_SARADC_RST    BIT(28)

// ADC
#define APB_SARADC_CTRL_REG            (DR_REG_APB_SARADC_BASE + 0x0000)
#define APB_SARADC_ONETIME_SAMPLE_REG  (DR_REG_APB_SARADC_BASE + 0x0020)
#define APB_SARADC_1_DATA_STATUS_REG   (DR_REG_APB_SARADC_BASE + 0x002C)
#define APB_SARADC_INT_ENA_REG         (DR_REG_APB_SARADC_BASE + 0x0040)
#define APB_SARADC_INT_CLR_REG         (DR_REG_APB_SARADC_BASE + 0x004C)
#define APB_SARADC_INT_ST_REG          (DR_REG_APB_SARADC_BASE + 0x0048)

/* --- PINES --- */
#define PIN_VERDE     3  // Agudos (Tope)
#define PIN_AMARILLO2 4  // Snare
#define PIN_AMARILLO1 5  // Medios
#define PIN_ROJO2     6  // Golpe
#define PIN_ROJO1     7  // Sub (Base)
#define PIN_POT       0

// Resolución PWM (Software)
#define PWM_STEPS 50 

/* --- VARIABLES GLOBALES DE BRILLO (0 a PWM_STEPS) --- */
static volatile uint8_t br_rojo1 = 0;
static volatile uint8_t br_rojo2 = 0;
static volatile uint8_t br_amarillo1 = 0;
static volatile uint8_t br_amarillo2 = 0;
static volatile uint8_t br_verde = 0;

/* --- FUNCIONES --- */

static void setup_gpio_pin(uint32_t mux_reg, int pin) {
    uint32_t reg = REG32(mux_reg);
    reg &= ~(BIT(9)|BIT(8)|BIT(7)); // Limpiar pulls
    reg &= ~(0x7U << 12);           // Limpiar funcion
    reg |= (1U << 12);              // Setear Func GPIO
    REG32(mux_reg) = reg;
    REG32(GPIO_ENABLE_W1TS_REG) = (1U << pin); // Enable Output
}

static void gpio_init(void) {
    setup_gpio_pin(IO_MUX_GPIO3_REG, PIN_VERDE);
    setup_gpio_pin(IO_MUX_GPIO4_REG, PIN_AMARILLO2);
    setup_gpio_pin(IO_MUX_GPIO5_REG, PIN_AMARILLO1);
    setup_gpio_pin(IO_MUX_GPIO6_REG, PIN_ROJO2);
    setup_gpio_pin(IO_MUX_GPIO7_REG, PIN_ROJO1);

    // Potenciometro Input
    uint32_t reg = REG32(IO_MUX_GPIO0_REG);
    reg &= ~(BIT(9)|BIT(8)|BIT(7)); reg |= (1U << 12);
    REG32(IO_MUX_GPIO0_REG) = reg;
    REG32(GPIO_ENABLE_W1TC_REG) = BIT(0);
}

static void adc_init(void) {
    REG32(SYSTEM_PERIP_CLK_EN0_REG) |= SYSTEM_APB_SARADC_CLK_EN;
    REG32(SYSTEM_PERIP_RST_EN0_REG) |= SYSTEM_APB_SARADC_RST;
    REG32(SYSTEM_PERIP_RST_EN0_REG) &= ~SYSTEM_APB_SARADC_RST;

    uint32_t ctrl = REG32(APB_SARADC_CTRL_REG);
    ctrl |= (1U << 6); // CLK Gated
    ctrl &= ~(0x3U << 27); ctrl |= (0x3U << 27); // Force On
    ctrl &= ~(0xFFU << 7); ctrl |= (4U << 7);    // Div 4
    REG32(APB_SARADC_CTRL_REG) = ctrl;

    uint32_t sample = REG32(APB_SARADC_ONETIME_SAMPLE_REG);
    sample |= (1U << 31); // Enable
    sample &= ~(0xFU << 25); // Ch0
    sample |= (3U << 23); // 11dB
    REG32(APB_SARADC_ONETIME_SAMPLE_REG) = sample;

    REG32(APB_SARADC_INT_ENA_REG) |= BIT(31);
    REG32(APB_SARADC_INT_CLR_REG) = BIT(31);
}

static uint16_t adc_sample_once(void) {
    uint32_t sample = REG32(APB_SARADC_ONETIME_SAMPLE_REG);
    sample &= ~BIT(29); REG32(APB_SARADC_ONETIME_SAMPLE_REG) = sample;
    for (volatile int i=0; i<32; ++i);
    sample |= BIT(29); REG32(APB_SARADC_ONETIME_SAMPLE_REG) = sample;

    while ((REG32(APB_SARADC_INT_ST_REG) & BIT(31)) == 0);

    uint32_t raw = REG32(APB_SARADC_1_DATA_STATUS_REG) & 0xFFFF;
    REG32(APB_SARADC_INT_CLR_REG) = BIT(31);
    return (uint16_t)(raw & 0xFFF);
}

/*
 * Función: run_pwm_frame
 * Ejecuta un ciclo completo de PWM por software.
 */
static void run_pwm_frame(int duration_loops) {
    for (int loop = 0; loop < duration_loops; loop++) {
        // Un ciclo de PWM (0 a 50)
        for (int tick = 0; tick < PWM_STEPS; tick++) {
            uint32_t set_mask = 0;
            uint32_t clear_mask = 0;

            if (br_rojo1 > tick) set_mask |= (1U << PIN_ROJO1); else clear_mask |= (1U << PIN_ROJO1);
            if (br_rojo2 > tick) set_mask |= (1U << PIN_ROJO2); else clear_mask |= (1U << PIN_ROJO2);
            if (br_amarillo1 > tick) set_mask |= (1U << PIN_AMARILLO1); else clear_mask |= (1U << PIN_AMARILLO1);
            if (br_amarillo2 > tick) set_mask |= (1U << PIN_AMARILLO2); else clear_mask |= (1U << PIN_AMARILLO2);
            if (br_verde > tick) set_mask |= (1U << PIN_VERDE); else clear_mask |= (1U << PIN_VERDE);

            if (set_mask) REG32(GPIO_OUT_W1TS_REG) = set_mask;
            if (clear_mask) REG32(GPIO_OUT_W1TC_REG) = clear_mask;
            
            for(volatile int d=0; d<20; d++); 
        }
    }
}

static void startup_animation(void) {
    // Fade IN
    for(int b=0; b<=PWM_STEPS; b++) {
        br_rojo1 = br_rojo2 = br_amarillo1 = br_amarillo2 = br_verde = b;
        run_pwm_frame(10);
    }
    // Fade OUT
    for(int b=PWM_STEPS; b>=0; b--) {
        br_rojo1 = br_rojo2 = br_amarillo1 = br_amarillo2 = br_verde = b;
        run_pwm_frame(10);
    }
    br_rojo1 = br_rojo2 = br_amarillo1 = br_amarillo2 = br_verde = 0;
    run_pwm_frame(10);
}

// Función auxiliar para calcular brillo en cascada
// input: valor ADC, offset: donde empieza a brillar
int calc_brightness(int32_t input, int32_t offset) {
    int32_t val = input - offset;
    if (val < 0) return 0;
    
    // Escalamos 500 puntos de ADC a 50 pasos de PWM
    // (val * 50) / 500  => val / 10
    int32_t b = val / 10;
    
    if (b > PWM_STEPS) return PWM_STEPS;
    return b;
}

int main(void) {
    disable_timg_wdt(TIMG0_BASE);
    disable_timg_wdt(TIMG1_BASE);
    disable_rtc_wdts();

    gpio_init();
    adc_init();

    startup_animation();

    while (1) {
        // 1. LEER ADC
        uint16_t raw_val = adc_sample_once();

        // 2. CALCULAR CASCADA (RE-CALIBRADO)
        // Distribución pareja de 500 puntos por LED
        // El verde empieza en 3600 para tener espacio de llegar al maximo (4095)
        
        int32_t val = (int32_t)raw_val;

        br_rojo1     = calc_brightness(val, 1600); // Base
        br_rojo2     = calc_brightness(val, 2100); 
        br_amarillo1 = calc_brightness(val, 2600); 
        br_amarillo2 = calc_brightness(val, 3100); 
        br_verde     = calc_brightness(val, 3600); // Tope

        // 3. DIBUJAR CUADRO
        // Ejecutamos PWM ~20ms para que se vea estable
        run_pwm_frame(100); 
    }
}