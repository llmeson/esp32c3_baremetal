/*
 * ======================================================================================
 * PROYECTO: ESP32-C3 BARE-METAL AUDIO VISUALIZER (GOLDEN MASTER)
 * ======================================================================================
 * * [ MAPA DE CONEXIONES DE HARDWARE ]
 * * ESP32-C3         PERIFÉRICO             FUNCIÓN
 * -----------------------------------------------------------
 * GPIO 0     <---  Micrófono (MAX4466)    Entrada Audio (OUT)
 * GPIO 1     <---  Potenciómetro          Ajuste Umbral (Wiper)
 * * GPIO 7     --->  Triac / LED Rojo 1     BOMBO (Golpe)
 * GPIO 6     --->  Triac / LED Rojo 2     BAJOS (Cuerpo)
 * GPIO 5     --->  Triac / LED Amar 1     MEDIOS (Voces)
 * GPIO 4     --->  Triac / LED Amar 2     SNARE (Ritmo)
 * GPIO 3     --->  Triac / LED Verde      AGUDOS (Platillos)
 * ======================================================================================
 */

#include <stdint.h>
#include "WDT_FEED.h"

/* --- CONFIGURACIÓN DE HARDWARE --- */
// 0 = MODO LED (PWM Rápido, 0-100%)
// 1 = MODO TRIAC 220V (PWM Lento para AC, Pre-heat 30-100%)
#define MODO_LAMPARA  0 

/* --- REGISTROS --- */
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

// IO MUX
#define IO_MUX_GPIO0_REG        (DR_REG_IO_MUX_BASE + 0x0004) 
#define IO_MUX_GPIO1_REG        (DR_REG_IO_MUX_BASE + 0x0008) 
#define IO_MUX_GPIO3_REG        (DR_REG_IO_MUX_BASE + 0x0010)
#define IO_MUX_GPIO4_REG        (DR_REG_IO_MUX_BASE + 0x0014)
#define IO_MUX_GPIO5_REG        (DR_REG_IO_MUX_BASE + 0x0018)
#define IO_MUX_GPIO6_REG        (DR_REG_IO_MUX_BASE + 0x001C)
#define IO_MUX_GPIO7_REG        (DR_REG_IO_MUX_BASE + 0x0020)

// SYSTEM & ADC
#define SYSTEM_PERIP_CLK_EN0_REG (DR_REG_SYSTEM_BASE + 0x0010)
#define SYSTEM_PERIP_RST_EN0_REG (DR_REG_SYSTEM_BASE + 0x0018)
#define SYSTEM_APB_SARADC_CLK_EN BIT(28)
#define SYSTEM_APB_SARADC_RST    BIT(28)

#define APB_SARADC_CTRL_REG            (DR_REG_APB_SARADC_BASE + 0x0000)
#define APB_SARADC_ONETIME_SAMPLE_REG  (DR_REG_APB_SARADC_BASE + 0x0020)
#define APB_SARADC_1_DATA_STATUS_REG   (DR_REG_APB_SARADC_BASE + 0x002C)
#define APB_SARADC_INT_ENA_REG         (DR_REG_APB_SARADC_BASE + 0x0040)
#define APB_SARADC_INT_CLR_REG         (DR_REG_APB_SARADC_BASE + 0x004C)
#define APB_SARADC_INT_ST_REG          (DR_REG_APB_SARADC_BASE + 0x0048)

/* --- PINES --- */
#define PIN_VERDE     3
#define PIN_AMARILLO2 4
#define PIN_AMARILLO1 5
#define PIN_ROJO2     6
#define PIN_ROJO1     7

// Máscara combinada para eficiencia
#define LEDS_MASK     ((1U<<PIN_VERDE)|(1U<<PIN_AMARILLO2)|(1U<<PIN_AMARILLO1)|(1U<<PIN_ROJO2)|(1U<<PIN_ROJO1))

#define ADC_CH_MIC    0 
#define ADC_CH_POT    1 
#define PWM_STEPS     50

/* --- FFT CONSTANTES (64 Puntos) --- */
#define N_SAMPLES 64
#define LOG2_N    6

/* Tabla Seno */
const int8_t sin_table[N_SAMPLES] = {
    0, 12, 25, 38, 49, 60, 71, 81, 90, 98, 106, 112, 117, 122, 125, 127,
    127, 127, 125, 122, 117, 112, 106, 98, 90, 81, 71, 60, 49, 38, 25, 12,
    0, -12, -25, -38, -49, -60, -71, -81, -90, -98, -106, -112, -117, -122, -125, -127,
    -127, -127, -125, -122, -117, -112, -106, -98, -90, -81, -71, -60, -49, -38, -25, -12
};

/* Tabla Hann */
const uint8_t hann_window[N_SAMPLES] = {
    0, 0, 1, 3, 6, 10, 15, 20, 26, 33, 41, 49, 57, 66, 75, 84,
    94, 103, 112, 121, 130, 139, 147, 155, 163, 170, 177, 184, 190, 196, 201, 205,
    209, 212, 215, 217, 218, 218, 217, 215, 212, 209, 205, 201, 196, 190, 184, 177,
    170, 163, 155, 147, 139, 130, 121, 112, 103, 94, 84, 75, 66, 57, 49, 41
};

int16_t fr[N_SAMPLES];
int16_t fi[N_SAMPLES];

/* --- VISUAL VARIABLES --- */
static uint8_t curr_rojo1 = 0;
static uint8_t curr_rojo2 = 0;
static uint8_t curr_amarillo1 = 0;
static uint8_t curr_amarillo2 = 0;
static uint8_t curr_verde = 0;

/* --- HARDWARE INIT --- */
static void setup_gpio_output(uint32_t mux_reg, int pin) {
    uint32_t reg = REG32(mux_reg);
    reg &= ~(BIT(9)|BIT(8)|BIT(7)); reg &= ~(0x7U << 12); reg |= (1U << 12);
    REG32(mux_reg) = reg;
    REG32(GPIO_ENABLE_W1TS_REG) = (1U << pin);
}

static void gpio_init(void) {
    setup_gpio_output(IO_MUX_GPIO3_REG, PIN_VERDE);
    setup_gpio_output(IO_MUX_GPIO4_REG, PIN_AMARILLO2);
    setup_gpio_output(IO_MUX_GPIO5_REG, PIN_AMARILLO1);
    setup_gpio_output(IO_MUX_GPIO6_REG, PIN_ROJO2);
    setup_gpio_output(IO_MUX_GPIO7_REG, PIN_ROJO1);

    uint32_t r0 = REG32(IO_MUX_GPIO0_REG);
    r0 &= ~(BIT(9)|BIT(8)|BIT(7)); r0 &= ~(0x7U << 12); r0 |= (1U << 9);
    REG32(IO_MUX_GPIO0_REG) = r0;
    
    uint32_t r1 = REG32(IO_MUX_GPIO1_REG);
    r1 &= ~(BIT(9)|BIT(8)|BIT(7)); r1 &= ~(0x7U << 12); r1 |= (1U << 9);
    REG32(IO_MUX_GPIO1_REG) = r1;
}

static void adc_init(void) {
    REG32(SYSTEM_PERIP_CLK_EN0_REG) |= SYSTEM_APB_SARADC_CLK_EN;
    REG32(SYSTEM_PERIP_RST_EN0_REG) &= ~SYSTEM_APB_SARADC_RST;

    uint32_t ctrl = REG32(APB_SARADC_CTRL_REG);
    ctrl |= (1U << 6); ctrl &= ~(0x3U << 27); ctrl |= (0x3U << 27); 
    ctrl &= ~(0xFFU << 7); ctrl |= (4U << 7);    
    REG32(APB_SARADC_CTRL_REG) = ctrl;

    uint32_t sample = REG32(APB_SARADC_ONETIME_SAMPLE_REG);
    sample |= (1U << 31); sample |= (3U << 23); 
    REG32(APB_SARADC_ONETIME_SAMPLE_REG) = sample;
    
    REG32(APB_SARADC_INT_ENA_REG) |= BIT(31);
    REG32(APB_SARADC_INT_CLR_REG) = BIT(31);
}

static uint16_t adc_read(int channel) {
    uint32_t sample = REG32(APB_SARADC_ONETIME_SAMPLE_REG);
    sample &= ~(0xFU << 25); sample |= ((channel & 0xF) << 25);
    REG32(APB_SARADC_ONETIME_SAMPLE_REG) = sample;
    
    sample &= ~BIT(29); REG32(APB_SARADC_ONETIME_SAMPLE_REG) = sample;
    for (volatile int i=0; i<60; ++i); 
    sample |= BIT(29); REG32(APB_SARADC_ONETIME_SAMPLE_REG) = sample;

    while ((REG32(APB_SARADC_INT_ST_REG) & BIT(31)) == 0);
    uint32_t raw = REG32(APB_SARADC_1_DATA_STATUS_REG) & 0xFFFF;
    REG32(APB_SARADC_INT_CLR_REG) = BIT(31);
    return (uint16_t)(raw & 0xFFF);
}

/* --- FFT ENGINE --- */
void compute_fft(void) {
    int16_t tr, ti;
    int i, j, k, l, m = LOG2_N;
    j = 0;
    for (i = 0; i < N_SAMPLES - 1; i++) {
        if (i < j) { tr = fr[j]; fr[j] = fr[i]; fr[i] = tr; }
        k = N_SAMPLES >> 1;
        while (k <= j) { j -= k; k >>= 1; }
        j += k;
    }
    int step = 1;
    for (l = 0; l < m; l++) {
        int jump = step << 1;
        for (k = 0; k < step; k++) {
            int idx = (k * N_SAMPLES / jump);
            int cos_v = sin_table[(idx + (N_SAMPLES/4)) % N_SAMPLES];
            int sin_v = sin_table[idx];
            for (i = k; i < N_SAMPLES; i += jump) {
                j = i + step;
                tr = (fi[j] * sin_v + fr[j] * cos_v) >> 7;
                ti = (fi[j] * cos_v - fr[j] * sin_v) >> 7;
                fr[j] = fr[i] - tr; fi[j] = fi[i] - ti;
                fr[i] += tr; fi[i] += ti;
            }
        }
        step = jump;
    }
}

int16_t magnitude(int i) {
    int16_t r = fr[i] < 0 ? -fr[i] : fr[i];
    int16_t img = fi[i] < 0 ? -fi[i] : fi[i];
    return (r > img) ? (r + (img>>1)) : (img + (r>>1));
}

// UPDATE CON PRE-HEAT (30%) AUTOMÁTICO
void update_smooth_led(uint8_t *curr, int target) {
    int min_val = 0;
    
    // Si estamos en MODO_LAMPARA, el piso sube a 15 (Pre-Heat)
    if (MODO_LAMPARA) {
        min_val = 15;
        // Escalamos el rango musical para que opere entre 15 y 50
        if (target > 0) {
            int range = PWM_STEPS - min_val;
            target = min_val + ((target * range) / PWM_STEPS);
        } else {
            target = min_val;
        }
    }

    if (target > PWM_STEPS) target = PWM_STEPS;
    
    // Smoothing
    if (target > *curr) {
        *curr = target; // Ataque rápido
    } else if (*curr > min_val) {
        *curr -= 1;     // Decay lento
    }
}

static void run_pwm_frame(void) {
    // AJUSTE DE VELOCIDAD AUTOMÁTICO
    // - LEDs: Delay corto (25) -> Alta frecuencia, sin parpadeo.
    // - TRIACS: Delay largo (120) -> Baja frecuencia, permite que el Triac dispare.
    int delay_ticks = (MODO_LAMPARA) ? 120 : 25; 

    for (int t = 0; t < PWM_STEPS; t++) {
        uint32_t s = 0;
        if (curr_rojo1 > t) s |= (1U<<PIN_ROJO1);
        if (curr_rojo2 > t) s |= (1U<<PIN_ROJO2);
        if (curr_amarillo1 > t) s |= (1U<<PIN_AMARILLO1);
        if (curr_amarillo2 > t) s |= (1U<<PIN_AMARILLO2);
        if (curr_verde > t) s |= (1U<<PIN_VERDE);
        
        REG32(GPIO_OUT_W1TS_REG) = s;
        REG32(GPIO_OUT_W1TC_REG) = (~s) & LEDS_MASK;
        
        // Usamos la variable delay_ticks para controlar la velocidad
        for(volatile int d=0; d<delay_ticks; d++);
    }
}

int apply_manual_threshold(int val, int manual_threshold) {
    if (val < manual_threshold) return 0;
    int excess = val - manual_threshold;
    int res = excess / 3; 
    if (res > PWM_STEPS) return PWM_STEPS;
    return res;
}

int main(void) {
    disable_timg_wdt(TIMG0_BASE);
    disable_timg_wdt(TIMG1_BASE);
    disable_rtc_wdts();

    gpio_init();
    adc_init();

    while (1) {
        // LEER UMBRAL
        int threshold = adc_read(ADC_CH_POT);
        if (threshold > 4000) threshold = 4095;

        // LEER MIC
        int32_t block_avg = 0;
        for(int i=0; i<N_SAMPLES; i++) {
            int val = adc_read(ADC_CH_MIC);
            fr[i] = val;
            block_avg += val;
            fi[i] = 0;
        }
        block_avg /= N_SAMPLES;
        
        for(int i=0; i<N_SAMPLES; i++) {
            int32_t val = fr[i] - block_avg;
            fr[i] = (int16_t)((val * hann_window[i]) >> 8);
        }

        compute_fft();

        /* --- CROSSOVER TUNED --- */
        #define NOISE_GATE 4 
        
        int b1 = magnitude(4) + magnitude(5); 
        if(b1 < NOISE_GATE) b1 = 0; else b1 = (b1 * 12) / 10; // Bombo

        int b2 = magnitude(6) + magnitude(7);
        if(b2 < NOISE_GATE) b2 = 0; else b2 = (b2 * 18) / 10; // Bajos

        int b3 = magnitude(9) + magnitude(11) + magnitude(13);
        if(b3 < NOISE_GATE) b3 = 0; else b3 = (b3 * 14) / 10; // Medios

        int b4 = magnitude(16) + magnitude(19); 
        if(b4 < NOISE_GATE) b4 = 0; else b4 = (b4 * 28) / 10; // Snare

        int b5 = magnitude(23) + magnitude(28); 
        if(b5 < NOISE_GATE) b5 = 0; else b5 = (b5 * 20) / 10; // Agudos

        update_smooth_led(&curr_rojo1,     apply_manual_threshold(b1, threshold));
        update_smooth_led(&curr_rojo2,     apply_manual_threshold(b2, threshold));
        update_smooth_led(&curr_amarillo1, apply_manual_threshold(b3, threshold));
        update_smooth_led(&curr_amarillo2, apply_manual_threshold(b4, threshold));
        update_smooth_led(&curr_verde,     apply_manual_threshold(b5, threshold));

        run_pwm_frame();
        run_pwm_frame();
    }
}