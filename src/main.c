/*
 * ======================================================================================
 * PROYECTO: ESP32-C3 BARE-METAL - AUDIO SPECTRUM ANALYZER
 * ======================================================================================
 * DESCRIPCIÓN:
 * Sistema embebido de procesamiento digital de señales (DSP) para visualización
 * rítmica en tiempo real. Implementado sin sistema operativo (Bare-Metal) para
 * maximizar el rendimiento del ADC y los GPIOs.
 *
 * ESPECIFICACIONES TÉCNICAS DSP:
 * --------------------------------------------------------------------------------------
 * - Frecuencia de Muestreo (Fs):  ~4000 Hz (Limitada por polling ADC en bucle)
 * - Frecuencia de Nyquist (Fmax): ~2000 Hz (Máxima frecuencia detectable)
 * - Puntos FFT (N):               64 Muestras
 * - Resolución Espectral:         ~62.5 Hz por Bin (Calculado: Fs / N)
 * - Ventana:                      Hann (Para reducción de fugas espectrales)
 *
 * MAPA DE FRECUENCIAS (CALIBRADO):
 * --------------------------------------------------------------------------------------
 * Basado en resolución de 62.5 Hz/Bin:
 * - CANAL 1 [ROJO 1]:      Bin 2  (~125 Hz)  -> Sub-Graves / Retumbe
 * - CANAL 2 [ROJO 2]:      Bin 5  (~312 Hz)  -> Graves / Golpe (Kick)
 * - CANAL 3 [AMARILLO 1]:  Bin 9  (~560 Hz)  -> Medios / Voz Cuerpo
 * - CANAL 4 [AMARILLO 2]:  Bin 13 (~812 Hz)  -> Medios-Altos / Presencia
 * - CANAL 5 [VERDE]:       Bin 19 (~1.2 kHz) -> Agudos / Definición
 *
 * INTERFAZ DE DEPURACIÓN (UART):
 * --------------------------------------------------------------------------------------
 * - Velocidad (Baud Rate): 115200 bps (Bits por segundo).
 * - Configuración:         8N1 (8 bits de datos, No paridad, 1 bit de stop).
 * - Formato de Salida:     CSV (Valores separados por coma).
 * - Estructura de Datos:   [Umbral], [Banda1], [Banda2], [Banda3], [Banda4], [Banda5]
 *
 * CONEXIONES DE HARDWARE (PINOUT):
 * --------------------------------------------------------------------------------------
 * [ENTRADAS]
 * - MICROFONO (OUT)        -> GPIO 0 (ADC1_CH0)
 * - POTENCIOMETRO          -> GPIO 1 (ADC1_CH1)
 *
 * [SALIDAS - ACTIVE HIGH]
 * - LED VERDE (Agudos)     -> GPIO 3
 * - LED AMARILLO 2         -> GPIO 4
 * - LED AMARILLO 1         -> GPIO 5
 * - LED ROJO 2             -> GPIO 6
 * - LED ROJO 1 (Graves)    -> GPIO 7
 * ======================================================================================
 */

#include <stdint.h>
#include "WDT_FEED.h"

/* --- CONFIGURACIÓN DEL SISTEMA --- */
#define MIN_THRESHOLD   750   // Umbral mínimo de señal para ignorar ruido de fondo
#define HOLD_FRAMES     16    // Ciclos de retención de estado (Anti-Flickeo)
#define INVERT_POT      1     // 1: Invertir valor del ADC (4095 -> 0)
#define LED_INVERT      0     // 0: Salida Activa Alta, 1: Salida Activa Baja
#define PRINT_PRESCALER 500   // Divisor para velocidad de salida UART

/* --- MAPA DE REGISTROS (ESP32-C3) --- */
#define REG32(addr) (*(volatile uint32_t *)(addr))
#define BIT(n)      (1U << (n))

#define DR_REG_GPIO_BASE        0x60004000UL
#define DR_REG_IO_MUX_BASE      0x60009000UL
#define DR_REG_SYSTEM_BASE      0x600C0000UL
#define DR_REG_APB_SARADC_BASE  0x60040000UL
#define DR_REG_UART_BASE        0x60000000UL

/* REGISTROS GPIO */
#define GPIO_OUT_W1TS_REG       (DR_REG_GPIO_BASE + 0x0008)
#define GPIO_OUT_W1TC_REG       (DR_REG_GPIO_BASE + 0x000C)
#define GPIO_ENABLE_W1TS_REG    (DR_REG_GPIO_BASE + 0x0024)

/* REGISTROS IO MUX */
#define IO_MUX_GPIO0_REG        (DR_REG_IO_MUX_BASE + 0x0004) 
#define IO_MUX_GPIO1_REG        (DR_REG_IO_MUX_BASE + 0x0008) 
#define IO_MUX_GPIO3_REG        (DR_REG_IO_MUX_BASE + 0x0010)
#define IO_MUX_GPIO4_REG        (DR_REG_IO_MUX_BASE + 0x0014)
#define IO_MUX_GPIO5_REG        (DR_REG_IO_MUX_BASE + 0x0018)
#define IO_MUX_GPIO6_REG        (DR_REG_IO_MUX_BASE + 0x001C)
#define IO_MUX_GPIO7_REG        (DR_REG_IO_MUX_BASE + 0x0020)

/* REGISTROS SYSTEM & ADC */
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

/* REGISTROS UART */
#define UART_FIFO_REG           (DR_REG_UART_BASE + 0x0000)
#define UART_STATUS_REG         (DR_REG_UART_BASE + 0x001C)
#define UART_TXFIFO_CNT_S       16
#define UART_TXFIFO_CNT_V       0x7FU

/* --- DEFINICIÓN DE PINES --- */
#define PIN_VERDE     3
#define PIN_AMARILLO2 4
#define PIN_AMARILLO1 5
#define PIN_ROJO2     6
#define PIN_ROJO1     7
#define LEDS_MASK     ((1U<<PIN_VERDE)|(1U<<PIN_AMARILLO2)|(1U<<PIN_AMARILLO1)|(1U<<PIN_ROJO2)|(1U<<PIN_ROJO1))

#define ADC_CH_MIC    0 
#define ADC_CH_POT    1 

/* --- CONSTANTES FFT --- */
#define N_SAMPLES 64
#define LOG2_N    6

/* Tabla de Seno precalculada para optimización */
const int8_t sin_table[N_SAMPLES] = {
    0, 12, 25, 38, 49, 60, 71, 81, 90, 98, 106, 112, 117, 122, 125, 127,
    127, 127, 125, 122, 117, 112, 106, 98, 90, 81, 71, 60, 49, 38, 25, 12,
    0, -12, -25, -38, -49, -60, -71, -81, -90, -98, -106, -112, -117, -122, -125, -127,
    -127, -127, -125, -122, -117, -112, -106, -98, -90, -81, -71, -60, -49, -38, -25, -12
};

/* Ventana de Hann para reducción de fugas espectrales */
const uint8_t hann_window[N_SAMPLES] = {
    0, 0, 1, 3, 6, 10, 15, 20, 26, 33, 41, 49, 57, 66, 75, 84,
    94, 103, 112, 121, 130, 139, 147, 155, 163, 170, 177, 184, 190, 196, 201, 205,
    209, 212, 215, 217, 218, 218, 217, 215, 212, 209, 205, 201, 196, 190, 184, 177,
    170, 163, 155, 147, 139, 130, 121, 112, 103, 94, 84, 75, 66, 57, 49, 41
};

int16_t fr[N_SAMPLES];
int16_t fi[N_SAMPLES];

/* ===================== FUNCIONES DE DRIVER ===================== */

static void uart_init(void) { } 

static void uart_write_char(char c) {
    while (1) {
        uint32_t status = REG32(UART_STATUS_REG);
        if (((status >> UART_TXFIFO_CNT_S) & UART_TXFIFO_CNT_V) < UART_TXFIFO_CNT_V) break;
    }
    REG32(UART_FIFO_REG) = (uint32_t)c;
}

static void uart_write_str(const char *s) { while (*s) uart_write_char(*s++); }

static void uart_write_uint32(uint32_t value) {
    char buf[10]; int idx = 0;
    if (value == 0) { uart_write_char('0'); return; }
    while (value > 0 && idx < 10) { buf[idx++] = '0' + (value % 10U); value /= 10U; }
    while (idx > 0) uart_write_char(buf[--idx]);
}

static void setup_gpio_output(uint32_t mux_reg, int pin) {
    uint32_t reg = REG32(mux_reg);
    reg &= ~(BIT(9)|BIT(8)|BIT(7)); 
    reg &= ~(0x7U << 12); 
    reg |= (1U << 12);
    REG32(mux_reg) = reg;
    REG32(GPIO_ENABLE_W1TS_REG) = (1U << pin);
}

static void gpio_init(void) {
    setup_gpio_output(IO_MUX_GPIO3_REG, PIN_VERDE);
    setup_gpio_output(IO_MUX_GPIO4_REG, PIN_AMARILLO2);
    setup_gpio_output(IO_MUX_GPIO5_REG, PIN_AMARILLO1);
    setup_gpio_output(IO_MUX_GPIO6_REG, PIN_ROJO2);
    setup_gpio_output(IO_MUX_GPIO7_REG, PIN_ROJO1);

    uint32_t r0 = REG32(IO_MUX_GPIO0_REG); r0 &= ~(BIT(9)|BIT(8)|BIT(7)); r0 &= ~(0x7U << 12); r0 |= (1U << 9); REG32(IO_MUX_GPIO0_REG) = r0;
    uint32_t r1 = REG32(IO_MUX_GPIO1_REG); r1 &= ~(BIT(9)|BIT(8)|BIT(7)); r1 &= ~(0x7U << 12); r1 |= (1U << 9); REG32(IO_MUX_GPIO1_REG) = r1;
}

static void adc_init(void) {
    REG32(SYSTEM_PERIP_CLK_EN0_REG) |= SYSTEM_APB_SARADC_CLK_EN;
    REG32(SYSTEM_PERIP_RST_EN0_REG) &= ~SYSTEM_APB_SARADC_RST;

    uint32_t ctrl = REG32(APB_SARADC_CTRL_REG);
    ctrl |= (1U << 6);          // Start Force
    ctrl &= ~(0x3U << 27); 
    ctrl |= (0x3U << 27);       // Mode: Single
    ctrl &= ~(0xFFU << 7); 
    ctrl |= (4U << 7);          // Clk Div
    REG32(APB_SARADC_CTRL_REG) = ctrl;

    uint32_t sample = REG32(APB_SARADC_ONETIME_SAMPLE_REG);
    sample |= (1U << 31);       // Enable ADC1
    sample |= (3U << 23);       // 12-bit
    REG32(APB_SARADC_ONETIME_SAMPLE_REG) = sample;
    
    REG32(APB_SARADC_INT_ENA_REG) |= BIT(31);
    REG32(APB_SARADC_INT_CLR_REG) = BIT(31);
}

static uint16_t adc_read(int channel) {
    uint32_t sample = REG32(APB_SARADC_ONETIME_SAMPLE_REG);
    sample &= ~(0xFU << 25); 
    sample |= ((channel & 0xF) << 25);
    REG32(APB_SARADC_ONETIME_SAMPLE_REG) = sample;
    
    sample &= ~BIT(29); REG32(APB_SARADC_ONETIME_SAMPLE_REG) = sample;
    for (volatile int i=0; i<60; ++i); 
    sample |= BIT(29); REG32(APB_SARADC_ONETIME_SAMPLE_REG) = sample;
    
    while ((REG32(APB_SARADC_INT_ST_REG) & BIT(31)) == 0);
    
    uint32_t raw = REG32(APB_SARADC_1_DATA_STATUS_REG) & 0xFFFF;
    REG32(APB_SARADC_INT_CLR_REG) = BIT(31);
    return (uint16_t)(raw & 0xFFF);
}

/* ===================== PROCESAMIENTO DIGITAL DE SEÑAL ===================== */

void compute_fft(void) {
    int16_t tr, ti;
    int i, j, k, l, m = LOG2_N;
    j = 0;
    // Bit reversal
    for (i = 0; i < N_SAMPLES - 1; i++) {
        if (i < j) { tr = fr[j]; fr[j] = fr[i]; fr[i] = tr; }
        k = N_SAMPLES >> 1;
        while (k <= j) { j -= k; k >>= 1; }
        j += k;
    }
    // Butterfly
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
    // Magnitud aproximada para velocidad
    return (r > img) ? (r + (img>>1)) : (img + (r>>1));
}

/* ===================== PROGRAMA PRINCIPAL ===================== */

int main(void) {
    disable_timg_wdt(TIMG0_BASE);
    disable_timg_wdt(TIMG1_BASE);
    disable_rtc_wdts();

    gpio_init();
    adc_init();
    uart_init(); 
    
    int print_counter = 0;
    
    // Temporizadores para mantener la salida activa (Anti-Flicker)
    int led_timers[5] = {0, 0, 0, 0, 0};

    while (1) {
        /* 1. LECTURA DE POTENCIOMETRO (Control de Sensibilidad) */
        adc_read(ADC_CH_POT); 
        int raw_val = adc_read(ADC_CH_POT);
        if (INVERT_POT) raw_val = 4095 - raw_val;

        // Limitar umbral mínimo para filtrar ruido base
        int threshold = raw_val;
        if (threshold < MIN_THRESHOLD) threshold = MIN_THRESHOLD;
        if (threshold > 4095) threshold = 4095;

        /* 2. ADQUISICIÓN DE MUESTRAS DE AUDIO */
        adc_read(ADC_CH_MIC); 
        for(int i=0; i<N_SAMPLES; i++) fr[i] = adc_read(ADC_CH_MIC);
        
        // Remover DC Offset
        int32_t block_avg = 0;
        for(int i=0; i<N_SAMPLES; i++) block_avg += fr[i];
        block_avg /= N_SAMPLES;
        
        // Aplicar Ventana
        for(int i=0; i<N_SAMPLES; i++) {
            int32_t val = fr[i] - block_avg; fi[i] = 0; 
            fr[i] = (int16_t)((val * hann_window[i]) >> 8);
        }

        /* 3. TRANSFORMADA RÁPIDA DE FOURIER */
        compute_fft();

        /* 4. SELECCIÓN DE BANDAS Y ECUALIZACIÓN */
        #define NOISE_GATE 8
        int vals[5];
        
        // Selección de Bins (Francotirador)
        vals[0] = magnitude(2);   // Banda 1: Graves
        vals[1] = magnitude(5);   // Banda 2: Medios-Bajos
        vals[2] = magnitude(9);   // Banda 3: Medios
        vals[3] = magnitude(13);  // Banda 4: Medios-Altos
        vals[4] = magnitude(19);  // Banda 5: Agudos

        // Ecualización de ganancia para compensar respuesta del micrófono
        if(vals[0] < NOISE_GATE) vals[0] = 0; else vals[0] = (vals[0] * 20) / 10;
        if(vals[1] < NOISE_GATE) vals[1] = 0; else vals[1] = (vals[1] * 35) / 10;
        if(vals[2] < NOISE_GATE) vals[2] = 0; else vals[2] = (vals[2] * 40) / 10;
        if(vals[3] < NOISE_GATE) vals[3] = 0; else vals[3] = (vals[3] * 50) / 10;
        if(vals[4] < NOISE_GATE) vals[4] = 0; else vals[4] = (vals[4] * 60) / 10;

        /* 5. DEBUGGING (Salida Serial) */
        print_counter++;
        if (print_counter > PRINT_PRESCALER) {  
            uart_write_uint32(threshold); uart_write_char(',');
            uart_write_uint32(vals[0]);   uart_write_char(',');
            uart_write_uint32(vals[1]);   uart_write_char(',');
            uart_write_uint32(vals[2]);   uart_write_char(',');
            uart_write_uint32(vals[3]);   uart_write_char(',');
            uart_write_uint32(vals[4]);    
            uart_write_str("\r\n");
            print_counter = 0;
        }

        /* 6. CONTROL DE SALIDAS GPIO (Lógica Hold) */
        uint32_t set_mask = 0;

        for (int i = 0; i < 5; i++) {
            // Si supera el umbral, recargar temporizador (Encender)
            if (vals[i] > threshold) {
                led_timers[i] = HOLD_FRAMES;
            } 
            // Si no, descontar temporizador (Mantener o Apagar)
            else if (led_timers[i] > 0) {
                led_timers[i]--;
            }
        }

        // Generar máscara de salida basada en temporizadores activos
        if (led_timers[0] > 0) set_mask |= (1U << PIN_ROJO1);
        if (led_timers[1] > 0) set_mask |= (1U << PIN_ROJO2);
        if (led_timers[2] > 0) set_mask |= (1U << PIN_AMARILLO1);
        if (led_timers[3] > 0) set_mask |= (1U << PIN_AMARILLO2);
        if (led_timers[4] > 0) set_mask |= (1U << PIN_VERDE);

        // Escritura a pines físicos
        if (LED_INVERT) {
            REG32(GPIO_OUT_W1TC_REG) = set_mask;          
            REG32(GPIO_OUT_W1TS_REG) = (~set_mask) & LEDS_MASK; 
        } else {
            REG32(GPIO_OUT_W1TS_REG) = set_mask;          
            REG32(GPIO_OUT_W1TC_REG) = (~set_mask) & LEDS_MASK; 
        }
    }
}