#include <stdint.h>
#include "WDT_FEED.h"

#define BIT(n) (1U << (n))                    // Máscara de un bit
#define REG32(addr) (*(volatile uint32_t *)(addr)) // Acceso directo a registro de 32 bits

#define DR_REG_GPIO_BASE        0x60004000UL  // Base periférico GPIO
#define DR_REG_IO_MUX_BASE      0x60009000UL  // Base IO_MUX (selección de función/pulls)
#define DR_REG_SYSTEM_BASE      0x600C0000UL  // Base registro de sistema (clocks/resets)
#define DR_REG_APB_SARADC_BASE  0x60040000UL  // Base ADC SAR digital
#define DR_REG_LEDC_BASE        0x60019000UL  // Base bloque LEDC (PWM hardware)

#define GPIO_OUT_W1TS_REG   (DR_REG_GPIO_BASE + 0x0008)  // Set pin high (write-1-to-set)
#define GPIO_OUT_W1TC_REG   (DR_REG_GPIO_BASE + 0x000C)  // Set pin low  (write-1-to-clear)
#define GPIO_ENABLE_W1TS_REG (DR_REG_GPIO_BASE + 0x0024) // Habilitar OE
#define GPIO_ENABLE_W1TC_REG (DR_REG_GPIO_BASE + 0x0028) // Deshabilitar OE

#define IO_MUX_GPIO0_REG    (DR_REG_IO_MUX_BASE + 0x0004) // IO_MUX para GPIO0 (ADC)
#define IO_MUX_GPIO3_REG    (DR_REG_IO_MUX_BASE + 0x0010) // IO_MUX para GPIO3 (LED)
#define IO_MUX_FUN_IE       BIT(9)   // Input enable digital
#define IO_MUX_FUN_PU       BIT(8)   // Pull-up digital
#define IO_MUX_FUN_PD       BIT(7)   // Pull-down digital
#define IO_MUX_MCU_SEL_MASK (0x7U << 12) // Selector de función
#define IO_MUX_MCU_SEL_GPIO 1U       // Función GPIO

#define SYSTEM_PERIP_CLK_EN0_REG (DR_REG_SYSTEM_BASE + 0x0010) // Registro de clocks
#define SYSTEM_PERIP_RST_EN0_REG (DR_REG_SYSTEM_BASE + 0x0018) // Registro de resets
#define SYSTEM_APB_SARADC_CLK_EN BIT(28) // Bit de clock para ADC SAR
#define SYSTEM_APB_SARADC_RST    BIT(28) // Bit de reset para ADC SAR
#define SYSTEM_LEDC_CLK_EN       BIT(11) // Bit de clock para LEDC
#define SYSTEM_LEDC_RST          BIT(11) // Bit de reset para LEDC

#define APB_SARADC_CTRL_REG            (DR_REG_APB_SARADC_BASE + 0x0000) // Control general ADC
#define APB_SARADC_START_FORCE         BIT(0)  // Forzar arranque digital
#define APB_SARADC_START               BIT(1)  // Señal start SW
#define APB_SARADC_SAR_CLK_GATED       BIT(6)  // Clock gated para SAR
#define APB_SARADC_SAR_CLK_DIV_S       7       // Shift divisor clock
#define APB_SARADC_SAR_CLK_DIV_M       (0xFFU << APB_SARADC_SAR_CLK_DIV_S)
#define APB_SARADC_XPD_SAR_FORCE_S     27      // Shift modo power
#define APB_SARADC_XPD_SAR_FORCE_M     (0x3U << APB_SARADC_XPD_SAR_FORCE_S)

#define APB_SARADC_ONETIME_SAMPLE_REG  (DR_REG_APB_SARADC_BASE + 0x0020) // Control oneshot
#define APB_SARADC1_ONETIME_SAMPLE     BIT(31) // Selecciona ADC1
#define APB_SARADC_ONETIME_START       BIT(29) // Lanzar conversión
#define APB_SARADC_ONETIME_CHANNEL_S   25      // Shift canal
#define APB_SARADC_ONETIME_CHANNEL_M   (0xFU << APB_SARADC_ONETIME_CHANNEL_S)
#define APB_SARADC_ONETIME_ATTEN_S     23      // Shift atenuación
#define APB_SARADC_ONETIME_ATTEN_M     (0x3U << APB_SARADC_ONETIME_ATTEN_S)

#define APB_SARADC_1_DATA_STATUS_REG   (DR_REG_APB_SARADC_BASE + 0x002C) // Resultado ADC1

#define APB_SARADC_INT_ENA_REG         (DR_REG_APB_SARADC_BASE + 0x0040) // Enable de flags
#define APB_SARADC_ADC1_DONE_INT_ENA   BIT(31) // Habilita flag ADC1 done
#define APB_SARADC_INT_ST_REG          (DR_REG_APB_SARADC_BASE + 0x0048) // Estado de flags
#define APB_SARADC_ADC1_DONE_INT_ST    BIT(31) // Flag ADC1 conversión terminada
#define APB_SARADC_INT_CLR_REG         (DR_REG_APB_SARADC_BASE + 0x004C) // Clear de flags
#define APB_SARADC_ADC1_DONE_INT_CLR   BIT(31) // Limpia flag done

#define LEDC_LSTIMER0_CONF_REG   (DR_REG_LEDC_BASE + 0x00A0)
#define LEDC_LSTIMER0_PARA_UP    BIT(25)
#define LEDC_LSTIMER0_RST        BIT(23)
#define LEDC_LSTIMER0_PAUSE      BIT(22)
#define LEDC_CLK_DIV_LSTIMER0_M  ((0x0003FFFFU) << 4)
#define LEDC_CLK_DIV_LSTIMER0_S  4
#define LEDC_LSTIMER0_DUTY_RES_M ((0xFU) << 0)
#define LEDC_LSTIMER0_DUTY_RES_S 0

#define LEDC_CONF_REG            (DR_REG_LEDC_BASE + 0x00D0)
#define LEDC_CLK_EN              BIT(31)
#define LEDC_APB_CLK_SEL_M       ((0x3U) << 0)
#define LEDC_APB_CLK_SEL_S       0
#define LEDC_APB_CLK_SEL_APB     1U

#define LEDC_LSCH0_CONF0_REG     (DR_REG_LEDC_BASE + 0x0000)
#define LEDC_PARA_UP_LSCH0       BIT(4)
#define LEDC_IDLE_LV_LSCH0       BIT(3)
#define LEDC_SIG_OUT_EN_LSCH0    BIT(2)
#define LEDC_TIMER_SEL_LSCH0_M   ((0x3U) << 0)
#define LEDC_TIMER_SEL_LSCH0_S   0

#define LEDC_LSCH0_HPOINT_REG    (DR_REG_LEDC_BASE + 0x0004)
#define LEDC_LSCH0_DUTY_REG      (DR_REG_LEDC_BASE + 0x0008)

#define LEDC_LSCH0_CONF1_REG     (DR_REG_LEDC_BASE + 0x000C)
#define LEDC_DUTY_START_LSCH0    BIT(31)

#define GPIO_FUNC3_OUT_SEL_CFG_REG (DR_REG_GPIO_BASE + 0x0560)
#define GPIO_FUNC3_OEN_INV_SEL      BIT(10)
#define GPIO_FUNC3_OEN_SEL          BIT(9)
#define GPIO_FUNC3_OUT_INV_SEL      BIT(8)
#define GPIO_FUNC3_OUT_SEL_M        ((0xFFU) << 0)
#define GPIO_FUNC3_OUT_SEL_S        0

#define LEDC_LS_SIG_OUT0_IDX    45U  // Señal PWM canal 0 (low-speed)

#define LED_GPIO        3U
#define LED2_GPIO       5U
#define POT_GPIO        0U
#define LED_MASK        BIT(LED_GPIO)
#define LED2_MASK       BIT(LED2_GPIO)
#define POT_MASK        BIT(POT_GPIO)

#define ADC_ATTEN_11DB  3U
#define ADC_THRESHOLD   2000U
#define LOOP_DELAY      5000U

#define ADC_ZERO_BIAS   1650U   // Cuentas residuales con cursor a GND (ajustar según hardware)

#define LEDC_PWM_FREQ_HZ       2000ULL
#define LEDC_TIMER_RES_BITS    10U
#define LEDC_TIMER_SOURCE_HZ   80000000ULL
#define LEDC_CLK_DIV_FRAC_BITS 8U
#define LEDC_TIMER_DIVIDER_NUM (LEDC_TIMER_SOURCE_HZ << LEDC_CLK_DIV_FRAC_BITS)
#define LEDC_TIMER_DIVIDER_DEN (LEDC_PWM_FREQ_HZ * (1ULL << LEDC_TIMER_RES_BITS))
#define LEDC_TIMER_DIVIDER ((uint32_t)(LEDC_TIMER_DIVIDER_NUM / LEDC_TIMER_DIVIDER_DEN))
#define LEDC_DUTY_MAX        ((1U << LEDC_TIMER_RES_BITS) - 1U)
#define LEDC_DUTY_SHIFT      4U

#if ((LEDC_TIMER_DIVIDER_NUM / LEDC_TIMER_DIVIDER_DEN) == 0) || ((LEDC_TIMER_DIVIDER_NUM / LEDC_TIMER_DIVIDER_DEN) > 0x3FFFFU)
#error "LEDC_TIMER_DIVIDER fuera de rango para el campo de 18 bits"
#endif

static void ledc_set_duty(uint32_t duty);

static void gpio_init(void) {
    // GPIO3 queda como salida controlada por LEDC (sin pulls, función GPIO)
    uint32_t reg = REG32(IO_MUX_GPIO3_REG);
    reg &= ~(IO_MUX_FUN_IE | IO_MUX_FUN_PU | IO_MUX_FUN_PD | IO_MUX_MCU_SEL_MASK);
    reg |= (IO_MUX_MCU_SEL_GPIO << 12);
    REG32(IO_MUX_GPIO3_REG) = reg;
    REG32(GPIO_ENABLE_W1TS_REG) = LED_MASK;

    reg = REG32(DR_REG_IO_MUX_BASE + 0x0018);      // IO_MUX_GPIO5_REG (MTDI)
    reg &= ~(IO_MUX_FUN_IE | IO_MUX_FUN_PU | IO_MUX_FUN_PD | IO_MUX_MCU_SEL_MASK);
    reg |= (IO_MUX_MCU_SEL_GPIO << 12);
    REG32(DR_REG_IO_MUX_BASE + 0x0018) = reg;
    REG32(GPIO_ENABLE_W1TS_REG) = LED2_MASK;

    // GPIO0 en modo analógico (sin OE ni pulls) para el potenciómetro
    reg = REG32(IO_MUX_GPIO0_REG);
    reg &= ~(IO_MUX_FUN_IE | IO_MUX_FUN_PU | IO_MUX_FUN_PD | IO_MUX_MCU_SEL_MASK);
    REG32(IO_MUX_GPIO0_REG) = reg;
    REG32(GPIO_ENABLE_W1TC_REG) = POT_MASK;
}

static void adc_init(void) {
    // Clock/reset del SARADC
    REG32(SYSTEM_PERIP_CLK_EN0_REG) |= SYSTEM_APB_SARADC_CLK_EN;
    REG32(SYSTEM_PERIP_RST_EN0_REG) |= SYSTEM_APB_SARADC_RST;
    REG32(SYSTEM_PERIP_RST_EN0_REG) &= ~SYSTEM_APB_SARADC_RST;

    // Forzar ADC encendido, activar clock y fijar divisor
    uint32_t ctrl = REG32(APB_SARADC_CTRL_REG);
    ctrl |= APB_SARADC_SAR_CLK_GATED;
    ctrl &= ~APB_SARADC_XPD_SAR_FORCE_M;
    ctrl |= (3U << APB_SARADC_XPD_SAR_FORCE_S);
    ctrl &= ~APB_SARADC_SAR_CLK_DIV_M;
    ctrl |= (4U << APB_SARADC_SAR_CLK_DIV_S);
    ctrl &= ~(APB_SARADC_START_FORCE | APB_SARADC_START);
    REG32(APB_SARADC_CTRL_REG) = ctrl;

    // Configurar canal 0 con atenuación 11 dB (full scale ~3.3 V)
    uint32_t sample = REG32(APB_SARADC_ONETIME_SAMPLE_REG);
    sample |= APB_SARADC1_ONETIME_SAMPLE;
    sample &= ~APB_SARADC_ONETIME_CHANNEL_M;
    sample |= (0U << APB_SARADC_ONETIME_CHANNEL_S);
    sample &= ~APB_SARADC_ONETIME_ATTEN_M;
    sample |= (ADC_ATTEN_11DB << APB_SARADC_ONETIME_ATTEN_S);
    sample &= ~APB_SARADC_ONETIME_START;
    REG32(APB_SARADC_ONETIME_SAMPLE_REG) = sample;

    // Habilitar y limpiar flag de conversión terminada
    REG32(APB_SARADC_INT_ENA_REG) |= APB_SARADC_ADC1_DONE_INT_ENA;
    REG32(APB_SARADC_INT_CLR_REG) = APB_SARADC_ADC1_DONE_INT_CLR;
}

static void ledc_init(void) {
    // Activar clock/reset de LEDC
    REG32(SYSTEM_PERIP_CLK_EN0_REG) |= SYSTEM_LEDC_CLK_EN;
    REG32(SYSTEM_PERIP_RST_EN0_REG) |= SYSTEM_LEDC_RST;
    REG32(SYSTEM_PERIP_RST_EN0_REG) &= ~SYSTEM_LEDC_RST;

    // Seleccionar reloj APB (80 MHz) y habilitar módulo
    uint32_t ledc_conf = REG32(LEDC_CONF_REG);
    ledc_conf |= LEDC_CLK_EN;
    ledc_conf &= ~LEDC_APB_CLK_SEL_M;
    ledc_conf |= (LEDC_APB_CLK_SEL_APB << LEDC_APB_CLK_SEL_S);
    REG32(LEDC_CONF_REG) = ledc_conf;

    // Configurar temporizador low-speed 0: resolución y divisor fraccionario (8 bits fracc.)
    uint32_t timer_conf = REG32(LEDC_LSTIMER0_CONF_REG);
    timer_conf &= ~(LEDC_CLK_DIV_LSTIMER0_M | LEDC_LSTIMER0_DUTY_RES_M | LEDC_LSTIMER0_PAUSE);
    timer_conf |= ((LEDC_TIMER_DIVIDER << LEDC_CLK_DIV_LSTIMER0_S) & LEDC_CLK_DIV_LSTIMER0_M);
    timer_conf |= ((LEDC_TIMER_RES_BITS << LEDC_LSTIMER0_DUTY_RES_S) & LEDC_LSTIMER0_DUTY_RES_M);
    REG32(LEDC_LSTIMER0_CONF_REG) = timer_conf;
    REG32(LEDC_LSTIMER0_CONF_REG) |= LEDC_LSTIMER0_RST;
    REG32(LEDC_LSTIMER0_CONF_REG) &= ~LEDC_LSTIMER0_RST;
    REG32(LEDC_LSTIMER0_CONF_REG) |= LEDC_LSTIMER0_PARA_UP;

    // Inicializar canal 0: duty 0, usa timer 0, habilita salida
    REG32(LEDC_LSCH0_HPOINT_REG) = 0;
    REG32(LEDC_LSCH0_DUTY_REG) = 0;
    uint32_t ch0_conf0 = REG32(LEDC_LSCH0_CONF0_REG);
    ch0_conf0 &= ~(LEDC_TIMER_SEL_LSCH0_M | LEDC_IDLE_LV_LSCH0 | LEDC_SIG_OUT_EN_LSCH0);
    ch0_conf0 |= LEDC_SIG_OUT_EN_LSCH0; // Timer 0 (valor 0)
    REG32(LEDC_LSCH0_CONF0_REG) = ch0_conf0;
    REG32(LEDC_LSCH0_CONF0_REG) |= LEDC_PARA_UP_LSCH0;
    uint32_t ch0_conf1 = REG32(LEDC_LSCH0_CONF1_REG);
    ch0_conf1 |= LEDC_DUTY_START_LSCH0;
    REG32(LEDC_LSCH0_CONF1_REG) = ch0_conf1;

    // Conectar señal LEDC canal 0 a GPIO3
    uint32_t func3 = REG32(GPIO_FUNC3_OUT_SEL_CFG_REG);
    func3 &= ~(GPIO_FUNC3_OEN_INV_SEL | GPIO_FUNC3_OEN_SEL | GPIO_FUNC3_OUT_INV_SEL | GPIO_FUNC3_OUT_SEL_M);
    func3 |= (LEDC_LS_SIG_OUT0_IDX << GPIO_FUNC3_OUT_SEL_S) & GPIO_FUNC3_OUT_SEL_M;
    REG32(GPIO_FUNC3_OUT_SEL_CFG_REG) = func3;

    ledc_set_duty(0);
}

static uint16_t adc_sample_once(void) {
    // Pulso de start (low→high) para disparar conversión oneshot
    uint32_t sample = REG32(APB_SARADC_ONETIME_SAMPLE_REG);
    sample &= ~APB_SARADC_ONETIME_START;
    REG32(APB_SARADC_ONETIME_SAMPLE_REG) = sample;
    for (volatile uint32_t i = 0; i < 32; ++i) {
        __asm__ volatile("nop");
    }
    sample |= APB_SARADC_ONETIME_START;
    REG32(APB_SARADC_ONETIME_SAMPLE_REG) = sample;

    while ((REG32(APB_SARADC_INT_ST_REG) & APB_SARADC_ADC1_DONE_INT_ST) == 0U) {
    }

    // Capturo 12 bits útiles y limpio flag
    uint32_t raw = REG32(APB_SARADC_1_DATA_STATUS_REG) & 0x1FFFFU;
    REG32(APB_SARADC_INT_CLR_REG) = APB_SARADC_ADC1_DONE_INT_CLR;
    return (uint16_t)(raw & 0x0FFFU);
}

static void short_delay(void) {
    // Busy-wait simple (no timers configurados)
    for (volatile uint32_t i = 0; i < LOOP_DELAY; ++i) {
        __asm__ volatile("nop");
    }
}

static void ledc_set_duty(uint32_t duty) {
    if (duty > LEDC_DUTY_MAX) {
        duty = LEDC_DUTY_MAX;
    }
    REG32(LEDC_LSCH0_DUTY_REG) = duty << LEDC_DUTY_SHIFT;
    REG32(LEDC_LSCH0_CONF1_REG) |= LEDC_DUTY_START_LSCH0;
    REG32(LEDC_LSCH0_CONF0_REG) |= LEDC_PARA_UP_LSCH0;
}

int main(void) {
    // Deshabilitar watchdogs para bucle infinito didáctico
    disable_timg_wdt(TIMG0_BASE);
    disable_timg_wdt(TIMG1_BASE);
    disable_rtc_wdts();

    // Inicializaciones básicas de GPIO, ADC y PWM
    gpio_init();
    adc_init();
    ledc_init();

    // Loop principal: LED rojo por umbral digital, LED azul via PWM proporcional
    while (1) {
        uint16_t sample = adc_sample_once();
        if (sample >= ADC_THRESHOLD) {
            REG32(GPIO_OUT_W1TS_REG) = LED2_MASK;
        } else {
            REG32(GPIO_OUT_W1TC_REG) = LED2_MASK;
        }

        uint32_t pwm_input = (sample > ADC_ZERO_BIAS) ? (sample - ADC_ZERO_BIAS) : 0U;
        uint32_t pwm_range = 4095U - ADC_ZERO_BIAS;
        uint32_t duty = (pwm_input * LEDC_DUTY_MAX) / pwm_range;
        ledc_set_duty(duty);

        short_delay();
    }
}