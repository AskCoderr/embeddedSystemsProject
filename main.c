/*
 * ColdTrace - firmware v5 (final): cold-chain logger with analytics and a tamper-proof log
 * Board : STM32F401 "Black Pill" (WeAct V3), 25 MHz crystal
 * Tool  : Keil uVision 5, CMSIS-CORE + Device:Startup only (register level, no HAL)
 *
 * Every reset it runs a self-test:
 *   1. PC13 onboard LED blinks 3 times      -> clock + flashing work
 *   2. Green, amber, red LEDs light in turn  -> PB12 / PB13 / PB14
 *   3. Buzzer beeps once                     -> PB10 open-drain (5V buzzer), or PB1 if BUZZER_OPEN_DRAIN = 0
 *   4. Banner sent to the phone              -> HC-05 on USART2 (PA2 TX, PA3 RX), 9600 baud
 *   5. I2C scan of PB8/PB7, reported on the phone (expect 0x3C OLED and 0x68 MPU6050)
 *   6. OLED splash, then MPU6050 tilt calibration (keep the box still for ~1 s)
 *   7. Real-time clock (32.768 kHz crystal) and trip memory in the backup registers
 *   8. SD card: opens (or creates) TRIPLOG.CSV and writes a BOOT row, read back to check it
 *
 * Then it runs the mini-ColdTrace loop:
 *   - Reed switch on PA10 (EXTI10 interrupt): DOOR OPEN / CLOSED + beep
 *   - DS18B20 probes (up to 2) on PA8 every 2 s: the payload temperature that drives the alarms
 *   - DHT11 on PB15 every 2 s: humidity + air temperature (backup if no probe is connected)
 *   - MPU6050 every 10 ms: shock (> SHOCK_MG), free fall (drop), tilt from the calibrated position
 *   - OLED shows temperature, humidity, door, tilt, g-force and the current alert
 *   - Green = OK, amber = warning (temp high, shock, tilt), red = alarm (door open, temp too high)
 *   - SD card (SPI1: PA4 CS, PA5 SCK, PA6 MISO, PA7 MOSI): one CSV row every 5 s plus a row
 *     for every event (door, shock, drop, tilt, state change). Every sector written is read
 *     back and compared. Open TRIPLOG.CSV in Excel on a laptop to see the whole trip.
 *   - Cold-chain analytics on every reading:
 *       * Mean Kinetic Temperature (MKT), the pharma-standard average
 *       * Virtual vial monitor: remaining heat budget (WHO VVM14 model), 100 % -> 0 %
 *       * Prediction: "too warm in 4 min" from the temperature trend, before the alarm
 *       * Freeze detection, probe spike filter and probe cross-check
 *   - Tamper-proof log: each CSV row ends with an HMAC-SHA256 chain code;
 *     verify_log.py on a laptop finds any edited, deleted or inserted row
 *   - Real date/time in the log once set from the phone (t2026-10-21 09:30:00)
 *   - The trip survives a reset (and power-off with a CR2032 on VB)
 *   - Low power: the CPU sleeps between jobs; phone input wakes it by interrupt
 *   - Phone commands: h lists them all
 *
 * Parts that are not wired simply report errors; nothing else breaks.
 */

#include "RTE_Components.h"
#include CMSIS_device_header          /* if this line errors, use: #include "stm32f4xx.h" */
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

/* ---------------- Settings you may need to change ---------------- */
#define USE_84MHZ               1     /* 0 = stay on the 16 MHz internal clock               */
#define BUZZER_PASSIVE          0     /* 1 if your buzzer only clicks with plain on/off      */
#define BUZZER_OPEN_DRAIN       1     /* 1 = 5V buzzer: + to 5V rail, - to PB10              */
                                      /* 0 = 3.3V buzzer: + to PB1, - to GND                 */
#define REED_LEVEL_WITH_MAGNET  0     /* change to 1 if OPEN/CLOSED come out reversed        */
#define TEMP_PROFILE            2     /* 0 = room test (alarm 37 C)
                                         1 = vaccine cold chain, safe 2.0 - 8.0 C
                                         2 = DEMO: safe band centred on the starting temperature */
#define DEMO_ALARM_C10          30    /* demo: alarm 3.0 C above/below the centre             */
#define DEMO_WARN_C10           20    /* demo: warning 2.0 C above/below the centre           */
#if TEMP_PROFILE == 1
#define TEMP_LOW_ALARM_C10      20    /* below 2.0 C: too cold, freezing risk                */
#define TEMP_LOW_WARN_C10       30    /* below 3.0 C: getting close to the lower limit       */
#define TEMP_WARN_C10           70    /* 7.0 C and above: getting close to the upper limit   */
#define TEMP_ALARM_C10          80    /* 8.0 C and above: too warm                           */
#elif TEMP_PROFILE == 0
#define TEMP_LOW_ALARM_C10      (-400)/* low limits switched off for room testing            */
#define TEMP_LOW_WARN_C10       (-400)
#define TEMP_WARN_C10           350   /* 35.0 C */
#define TEMP_ALARM_C10          370   /* 37.0 C */
#else                                 /* demo: no alarms until the first reading sets the band */
#define TEMP_LOW_ALARM_C10      (-400)
#define TEMP_LOW_WARN_C10       (-400)
#define TEMP_WARN_C10           900
#define TEMP_ALARM_C10          900
#endif
#define SHOCK_MG                2500U /* total acceleration above 2.5 g counts as a shock    */
#define FREEFALL_MG             350U  /* below 0.35 g for 40 ms counts as a drop             */
#define TILT_WARN_DEG           45    /* tilt from the calibrated position that warns        */
#define I2C_FAST                1     /* 1 = 400 kHz, 0 = 100 kHz (try 0 if I2C is flaky)     */
#define I2C_SCL_ON_PB8          1     /* 1 = I2C clock on PB8 instead of PB6                 */
#define DHT_PERIOD_MS           2000U
#define UART_BAUD               9600U
#define SD_LOG_PERIOD_MS        5000U /* one CSV row every 5 s (events are logged at once)  */
#define SD_SPI_FAST_BR          3     /* SPI speed after start-up: 3 = 84/16 = 5.25 MHz.     */
                                      /* If writes fail on long jumpers, try 4 or 5 (slower) */

/* Cold-chain analytics */
#define VVM_DAYS_AT_37          14    /* vaccine heat stability: WHO VVM category 2, 7, 14 or 30 */
#define VVM_EA_R                14340.0 /* Arrhenius constant fitted to the WHO VVM points       */
#define VVM_DEMO_SPEEDUP        500.0 /* demo profile only: heat budget runs 500x faster       */
#define MKT_DH_R                10000.0 /* MKT: 83.144 kJ/mol / R, the standard pharma value   */
#define PREDICT_WINDOW          20    /* trend from the last 20 readings (40 s)              */
#define PREDICT_WARN_MIN        15.0f /* warn when an alarm is predicted within 15 min       */
#define PREDICT_MIN_SLOPE       0.10f /* ignore trends slower than 0.1 C per minute          */
#define PROBE_SPIKE_C10         50    /* a 5.0 C jump in 2 s must be confirmed before use     */
#define PROBE_DIFF_C10          150   /* probes more than 15.0 C apart = possible probe fault */
#define FREEZE_C10              (-5)  /* any probe at or below -0.5 C: doses may be frozen     */
#define LOG_KEY  "ColdTrace-23CSE304-change-me"  /* secret for the log chain: change it, and
                                                   use the same text in verify_log.py      */
#define LOW_POWER               1     /* 1 = CPU sleeps between jobs (wakes every 1 ms tick) */

/* ---------------- Pins ---------------- */
#define LED_G      12   /* PB12 */
#define LED_A      13   /* PB13 */
#define LED_R      14   /* PB14 */
#if BUZZER_OPEN_DRAIN
#define BUZ        10   /* PB10: 5V-tolerant pin, pulls the buzzer's - side to GND */
#else
#define BUZ        1    /* PB1  */
#endif
#define DHT        15   /* PB15 */
#define REED       10   /* PA10 */
#define BOARD_LED  13   /* PC13, LOW = on */
#if I2C_SCL_ON_PB8
#define I2C_SCL    8    /* PB8  */
#else
#define I2C_SCL    6    /* PB6  */
#endif
#define I2C_SDA    7    /* PB7  */
#define OW_PIN     8    /* PA8: DS18B20 1-Wire data, 4.7 kOhm pull-up to 3.3V */
#define MAX_PROBES 2

#define OLED_ADDR  0x3C
#define MPU_ADDR   0x68

#define PIN_HIGH(p, n)  ((p)->BSRR = (1U << (n)))
#define PIN_LOW(p, n)   ((p)->BSRR = (1U << ((n) + 16U)))
#define PIN_READ(p, n)  ((((p)->IDR) >> (n)) & 1U)

#if BUZZER_OPEN_DRAIN                  /* open-drain: LOW = current flows = beep */
#define BUZ_ON()   PIN_LOW(GPIOB, BUZ)
#define BUZ_OFF()  PIN_HIGH(GPIOB, BUZ)
#else
#define BUZ_ON()   PIN_HIGH(GPIOB, BUZ)
#define BUZ_OFF()  PIN_LOW(GPIOB, BUZ)
#endif

/* ---------------- Globals: add these to Keil's Watch window ---------------- */
volatile uint32_t g_ms = 0;
volatile uint8_t  g_door_irq = 0;
volatile uint32_t g_door_irq_count = 0;
volatile uint8_t  g_door_open = 0;
volatile int16_t  g_temp_c10 = 0;      /* payload temperature x10 used for alarms (warmest probe) */
volatile int16_t  g_tmin_c10 = 0;      /* coldest reading used for the "too cold" check */
volatile uint8_t  g_temp_valid = 0;    /* 1 when any temperature source is working */
volatile uint8_t  g_temp_src = 0;      /* 0 none, 1 DS18B20 probes, 2 DHT11 air (backup) */
volatile int16_t  g_air_c10 = 0;       /* DHT11 air temperature x10 */
volatile int16_t  g_probe_c10[MAX_PROBES];
volatile uint8_t  g_probe_ok[MAX_PROBES];
volatile uint8_t  g_probe_n = 0;       /* DS18B20 probes found on the bus */
volatile uint8_t  g_hum = 0;           /* relative humidity, % */
volatile uint8_t  g_dht_valid = 0;
volatile uint32_t g_dht_ok = 0;
volatile uint32_t g_dht_err = 0;
volatile int      g_dht_last_error = 0;

volatile uint8_t  g_oled_ok = 0;
volatile uint8_t  g_mpu_ok = 0;
volatile uint8_t  g_mpu_who = 0;       /* WHO_AM_I, 0x68 on a genuine MPU6050 */
volatile uint32_t g_accel_mg = 0;      /* total acceleration, milli-g (1000 = still) */
volatile uint32_t g_peak_mg = 0;       /* highest since the last report */
volatile int      g_tilt_deg = 0;
volatile uint8_t  g_tilted = 0;
volatile uint32_t g_shock_count = 0;

/* Alarm limits in use (x10 C). Fixed for profiles 0/1; set from the first reading in demo mode */
volatile int16_t  g_lo_alarm = TEMP_LOW_ALARM_C10;
volatile int16_t  g_lo_warn  = TEMP_LOW_WARN_C10;
volatile int16_t  g_hi_warn  = TEMP_WARN_C10;
volatile int16_t  g_hi_alarm = TEMP_ALARM_C10;
volatile uint8_t  g_band_set = (TEMP_PROFILE != 2);

/* Trip record (phone command 'x' prints it, 'z' clears it) */
volatile uint32_t g_excursion_s = 0;   /* total seconds outside the safe range */
volatile uint32_t g_door_opens = 0;
volatile int16_t  g_min_c10 = 32767;
volatile int16_t  g_max_c10 = -32768;
volatile uint32_t g_trip_start_ms = 0;
volatile uint32_t g_i2c_err = 0;
volatile uint32_t g_log_peak_mg = 0;   /* highest g since the last SD row */

/* Cold-chain analytics (kept in the RTC backup registers across resets) */
volatile double   g_mkt_sum = 0.0;     /* sum of exp(-dH/RT) over all readings */
volatile uint32_t g_mkt_n = 0;
volatile float    g_vvm_used = 0.0f;   /* share of the heat budget used: 0 = fresh, 1 = discard */
volatile uint8_t  g_frozen = 0;        /* latched: a probe went below FREEZE_C10 */
volatile uint8_t  g_probe_fault = 0;   /* the two probes disagree */
volatile uint32_t g_probe_spikes = 0;  /* single wild readings that were ignored */
volatile uint8_t  g_pred_active = 0;   /* 1 = too warm predicted, 2 = too cold predicted */
volatile float    g_pred_min = 0.0f;   /* minutes until the predicted alarm */
volatile float    g_slope_cpm = 0.0f;  /* temperature trend, C per minute */
volatile uint8_t  g_rtc_ok = 0;        /* 32.768 kHz crystal running */

/* SD card log */
volatile uint8_t  g_sd_ok = 0;        /* 1 = card ready and TRIPLOG.CSV open */
volatile uint32_t g_sd_rows = 0;      /* rows written since power-on */
volatile uint32_t g_sd_errors = 0;
#define LOG_Q 6                         /* events waiting to be written to the card */
static char    g_logq[LOG_Q][24];
static uint8_t g_logq_n = 0;
static void log_event(const char *text)
{
    if (g_logq_n >= LOG_Q) return;
    {
        uint32_t i;
        for (i = 0; i < sizeof g_logq[0] - 1U && text[i]; i++) g_logq[g_logq_n][i] = text[i];
        g_logq[g_logq_n][i] = '\0';
    }
    g_logq_n++;
}

typedef enum { ST_OK = 0, ST_WARN, ST_ALARM } state_t;
static const char *const state_name[] = { "OK", "WARNING", "ALARM" };
static const char *const state_short[] = { "OK", "WARN", "ALARM" };
volatile state_t g_state = ST_WARN;

static uint32_t g_sysclk = 16000000U;
static uint32_t g_pclk1  = 16000000U;
static uint32_t g_cyc_per_us = 16U;

static char     g_event[24] = "";      /* last shock / drop message */
static uint32_t g_event_until = 0;     /* show it until this time (ms) */

/* ================================================================
 *  Timing: SysTick (1 ms) and DWT cycle counter (microseconds)
 * ================================================================ */
void SysTick_Handler(void) { g_ms++; }

static void delay_ms(uint32_t ms)
{
    uint32_t start = g_ms;
    while ((g_ms - start) < ms) { }
}

#define DEMCR (*(volatile uint32_t *)0xE000EDFCU)
static void dwt_init(void)
{
    DEMCR |= (1U << 24);      /* TRCENA: enable the DWT unit   */
    DWT->CYCCNT = 0;
    DWT->CTRL |= 1U;          /* CYCCNTENA: start cycle counter */
}

static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * g_cyc_per_us;
    while ((DWT->CYCCNT - start) < ticks) { }
}

static uint8_t time_before(uint32_t until) { return (int32_t)(until - g_ms) > 0; }

/* ================================================================
 *  Clock: 25 MHz HSE -> PLL -> 84 MHz SYSCLK, APB1 42 MHz, APB2 84 MHz
 *  If the crystal does not start, it stays on the 16 MHz HSI.
 * ================================================================ */
static void clock_init(void)
{
#if USE_84MHZ
    uint32_t t;

    RCC->CR |= RCC_CR_HSEON;
    for (t = 0; !(RCC->CR & RCC_CR_HSERDY); t++) {
        if (t > 1000000U) return;                 /* no crystal: stay on HSI */
    }

    RCC->CFGR &= ~RCC_CFGR_SW;                    /* run from HSI while changing the PLL */
    while (RCC->CFGR & RCC_CFGR_SWS) { }
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY) { }

    FLASH->ACR = (1U << 8) | (1U << 9) | (1U << 10) | 2U;   /* prefetch, I/D cache, 2 wait states */

    /* PLL: 25 MHz / M25 = 1 MHz, x N336 = 336 MHz, / P4 = 84 MHz, / Q7 = 48 MHz */
    RCC->PLLCFGR = (25U << 0) | (336U << 6) | (1U << 16) | (1U << 22) | (7U << 24);

    /* AHB /1, APB1 /2 (max 42 MHz), APB2 /1 */
    RCC->CFGR = (RCC->CFGR & ~((0xFU << 4) | (7U << 10) | (7U << 13))) | (4U << 10);

    RCC->CR |= RCC_CR_PLLON;
    for (t = 0; !(RCC->CR & RCC_CR_PLLRDY); t++) {
        if (t > 1000000U) return;
    }

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { }

    g_sysclk = 84000000U;
    g_pclk1  = 42000000U;
#endif
}

/* ================================================================
 *  GPIO
 * ================================================================ */
static void pin_mode(GPIO_TypeDef *p, uint32_t pin, uint32_t mode)   /* 0 in, 1 out, 2 AF, 3 analog */
{
    p->MODER = (p->MODER & ~(3U << (pin * 2U))) | (mode << (pin * 2U));
}

static void pin_pull(GPIO_TypeDef *p, uint32_t pin, uint32_t pull)   /* 0 none, 1 up, 2 down */
{
    p->PUPDR = (p->PUPDR & ~(3U << (pin * 2U))) | (pull << (pin * 2U));
}

static void gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR;

    PIN_HIGH(GPIOC, BOARD_LED);                          /* onboard LED off */
    pin_mode(GPIOC, BOARD_LED, 1);

    PIN_LOW(GPIOB, LED_G); PIN_LOW(GPIOB, LED_A); PIN_LOW(GPIOB, LED_R);
    BUZ_OFF();
#if BUZZER_OPEN_DRAIN
    GPIOB->OTYPER |= (1U << BUZ);        /* open-drain: the pin never drives 3.3V into the 5V buzzer */
#endif
    pin_mode(GPIOB, LED_G, 1);
    pin_mode(GPIOB, LED_A, 1);
    pin_mode(GPIOB, LED_R, 1);
    pin_mode(GPIOB, BUZ, 1);

    /* DHT11 data: open-drain output, released (the pull-up holds it high) */
    GPIOB->OTYPER |= (1U << DHT);
    pin_pull(GPIOB, DHT, 1);
    PIN_HIGH(GPIOB, DHT);
    pin_mode(GPIOB, DHT, 1);

    /* Reed switch DO: input with pull-up */
    pin_mode(GPIOA, REED, 0);
    pin_pull(GPIOA, REED, 1);
}

static void leds(uint8_t g, uint8_t a, uint8_t r)
{
    if (g) PIN_HIGH(GPIOB, LED_G); else PIN_LOW(GPIOB, LED_G);
    if (a) PIN_HIGH(GPIOB, LED_A); else PIN_LOW(GPIOB, LED_A);
    if (r) PIN_HIGH(GPIOB, LED_R); else PIN_LOW(GPIOB, LED_R);
}

static void buzzer(uint32_t ms)
{
#if BUZZER_PASSIVE
    uint32_t start = g_ms;                  /* ~2 kHz square wave */
    while ((g_ms - start) < ms) {
        BUZ_ON();  delay_us(250);
        BUZ_OFF(); delay_us(250);
    }
#else
    BUZ_ON();
    delay_ms(ms);
    BUZ_OFF();
#endif
}

/* ================================================================
 *  USART2 -> HC-05   (PA2 = TX to HC-05 RXD, PA3 = RX from HC-05 TXD)
 * ================================================================ */
static void uart_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    (void)RCC->APB1ENR;

    pin_mode(GPIOA, 2, 2);
    pin_mode(GPIOA, 3, 2);
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~((0xFU << 8) | (0xFU << 12))) | (7U << 8) | (7U << 12);  /* AF7 */
    pin_pull(GPIOA, 3, 1);

    USART2->BRR = (g_pclk1 + UART_BAUD / 2U) / UART_BAUD;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
    NVIC_SetPriority(USART2_IRQn, 3);
    NVIC_EnableIRQ(USART2_IRQn);
}

/* Received characters go into a small ring buffer; the interrupt also wakes the CPU */
static volatile uint8_t g_rx_buf[64];
static volatile uint8_t g_rx_head = 0, g_rx_tail = 0;
void USART2_IRQHandler(void)
{
    uint32_t sr = USART2->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
        uint8_t c = (uint8_t)USART2->DR;           /* reading DR clears RXNE and overrun */
        uint8_t n = (uint8_t)((g_rx_head + 1U) & 63U);
        if (n != g_rx_tail) { g_rx_buf[g_rx_head] = c; g_rx_head = n; }
    }
}

static void uart_putc(char c)
{
    while (!(USART2->SR & USART_SR_TXE)) { }
    USART2->DR = (uint8_t)c;
}

static void uart_puts(const char *s)
{
    while (*s) uart_putc(*s++);
}

static void uart_printf(const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    uart_puts(buf);
}

static int uart_getc(void)
{
    int c;
    if (g_rx_tail == g_rx_head) return -1;
    c = g_rx_buf[g_rx_tail];
    g_rx_tail = (uint8_t)((g_rx_tail + 1U) & 63U);
    return c;
}

/* ================================================================
 *  Reed switch on PA10 -> EXTI10 (both edges)
 * ================================================================ */
static void reed_irq_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR;

    SYSCFG->EXTICR[2] &= ~(0xFU << 8);   /* EXTI10 source = port A */
    EXTI->IMR  |= (1U << REED);
    EXTI->RTSR |= (1U << REED);
    EXTI->FTSR |= (1U << REED);
    EXTI->PR    = (1U << REED);

    NVIC_SetPriority(EXTI15_10_IRQn, 2);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & (1U << REED)) {
        EXTI->PR = (1U << REED);         /* write 1 to clear */
        g_door_irq = 1;
        g_door_irq_count++;
    }
}

static uint8_t reed_door_open(void)
{
    return (uint8_t)(PIN_READ(GPIOA, REED) != REED_LEVEL_WITH_MAGNET);
}

/* ================================================================
 *  DHT11 on PB15
 * ================================================================ */
static int32_t dht_wait(uint32_t level, uint32_t timeout_us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t limit = timeout_us * g_cyc_per_us;
    while (PIN_READ(GPIOB, DHT) != level) {
        if ((DWT->CYCCNT - start) > limit) return -1;
    }
    return (int32_t)((DWT->CYCCNT - start) / g_cyc_per_us);
}

/* Returns 0 on success, or: -1 no response, -2 bad response timing,
   -3 timeout while reading bits, -4 checksum mismatch */
static int dht11_read(int16_t *temp_c10, uint8_t *hum)
{
    uint8_t d[5] = { 0, 0, 0, 0, 0 };
    int32_t high_us;
    int i, err = 0;

    PIN_LOW(GPIOB, DHT);                 /* start signal: hold low >= 18 ms */
    delay_ms(20);

    __disable_irq();                     /* keep the microsecond timing clean (~5 ms) */
    PIN_HIGH(GPIOB, DHT);

    if      (dht_wait(1, 50)  < 0) err = -1;
    else if (dht_wait(0, 100) < 0) err = -1;
    else if (dht_wait(1, 100) < 0) err = -2;
    else if (dht_wait(0, 100) < 0) err = -2;
    else {
        for (i = 0; i < 40; i++) {
            if (dht_wait(1, 80) < 0) { err = -3; break; }
            high_us = dht_wait(0, 100);  /* 26-28 us = 0, ~70 us = 1 */
            if (high_us < 0) { err = -3; break; }
            d[i / 8] = (uint8_t)(d[i / 8] << 1);
            if (high_us > 45) d[i / 8] |= 1U;
        }
    }
    __enable_irq();

    if (err) return err;
    if ((uint8_t)(d[0] + d[1] + d[2] + d[3]) != d[4]) return -4;

    *hum = d[0];
    {
        uint8_t dec = (uint8_t)(d[3] & 0x0FU);
        int16_t t;
        if (dec > 9U) dec = 9U;
        t = (int16_t)(d[2] * 10 + dec);
        if (d[3] & 0x80U) t = (int16_t)(-t);
        *temp_c10 = t;
    }
    return 0;
}

/* ================================================================
 *  DS18B20 probes on PA8 (1-Wire). Both probes share one data wire;
 *  each has a unique 64-bit ID found by the ROM search below.
 * ================================================================ */
#define OW_LOW()   PIN_LOW(GPIOA, OW_PIN)
#define OW_REL()   PIN_HIGH(GPIOA, OW_PIN)
#define OW_READ()  PIN_READ(GPIOA, OW_PIN)

static uint8_t g_rom[MAX_PROBES][8];
static uint8_t g_probe_fail[MAX_PROBES];
static uint8_t g_conv_started = 0;

static void ow_init_pin(void)
{
    GPIOA->OTYPER |= (1U << OW_PIN);        /* open-drain: only ever pulls the line low */
    pin_pull(GPIOA, OW_PIN, 1);
    OW_REL();
    pin_mode(GPIOA, OW_PIN, 1);
}

/* Reset pulse. Returns 1 if at least one probe answers with a presence pulse. */
static int ow_reset(void)
{
    int present;
    OW_LOW();
    delay_us(480);
    __disable_irq();
    OW_REL();
    delay_us(70);
    present = !OW_READ();
    __enable_irq();
    delay_us(410);
    return present;
}

static void ow_write_bit(uint8_t b)
{
    __disable_irq();
    OW_LOW();
    if (b) { delay_us(6);  OW_REL(); __enable_irq(); delay_us(64); }
    else   { delay_us(60); OW_REL(); __enable_irq(); delay_us(10); }
}

static uint8_t ow_read_bit(void)
{
    uint8_t b;
    __disable_irq();
    OW_LOW();
    delay_us(3);
    OW_REL();
    delay_us(10);
    b = (uint8_t)OW_READ();
    __enable_irq();
    delay_us(53);
    return b;
}

static void ow_write_byte(uint8_t v)
{
    int i;
    for (i = 0; i < 8; i++) { ow_write_bit(v & 1U); v >>= 1; }
}

static uint8_t ow_read_byte(void)
{
    uint8_t v = 0;
    int i;
    for (i = 0; i < 8; i++) { v >>= 1; if (ow_read_bit()) v |= 0x80U; }
    return v;
}

static uint8_t ow_crc8(const uint8_t *d, int n)
{
    uint8_t crc = 0;
    int i;
    while (n--) {
        uint8_t in = *d++;
        for (i = 0; i < 8; i++) {
            uint8_t mix = (uint8_t)((crc ^ in) & 1U);
            crc >>= 1;
            if (mix) crc ^= 0x8CU;
            in >>= 1;
        }
    }
    return crc;
}

/* Find every probe on the bus (Maxim ROM search). Returns how many were found. */
static int ow_search_all(void)
{
    uint8_t rom[8] = { 0 };
    int last_disc = 0, count = 0, done = 0;

    while (!done && count < MAX_PROBES) {
        int bitn, last_zero = 0, fail = 0;
        if (!ow_reset()) break;
        ow_write_byte(0xF0);                               /* SEARCH ROM */
        for (bitn = 1; bitn <= 64; bitn++) {
            uint8_t idb = ow_read_bit(), cmp = ow_read_bit(), dir;
            int byte = (bitn - 1) / 8, bit = (bitn - 1) % 8;
            if (idb && cmp) { fail = 1; break; }          /* nobody answered */
            if (idb != cmp) dir = idb;
            else {
                if (bitn < last_disc) dir = (uint8_t)((rom[byte] >> bit) & 1U);
                else                  dir = (uint8_t)(bitn == last_disc);
                if (!dir) last_zero = bitn;
            }
            if (dir) rom[byte] |= (uint8_t)(1U << bit);
            else     rom[byte] &= (uint8_t)~(1U << bit);
            ow_write_bit(dir);
        }
        if (fail || ow_crc8(rom, 7) != rom[7] || rom[0] == 0) break;
        memcpy(g_rom[count], rom, 8);
        g_probe_ok[count] = 0;
        g_probe_fail[count] = 0;
        count++;
        last_disc = last_zero;
        if (last_disc == 0) done = 1;
    }
    g_probe_n = (uint8_t)count;
    return count;
}

/* Tell every probe to start a measurement (takes up to 750 ms) */
static void probes_start(void)
{
    if (g_probe_n == 0 || !ow_reset()) { g_conv_started = 0; return; }
    ow_write_byte(0xCC);                                   /* SKIP ROM: all probes */
    ow_write_byte(0x44);                                   /* CONVERT T */
    g_conv_started = 1;
}

/* Read the measurement of probe i. Returns 0 ok, -1 no answer, -2 bad CRC, -3 not ready */
static int probe_read(int i, int16_t *c10)
{
    uint8_t b[9];
    int k;
    int16_t raw;
    if (!ow_reset()) return -1;
    ow_write_byte(0x55);                                   /* MATCH ROM: this probe only */
    for (k = 0; k < 8; k++) ow_write_byte(g_rom[i][k]);
    ow_write_byte(0xBE);                                   /* READ SCRATCHPAD */
    for (k = 0; k < 9; k++) b[k] = ow_read_byte();
    if (ow_crc8(b, 8) != b[8]) return -2;
    raw = (int16_t)((b[1] << 8) | b[0]);                   /* 1/16 C per step */
    if (raw == 0x0550) return -3;                          /* 85.0 C = power-up value, no reading yet */
    *c10 = (int16_t)((raw * 10 + (raw >= 0 ? 8 : -8)) / 16);
    return 0;
}

static void probes_print_ids(void)
{
    int i;
    uart_printf("DS18B20: %u probe(s) found on PA8\r\n", (unsigned)g_probe_n);
    for (i = 0; i < g_probe_n; i++) {
        uart_printf("  P%d ID %02X-%02X%02X%02X%02X%02X%02X\r\n", i + 1, g_rom[i][0],
                    g_rom[i][6], g_rom[i][5], g_rom[i][4], g_rom[i][3], g_rom[i][2], g_rom[i][1]);
    }
    if (g_probe_n == 0) {
        if (!OW_READ()) uart_puts("  PA8 line is LOW: pull-up resistor missing, or DATA shorted to GND\r\n");
        else            uart_puts("  No probe answered: check red->3.3V, black->GND, yellow->PA8 column\r\n");
    }
}

/* Called every 2 s: read the last measurement, then start the next one */
static uint8_t g_spike_hold[MAX_PROBES];
static int16_t g_spike_val[MAX_PROBES];
static void probes_cycle(void)
{
    int i;
    if (g_probe_n == 0) {                                  /* plug-and-play: keep looking */
        if (ow_search_all() > 0) probes_print_ids();
    }
    if (g_conv_started) {
        for (i = 0; i < g_probe_n; i++) {
            int16_t t;
            int r = probe_read(i, &t);
            if (r == 0) {
                int16_t prev = g_probe_c10[i];
                if (g_probe_ok[i] && !g_spike_hold[i] && (t - prev > PROBE_SPIKE_C10 || prev - t > PROBE_SPIKE_C10)) {
                    g_spike_hold[i] = 1;                   /* big jump: believe it only if the next reading agrees */
                    g_spike_val[i] = t;
                } else {
                    if (g_spike_hold[i]) {
                        g_spike_hold[i] = 0;
                        if (t - g_spike_val[i] > 10 || g_spike_val[i] - t > 10) {
                            g_probe_spikes++;
                            uart_printf("[%lu s] P%d: one wild reading ignored\r\n", (unsigned long)(g_ms / 1000U), i + 1);
                            log_event(i ? "P2 SPIKE IGNORED" : "P1 SPIKE IGNORED");
                        }
                    }
                    g_probe_c10[i] = t;
                    g_probe_ok[i] = 1;
                    g_probe_fail[i] = 0;
                }
            } else if (++g_probe_fail[i] >= 3) {
                if (g_probe_ok[i]) uart_printf("[%lu s] Probe P%d lost (error %d)\r\n", (unsigned long)(g_ms / 1000U), i + 1, r);
                g_probe_ok[i] = 0;
            }
        }
    }
    probes_start();
}

/* Pick the temperatures the alarms use: probes if any work, else the DHT11 air reading */
static void update_temps(void)
{
    int i, n = 0;
    int16_t lo = 32767, hi = -32767;
    for (i = 0; i < g_probe_n; i++) {
        if (g_probe_ok[i]) {
            n++;
            if (g_probe_c10[i] < lo) lo = g_probe_c10[i];
            if (g_probe_c10[i] > hi) hi = g_probe_c10[i];
        }
    }
    if (n > 0)            g_temp_src = 1;
    else if (g_dht_valid) { lo = hi = g_air_c10; g_temp_src = 2; }
    else                  g_temp_src = 0;
    g_temp_valid = (uint8_t)(g_temp_src != 0);
    if (g_temp_valid) { g_temp_c10 = hi; g_tmin_c10 = lo; }
}

/* "12.3" style text for a x10 value */
static const char *c10_str(char *buf, int n, int16_t v)
{
    snprintf(buf, (size_t)n, "%s%d.%d", (v < 0 && v > -10) ? "-" : "", v / 10, (v < 0 ? -v : v) % 10);
    return buf;
}

/* ================================================================
 *  Tamper-proof log chain: HMAC-SHA256 (FIPS 180-4 / RFC 2104)
 *  Every CSV row ends with 8 hex digits = HMAC(LOG_KEY, previous code + this row).
 *  Change one character, delete or insert a row, and verify_log.py finds it.
 * ================================================================ */
typedef struct { uint32_t h[8]; uint8_t buf[64]; uint32_t n; uint32_t total; } sha256_t;

static const uint32_t K256[64] = {
    0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
    0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U, 0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
    0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU, 0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
    0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U, 0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
    0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U, 0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
    0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U, 0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
    0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
    0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U, 0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U
};
#define ROR32(x, k) (((x) >> (k)) | ((x) << (32U - (k))))

static void sha256_block(sha256_t *s, const uint8_t *p)
{
    uint32_t w[64], a, b, c, d, e, f, g, h, t1, t2;
    int i;
    for (i = 0; i < 16; i++) w[i] = ((uint32_t)p[4 * i] << 24) | ((uint32_t)p[4 * i + 1] << 16) | ((uint32_t)p[4 * i + 2] << 8) | p[4 * i + 3];
    for (i = 16; i < 64; i++) {
        uint32_t s0 = ROR32(w[i - 15], 7) ^ ROR32(w[i - 15], 18) ^ (w[i - 15] >> 3);
        uint32_t s1 = ROR32(w[i - 2], 17) ^ ROR32(w[i - 2], 19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    a = s->h[0]; b = s->h[1]; c = s->h[2]; d = s->h[3]; e = s->h[4]; f = s->h[5]; g = s->h[6]; h = s->h[7];
    for (i = 0; i < 64; i++) {
        t1 = h + (ROR32(e, 6) ^ ROR32(e, 11) ^ ROR32(e, 25)) + ((e & f) ^ (~e & g)) + K256[i] + w[i];
        t2 = (ROR32(a, 2) ^ ROR32(a, 13) ^ ROR32(a, 22)) + ((a & b) ^ (a & c) ^ (b & c));
        h = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
    }
    s->h[0] += a; s->h[1] += b; s->h[2] += c; s->h[3] += d; s->h[4] += e; s->h[5] += f; s->h[6] += g; s->h[7] += h;
}

static void sha256_init(sha256_t *s)
{
    static const uint32_t iv[8] = { 0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU, 0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U };
    memcpy(s->h, iv, sizeof iv);
    s->n = 0;
    s->total = 0;
}

static void sha256_update(sha256_t *s, const void *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    s->total += len;
    while (len--) {
        s->buf[s->n++] = *p++;
        if (s->n == 64U) { sha256_block(s, s->buf); s->n = 0; }
    }
}

static void sha256_final(sha256_t *s, uint8_t out[32])
{
    uint32_t bits = s->total * 8U;                       /* messages here are far below 512 MB */
    uint8_t pad = 0x80, zero = 0, len8[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    int i;
    len8[4] = (uint8_t)(bits >> 24); len8[5] = (uint8_t)(bits >> 16); len8[6] = (uint8_t)(bits >> 8); len8[7] = (uint8_t)bits;
    sha256_update(s, &pad, 1);
    while (s->n != 56U) sha256_update(s, &zero, 1);
    sha256_update(s, len8, 8);
    for (i = 0; i < 8; i++) {
        out[4 * i] = (uint8_t)(s->h[i] >> 24); out[4 * i + 1] = (uint8_t)(s->h[i] >> 16);
        out[4 * i + 2] = (uint8_t)(s->h[i] >> 8); out[4 * i + 3] = (uint8_t)s->h[i];
    }
}

static char g_chain[9] = "00000000";                    /* code of the last row written */

/* out8 = first 8 hex digits of HMAC-SHA256(LOG_KEY, prev8 + row[0..n)) */
static void chain_code(const char *prev8, const char *row, uint32_t n, char *out8)
{
    static const char hexd[] = "0123456789abcdef";
    uint8_t k[64], d[32];
    sha256_t s;
    uint32_t klen = (uint32_t)(sizeof LOG_KEY - 1U), i;
    memset(k, 0, sizeof k);
    if (klen > 64U) { sha256_init(&s); sha256_update(&s, LOG_KEY, klen); sha256_final(&s, k); }
    else memcpy(k, LOG_KEY, klen);

    for (i = 0; i < 64U; i++) k[i] ^= 0x36;              /* inner hash */
    sha256_init(&s); sha256_update(&s, k, 64); sha256_update(&s, prev8, 8); sha256_update(&s, row, n); sha256_final(&s, d);
    for (i = 0; i < 64U; i++) k[i] ^= (0x36 ^ 0x5c);     /* outer hash */
    sha256_init(&s); sha256_update(&s, k, 64); sha256_update(&s, d, 32); sha256_final(&s, d);
    for (i = 0; i < 4U; i++) { out8[2 * i] = hexd[d[i] >> 4]; out8[2 * i + 1] = hexd[d[i] & 15U]; }
    out8[8] = '\0';
}

/* ================================================================
 *  Real-time clock (32.768 kHz crystal on the Black Pill; keeps time on the
 *  VB coin cell) + backup registers that remember the trip across resets
 * ================================================================ */
static uint8_t rtc_bcd(uint32_t v) { return (uint8_t)(((v / 10U) << 4) | (v % 10U)); }
static uint32_t rtc_dec(uint32_t b) { return (b >> 4) * 10U + (b & 15U); }

static void rtc_unlock(void) { RTC->WPR = 0xCA; RTC->WPR = 0x53; }
static void rtc_lock(void)   { RTC->WPR = 0xFF; }

static void rtc_wait_sync(void)                         /* shadow registers in step with the clock */
{
    uint32_t until = g_ms + 50U;
    rtc_unlock();
    RTC->ISR &= ~RTC_ISR_RSF;
    rtc_lock();
    while (!(RTC->ISR & RTC_ISR_RSF) && time_before(until)) { }
}

static void rtc_init(void)
{
    uint32_t until;
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR;
    PWR->CR |= PWR_CR_DBP;                               /* allow writes to the backup domain */

    if ((RCC->BDCR & RCC_BDCR_RTCEN) && (RCC->BDCR & RCC_BDCR_LSERDY)) {   /* already running */
        g_rtc_ok = 1;
        rtc_wait_sync();
        return;
    }
    RCC->BDCR |= RCC_BDCR_LSEON;
    until = g_ms + 3000U;                                /* a 32 kHz crystal can take 2 s to start */
    while (!(RCC->BDCR & RCC_BDCR_LSERDY)) {
        if (!time_before(until)) return;                 /* no crystal: no clock, no trip memory */
    }
    if ((RCC->BDCR & RCC_BDCR_RTCSEL) != RCC_BDCR_RTCSEL_0) {
        if (RCC->BDCR & RCC_BDCR_RTCSEL) {               /* another clock was chosen before: reset domain */
            RCC->BDCR |= RCC_BDCR_BDRST;
            RCC->BDCR &= ~RCC_BDCR_BDRST;
            RCC->BDCR |= RCC_BDCR_LSEON;
            until = g_ms + 3000U;
            while (!(RCC->BDCR & RCC_BDCR_LSERDY)) { if (!time_before(until)) return; }
        }
        RCC->BDCR |= RCC_BDCR_RTCSEL_0;                  /* RTC clock = LSE */
    }
    RCC->BDCR |= RCC_BDCR_RTCEN;
    g_rtc_ok = 1;
    rtc_wait_sync();
}

static uint8_t rtc_time_set(void) { return g_rtc_ok && (RTC->ISR & RTC_ISR_INITS); }   /* calendar has been set */

static int rtc_set(uint32_t y, uint32_t mo, uint32_t d, uint32_t h, uint32_t mi, uint32_t s)
{
    static const uint8_t t[12] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };
    uint32_t yy = (mo < 3U) ? y - 1U : y, wd, until;
    if (!g_rtc_ok || y < 2000U || y > 2099U || mo < 1U || mo > 12U || d < 1U || d > 31U || h > 23U || mi > 59U || s > 59U) return -1;
    wd = (yy + yy / 4U - yy / 100U + yy / 400U + t[mo - 1U] + d) % 7U;   /* 0 = Sunday */
    if (wd == 0U) wd = 7U;                               /* RTC: 1 = Monday ... 7 = Sunday */

    rtc_unlock();
    RTC->ISR |= RTC_ISR_INIT;
    until = g_ms + 100U;
    while (!(RTC->ISR & RTC_ISR_INITF)) { if (!time_before(until)) { rtc_lock(); return -2; } }
    RTC->PRER = 255U;                                    /* 32768 / 128 / 256 = 1 Hz */
    RTC->PRER = (127U << 16) | 255U;
    RTC->TR = ((uint32_t)rtc_bcd(h) << 16) | ((uint32_t)rtc_bcd(mi) << 8) | rtc_bcd(s);
    RTC->DR = ((uint32_t)rtc_bcd(y - 2000U) << 16) | (wd << 13) | ((uint32_t)rtc_bcd(mo) << 8) | rtc_bcd(d);
    RTC->CR &= ~RTC_CR_FMT;                              /* 24-hour clock */
    RTC->ISR &= ~RTC_ISR_INIT;
    rtc_lock();
    rtc_wait_sync();
    return 0;
}

typedef struct { uint16_t y; uint8_t mo, d, h, mi, s; } dt_t;

static int rtc_get(dt_t *t)
{
    uint32_t tr, dr;
    if (!rtc_time_set()) return -1;
    tr = RTC->TR;                                        /* reading TR freezes DR until it is read */
    dr = RTC->DR;
    t->h  = (uint8_t)rtc_dec((tr >> 16) & 0x3FU);
    t->mi = (uint8_t)rtc_dec((tr >> 8) & 0x7FU);
    t->s  = (uint8_t)rtc_dec(tr & 0x7FU);
    t->y  = (uint16_t)(2000U + rtc_dec((dr >> 16) & 0xFFU));
    t->mo = (uint8_t)rtc_dec((dr >> 8) & 0x1FU);
    t->d  = (uint8_t)rtc_dec(dr & 0x3FU);
    return 0;
}

/* "2026-10-21 09:30:12", or "" when the clock is not set */
static const char *rtc_str(char *buf, int n)
{
    dt_t t;
    if (rtc_get(&t) != 0) { buf[0] = '\0'; return buf; }
    snprintf(buf, (size_t)n, "%04u-%02u-%02u %02u:%02u:%02u", t.y, t.mo, t.d, t.h, t.mi, t.s);
    return buf;
}

/* ---- Trip memory in the RTC backup registers (survives reset; power-off too with a coin cell) ---- */
#define TRIP_MAGIC 0xC01D7E05U
static void trip_save(void)
{
    volatile uint32_t *b = &RTC->BKP0R;
    uint32_t m[2];
    float f = g_vvm_used;
    double sum = g_mkt_sum;
    if (!g_rtc_ok) return;
    memcpy(m, &sum, sizeof m);
    b[1] = (g_ms - g_trip_start_ms) / 1000U;
    b[2] = g_excursion_s;
    b[3] = g_door_opens;
    b[4] = g_shock_count;
    b[5] = ((uint32_t)(uint16_t)g_min_c10) | ((uint32_t)(uint16_t)g_max_c10 << 16);
    b[6] = m[0];
    b[7] = m[1];
    b[8] = g_mkt_n;
    memcpy((void *)&m[0], &f, 4);
    b[9] = m[0];
    b[10] = (uint32_t)g_frozen | ((uint32_t)g_band_set << 1);
    b[11] = ((uint32_t)(uint16_t)g_hi_alarm) | ((uint32_t)(uint16_t)g_lo_alarm << 16);
    b[12] = ((uint32_t)(uint16_t)g_hi_warn) | ((uint32_t)(uint16_t)g_lo_warn << 16);
    b[13] = g_probe_spikes;
    b[0] = TRIP_MAGIC;
}

static int trip_restore(void)
{
    volatile uint32_t *b = &RTC->BKP0R;
    uint32_t m[2];
    double sum;
    float f;
    if (!g_rtc_ok || b[0] != TRIP_MAGIC) return 0;
    g_trip_start_ms = g_ms - b[1] * 1000U;
    g_excursion_s = b[2];
    g_door_opens  = b[3];
    g_shock_count = b[4];
    g_min_c10 = (int16_t)(b[5] & 0xFFFFU);
    g_max_c10 = (int16_t)(b[5] >> 16);
    m[0] = b[6]; m[1] = b[7];
    memcpy(&sum, m, sizeof sum);
    g_mkt_sum = sum;
    g_mkt_n = b[8];
    m[0] = b[9];
    memcpy(&f, &m[0], 4);
    g_vvm_used = f;
    g_frozen = (uint8_t)(b[10] & 1U);
    if (TEMP_PROFILE == 2 && (b[10] & 2U)) {             /* keep the demo band the trip started with */
        g_band_set = 1;
        g_hi_alarm = (int16_t)(b[11] & 0xFFFFU); g_lo_alarm = (int16_t)(b[11] >> 16);
        g_hi_warn  = (int16_t)(b[12] & 0xFFFFU); g_lo_warn  = (int16_t)(b[12] >> 16);
    }
    g_probe_spikes = b[13];
    return 1;
}

/* ================================================================
 *  Cold-chain analytics, run on every 2 s reading
 *   - Mean Kinetic Temperature (MKT): the pharma-standard single number for
 *     a varying temperature (dH/R = 10000 K, i.e. 83.144 kJ/mol)
 *   - Virtual vial monitor: Arrhenius heat budget fitted to WHO VVM points
 *     (VVM14: 14 days at 37 C, 90 days at 25 C). 100 % = fresh, 0 % = discard
 *   - Trend: least-squares slope of the last readings -> minutes to alarm
 * ================================================================ */
static int16_t g_hist_hot[PREDICT_WINDOW], g_hist_cold[PREDICT_WINDOW];
static uint8_t g_hist_n = 0, g_hist_i = 0;
static uint32_t g_pred_count = 0;

static int16_t mkt_c10(void)
{
    double tk;
    if (g_mkt_n == 0U || g_mkt_sum <= 0.0) return 0;
    tk = MKT_DH_R / (-log(g_mkt_sum / (double)g_mkt_n));
    return (int16_t)floor((tk - 273.15) * 10.0 + 0.5);
}

static int life_pct(void)
{
    float l = 100.0f * (1.0f - g_vvm_used);
    if (l < 0.0f) l = 0.0f;
    if (l > 100.0f) l = 100.0f;
    return (int)(l + 0.5f);
}

static float hist_slope(const int16_t *h)               /* C per minute */
{
    float mx = (g_hist_n - 1) / 2.0f, my = 0.0f, sxy = 0.0f, sxx = 0.0f;
    int k, i0 = (g_hist_i + PREDICT_WINDOW - g_hist_n) % PREDICT_WINDOW;
    for (k = 0; k < g_hist_n; k++) my += h[(i0 + k) % PREDICT_WINDOW];
    my /= g_hist_n;
    for (k = 0; k < g_hist_n; k++) {
        float dx = k - mx;
        sxy += dx * (h[(i0 + k) % PREDICT_WINDOW] - my);
        sxx += dx * dx;
    }
    if (sxx <= 0.0f) return 0.0f;
    return (sxy / sxx) * (60000.0f / (float)DHT_PERIOD_MS) / 10.0f;
}

static void analytics_update(void)
{
    uint8_t p = 0;
    float eta = 0.0f, sc;
    int i;

    /* Freeze: any probe at or below -0.5 C means the doses may be damaged (latched until 'z') */
    for (i = 0; i < g_probe_n; i++) {
        if (g_probe_ok[i] && g_probe_c10[i] <= FREEZE_C10 && !g_frozen) {
            g_frozen = 1;
            uart_printf("[%lu s] !!! P%d reached %d.%d C - vaccine may be FROZEN (shake test needed) !!!\r\n",
                        (unsigned long)(g_ms / 1000U), i + 1, g_probe_c10[i] / 10, (g_probe_c10[i] < 0 ? -g_probe_c10[i] : g_probe_c10[i]) % 10);
            log_event("FROZEN");
        }
    }
    /* Probe cross-check: two probes in one box should not be far apart */
    {
        uint8_t f = (uint8_t)(g_probe_n >= 2 && g_probe_ok[0] && g_probe_ok[1] &&
                              (g_probe_c10[0] - g_probe_c10[1] > PROBE_DIFF_C10 || g_probe_c10[1] - g_probe_c10[0] > PROBE_DIFF_C10));
        if (f && !g_probe_fault) {
            uart_printf("[%lu s] Probes disagree by more than %d C - check that both are in the box\r\n",
                        (unsigned long)(g_ms / 1000U), PROBE_DIFF_C10 / 10);
            log_event("PROBES DISAGREE");
        }
        g_probe_fault = f;
    }

    if (!g_temp_valid) { g_hist_n = 0; g_pred_active = 0; return; }

    /* MKT and heat budget use the warmest reading (worst case) */
    {
        double tk = g_temp_c10 / 10.0 + 273.15;
        double per_day = exp(VVM_EA_R * (1.0 / 310.15 - 1.0 / tk)) / (double)VVM_DAYS_AT_37;
        double dt_days = ((double)DHT_PERIOD_MS / 1000.0) / 86400.0 * (TEMP_PROFILE == 2 ? VVM_DEMO_SPEEDUP : 1.0);
        int before = life_pct();
        g_mkt_sum += exp(-MKT_DH_R / tk);
        g_mkt_n++;
        g_vvm_used += (float)(per_day * dt_days);
        if (before > 25 && life_pct() <= 25) { uart_puts("Heat budget below 25 % - use these doses first\r\n"); log_event("LIFE LOW"); }
        if (before > 0 && life_pct() == 0)   { uart_puts("!!! Heat budget used up - VVM discard point reached !!!\r\n"); log_event("VVM DISCARD"); }
    }

    /* Trend and prediction */
    g_hist_hot[g_hist_i] = g_temp_c10;
    g_hist_cold[g_hist_i] = g_tmin_c10;
    g_hist_i = (uint8_t)((g_hist_i + 1U) % PREDICT_WINDOW);
    if (g_hist_n < PREDICT_WINDOW) g_hist_n++;
    if (g_hist_n < PREDICT_WINDOW) { g_pred_active = 0; g_slope_cpm = 0.0f; return; }

    g_slope_cpm = hist_slope(g_hist_hot);
    sc = hist_slope(g_hist_cold);
    if (g_slope_cpm > PREDICT_MIN_SLOPE && g_temp_c10 < g_hi_alarm) {
        eta = (g_hi_alarm - g_temp_c10) / 10.0f / g_slope_cpm;
        if (eta < PREDICT_WARN_MIN) p = 1;
    }
    if (!p && sc < -PREDICT_MIN_SLOPE && g_tmin_c10 >= g_lo_alarm) {
        eta = (g_tmin_c10 - g_lo_alarm) / 10.0f / -sc;
        if (eta < PREDICT_WARN_MIN) p = 2;
    }
    if (p && p != g_pred_active) {
        char ev[24];
        g_pred_count++;
        uart_printf("[%lu s] PREDICTION: too %s in about %d.%d min (%s %d.%d C/min)\r\n",
                    (unsigned long)(g_ms / 1000U), p == 1 ? "warm" : "cold", (int)eta, (int)(eta * 10.0f) % 10,
                    p == 1 ? "rising" : "falling", (int)fabsf(p == 1 ? g_slope_cpm : sc), (int)(fabsf(p == 1 ? g_slope_cpm : sc) * 10.0f) % 10);
        snprintf(ev, sizeof ev, "PREDICT %s %d MIN", p == 1 ? "WARM" : "COLD", (int)(eta + 0.5f));
        log_event(ev);
    }
    g_pred_active = p;
    g_pred_min = eta;
}

/* ================================================================
 *  Micro SD card on SPI1: PA4 = CS, PA5 = SCK, PA6 = MISO, PA7 = MOSI (AF5)
 *  The module's VCC goes to 5V (it has its own 3.3V regulator + level shifter).
 * ================================================================ */
#define SD_CS     4          /* PA4 */
#define CT_MMC    0x01
#define CT_SD1    0x02
#define CT_SD2    0x04
#define CT_BLOCK  0x08       /* SDHC/SDXC: addressed in 512-byte blocks, not bytes */
#define ACMD41    (0x80U | 41U)

static void spi1_init(uint32_t br)
{
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    (void)RCC->APB2ENR;

    PIN_HIGH(GPIOA, SD_CS);                              /* card not selected */
    pin_mode(GPIOA, SD_CS, 1);
    pin_mode(GPIOA, 5, 2);
    pin_mode(GPIOA, 6, 2);
    pin_mode(GPIOA, 7, 2);
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~((0xFU << 20) | (0xFU << 24) | (0xFU << 28)))
                  | (5U << 20) | (5U << 24) | (5U << 28);          /* AF5 = SPI1 */
    pin_pull(GPIOA, 6, 1);                               /* MISO reads 0xFF when nothing answers */
    GPIOA->OSPEEDR |= (1U << (4 * 2)) | (1U << (5 * 2)) | (1U << (7 * 2));

    SPI1->CR1 = 0;
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | ((br & 7U) << 3);   /* mode 0, 8-bit, MSB first */
    SPI1->CR1 |= SPI_CR1_SPE;
}

static void spi1_speed(uint32_t br)
{
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 = (SPI1->CR1 & ~(7U << 3)) | ((br & 7U) << 3);
    SPI1->CR1 |= SPI_CR1_SPE;
}

static uint8_t spi_xfer(uint8_t b)
{
    uint32_t n = 0;
    while (!(SPI1->SR & SPI_SR_TXE))  { if (++n > 200000U) return 0xFF; }
    *(volatile uint8_t *)&SPI1->DR = b;
    n = 0;
    while (!(SPI1->SR & SPI_SR_RXNE)) { if (++n > 200000U) return 0xFF; }
    return *(volatile uint8_t *)&SPI1->DR;
}

static int sd_wait_ready(uint32_t ms)                    /* the card sends 0xFF when it is not busy */
{
    uint32_t until = g_ms + ms;
    do {
        if (spi_xfer(0xFF) == 0xFF) return 1;
    } while (time_before(until));
    return 0;
}

static void sd_deselect(void)
{
    PIN_HIGH(GPIOA, SD_CS);
    spi_xfer(0xFF);                                      /* one extra clock releases MISO */
}

static int sd_select(void)
{
    PIN_LOW(GPIOA, SD_CS);
    spi_xfer(0xFF);
    if (sd_wait_ready(500)) return 1;
    sd_deselect();
    return 0;
}

/* Send a command, return the R1 response (0 = OK, 1 = idle, 0xFF = no answer) */
static uint8_t sd_cmd(uint8_t cmd, uint32_t arg)
{
    uint8_t r, crc = 0x01;
    int i;

    if (cmd & 0x80U) {                                   /* ACMDxx = CMD55 + CMDxx */
        cmd &= 0x7FU;
        r = sd_cmd(55, 0);
        if (r > 1U) return r;
    }
    sd_deselect();
    if (!sd_select()) return 0xFF;

    spi_xfer((uint8_t)(0x40U | cmd));
    spi_xfer((uint8_t)(arg >> 24));
    spi_xfer((uint8_t)(arg >> 16));
    spi_xfer((uint8_t)(arg >> 8));
    spi_xfer((uint8_t)arg);
    if (cmd == 0U) crc = 0x95;                           /* only CMD0 and CMD8 need a real CRC */
    if (cmd == 8U) crc = 0x87;
    spi_xfer(crc);

    for (i = 0, r = 0xFF; i < 10; i++) {
        r = spi_xfer(0xFF);
        if (!(r & 0x80U)) break;
    }
    return r;
}

static uint8_t g_sd_type = 0;

/* Wake the card up in SPI mode. 0 = ready, -1 = no card / no answer */
static int sd_init(void)
{
    uint8_t ocr[4], ty = 0, cmd;
    uint32_t until;
    int i, tries;

    spi1_init(7);                                        /* 84 MHz / 256 = 328 kHz for the wake-up */
    for (tries = 0; tries < 3 && ty == 0U; tries++) {
        PIN_HIGH(GPIOA, SD_CS);
        for (i = 0; i < 10; i++) spi_xfer(0xFF);         /* 80 clocks with CS high */

        if (sd_cmd(0, 0) == 1U) {                        /* CMD0: go idle, enter SPI mode */
            until = g_ms + 1500U;
            if (sd_cmd(8, 0x1AA) == 1U) {                /* SD v2 card */
                for (i = 0; i < 4; i++) ocr[i] = spi_xfer(0xFF);
                if (ocr[2] == 0x01U && ocr[3] == 0xAAU) {
                    while (time_before(until) && sd_cmd(ACMD41, 1UL << 30)) { }
                    if (time_before(until) && sd_cmd(58, 0) == 0U) {
                        for (i = 0; i < 4; i++) ocr[i] = spi_xfer(0xFF);
                        ty = (ocr[0] & 0x40U) ? (CT_SD2 | CT_BLOCK) : CT_SD2;
                    }
                }
            } else {                                     /* old SD v1 or MMC */
                if (sd_cmd(ACMD41, 0) <= 1U) { ty = CT_SD1; cmd = ACMD41; }
                else                          { ty = CT_MMC; cmd = 1; }
                while (time_before(until) && sd_cmd(cmd, 0)) { }
                if (!time_before(until)) ty = 0;
            }
            if (ty != 0U && !(ty & CT_BLOCK) && sd_cmd(16, 512) != 0U) ty = 0;   /* 512-byte blocks */
        }
        sd_deselect();
    }
    g_sd_type = ty;
    if (ty == 0U) return -1;
    spi1_speed(SD_SPI_FAST_BR);
    return 0;
}

static int sd_read_block(uint32_t lba, uint8_t *buf)
{
    uint32_t until;
    uint8_t tok;
    int i;
    if (!(g_sd_type & CT_BLOCK)) lba *= 512U;            /* old cards use byte addresses */
    if (sd_cmd(17, lba) != 0U) { sd_deselect(); return -1; }
    until = g_ms + 200U;
    do { tok = spi_xfer(0xFF); } while (tok == 0xFF && time_before(until));
    if (tok != 0xFE) { sd_deselect(); return -2; }
    for (i = 0; i < 512; i++) buf[i] = spi_xfer(0xFF);
    spi_xfer(0xFF); spi_xfer(0xFF);                      /* CRC, ignored */
    sd_deselect();
    return 0;
}

static int sd_write_block(uint32_t lba, const uint8_t *buf)
{
    uint8_t resp;
    int i;
    if (!(g_sd_type & CT_BLOCK)) lba *= 512U;
    if (sd_cmd(24, lba) != 0U) { sd_deselect(); return -1; }
    spi_xfer(0xFF);
    spi_xfer(0xFE);                                      /* start token */
    for (i = 0; i < 512; i++) spi_xfer(buf[i]);
    spi_xfer(0xFF); spi_xfer(0xFF);                      /* dummy CRC */
    resp = spi_xfer(0xFF);
    if ((resp & 0x1FU) != 0x05U) { sd_deselect(); return -2; }   /* data rejected */
    if (!sd_wait_ready(600)) { sd_deselect(); return -3; }       /* wait until it is really written */
    sd_deselect();
    return 0;
}

static int blk_read(uint32_t lba, uint8_t *buf)        { return sd_read_block(lba, buf); }
static int blk_write(uint32_t lba, const uint8_t *buf) { return sd_write_block(lba, buf); }

/* ==== FAT LOGGER BEGIN (tested on a PC against FAT16/FAT32 disk images) ====
 * Tiny FAT16/FAT32 writer: finds or creates TRIPLOG.CSV in the root folder and
 * appends text to it. It only needs blk_read()/blk_write() for 512-byte sectors.
 * Every sector written is read back and compared (fat_verify_err counts mismatches).
 */
#define LOG_NAME83 "TRIPLOG CSV"                /* 8.3 name of TRIPLOG.CSV */
static uint16_t fat_date_now = 0, fat_time_now = 0;  /* set from the real-time clock when it is running */

typedef struct {
    uint8_t  fat32;            /* 1 = FAT32, 0 = FAT16 */
    uint8_t  spc;              /* sectors per cluster */
    uint8_t  nfats;
    uint8_t  fsinfo_done;      /* FSInfo free count already marked "unknown" */
    uint32_t vol_lba, vol_secs, fat_lba, fat_sz, root_lba, root_secs, data_lba;
    uint32_t root_clus, nclus, fsinfo_lba, free_hint;
    /* the open log file */
    uint32_t dir_lba;
    uint16_t dir_off;
    uint8_t  tail_dirty;
    uint32_t first_clus, cur_clus, prev_clus, size, tail_lba;
} fat_t;

static fat_t    F;
static uint8_t  fat_tail[512];                  /* RAM copy of the file's last sector */
static uint8_t  fat_tmp[512];                   /* directory / boot sector work buffer */
static uint8_t  fat_vbuf[512];                  /* read-back buffer for verifying writes */
static uint8_t  fat_fbuf[512];                  /* one cached FAT sector */
static uint32_t fat_fbuf_lba = 0xFFFFFFFFU;
static uint32_t fat_verify_err = 0;
static uint32_t fat_sectors_written = 0;

static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t rd32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static void wr16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void wr32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }

/* Write one sector, read it back, compare. 0 ok, -1 write failed, -2 read-back differs */
static int fat_write_verify(uint32_t lba, const uint8_t *buf)
{
    if (blk_write(lba, buf) != 0) return -1;
    fat_sectors_written++;
    if (blk_read(lba, fat_vbuf) != 0 || memcmp(fat_vbuf, buf, 512) != 0) {
        fat_verify_err++;
        return -2;
    }
    return 0;
}

static uint32_t fat_clus_lba(uint32_t c) { return F.data_lba + (c - 2U) * F.spc; }
static uint32_t fat_eoc(void)            { return F.fat32 ? 0x0FFFFFFFU : 0xFFFFU; }
static int      fat_is_eoc(uint32_t v)   { return F.fat32 ? (v >= 0x0FFFFFF8U) : (v >= 0xFFF8U); }

static int fat_load(uint32_t lba)
{
    if (fat_fbuf_lba == lba) return 0;
    fat_fbuf_lba = 0xFFFFFFFFU;
    if (blk_read(lba, fat_fbuf) != 0) return -1;
    fat_fbuf_lba = lba;
    return 0;
}

static int fat_get(uint32_t c, uint32_t *v)
{
    uint32_t off = F.fat32 ? c * 4U : c * 2U;
    if (fat_load(F.fat_lba + off / 512U) != 0) return -1;
    *v = F.fat32 ? (rd32(&fat_fbuf[off % 512U]) & 0x0FFFFFFFU) : rd16(&fat_fbuf[off % 512U]);
    return 0;
}

static int fat_set(uint32_t c, uint32_t v)
{
    uint32_t off = F.fat32 ? c * 4U : c * 2U;
    uint32_t sec = off / 512U;
    uint8_t i;
    if (fat_load(F.fat_lba + sec) != 0) return -1;
    if (F.fat32) wr32(&fat_fbuf[off % 512U], (rd32(&fat_fbuf[off % 512U]) & 0xF0000000U) | (v & 0x0FFFFFFFU));
    else         wr16(&fat_fbuf[off % 512U], (uint16_t)v);
    for (i = 0; i < F.nfats; i++) {                         /* keep every FAT copy identical */
        if (fat_write_verify(F.fat_lba + sec + (uint32_t)i * F.fat_sz, fat_fbuf) != 0) {
            fat_fbuf_lba = 0xFFFFFFFFU;
            return -1;
        }
    }
    return 0;
}

/* FAT32 keeps a "free clusters" hint; mark it unknown so Windows/Linux recount it */
static void fat_fsinfo_invalidate(void)
{
    if (!F.fat32 || F.fsinfo_done || F.fsinfo_lba == 0) return;
    F.fsinfo_done = 1;
    if (blk_read(F.fsinfo_lba, fat_tmp) != 0) return;
    if (rd32(fat_tmp) != 0x41615252U || rd32(fat_tmp + 484) != 0x61417272U) return;
    wr32(fat_tmp + 488, 0xFFFFFFFFU);
    (void)fat_write_verify(F.fsinfo_lba, fat_tmp);
}

/* Grab a free cluster, mark it end-of-chain and link it after 'prev' (0 = none) */
static int fat_alloc(uint32_t prev, uint32_t *out)
{
    uint32_t c = F.free_hint, n, v;
    for (n = 0; n < F.nclus; n++, c++) {
        if (c < 2U || c > F.nclus + 1U) c = 2U;
        if (fat_get(c, &v) != 0) return -1;
        if (v == 0U) {
            fat_fsinfo_invalidate();
            if (fat_set(c, fat_eoc()) != 0) return -1;
            if (prev != 0U && fat_set(prev, c) != 0) return -1;
            F.free_hint = c + 1U;
            *out = c;
            return 0;
        }
    }
    return -2;                                              /* card full */
}

static int fat_is_vbr(const uint8_t *b)
{
    uint8_t spc = b[13];
    return (b[0] == 0xEB || b[0] == 0xE9) && rd16(b + 11) == 512U && spc != 0U && (spc & (spc - 1U)) == 0U &&
           rd16(b + 14) != 0U && (b[16] == 1U || b[16] == 2U);
}

/* 0 ok; -1 read error, -2 no FAT signature, -3 no FAT partition, -4 not 512-byte sectors, -5 FAT12/exFAT */
static int fat_mount(void)
{
    uint32_t part = 0, tot, rsvd, data_secs;
    memset(&F, 0, sizeof F);
    fat_fbuf_lba = 0xFFFFFFFFU;

    if (blk_read(0, fat_tmp) != 0) return -1;
    if (rd16(fat_tmp + 510) != 0xAA55U) return -2;
    if (!fat_is_vbr(fat_tmp)) {                             /* sector 0 is a partition table (MBR) */
        int i;
        for (i = 0; i < 4 && part == 0U; i++) {
            const uint8_t *pe = fat_tmp + 446 + i * 16;
            uint8_t type = pe[4];
            if (type == 0x01 || type == 0x04 || type == 0x06 || type == 0x0B || type == 0x0C || type == 0x0E)
                part = rd32(pe + 8);
            else if (type == 0x07) return -5;               /* exFAT / NTFS */
        }
        if (part == 0U) return -3;
        if (blk_read(part, fat_tmp) != 0) return -1;
        if (rd16(fat_tmp + 510) != 0xAA55U) return -2;
        if (!fat_is_vbr(fat_tmp)) return (memcmp(fat_tmp + 3, "EXFAT", 5) == 0) ? -5 : -3;
    }
    if (rd16(fat_tmp + 11) != 512U) return -4;

    F.vol_lba = part;
    F.spc     = fat_tmp[13];
    rsvd      = rd16(fat_tmp + 14);
    F.nfats   = fat_tmp[16];
    F.root_secs = ((uint32_t)rd16(fat_tmp + 17) * 32U + 511U) / 512U;
    tot       = rd16(fat_tmp + 19) ? rd16(fat_tmp + 19) : rd32(fat_tmp + 32);
    F.fat_sz  = rd16(fat_tmp + 22) ? rd16(fat_tmp + 22) : rd32(fat_tmp + 36);
    F.vol_secs = tot;
    F.fat_lba = part + rsvd;
    data_secs = tot - (rsvd + F.nfats * F.fat_sz + F.root_secs);
    F.nclus   = data_secs / F.spc;
    F.fat32   = (uint8_t)(rd16(fat_tmp + 22) == 0U);        /* FAT32 has no 16-bit FAT size */
    if (!F.fat32 && F.nclus < 4085U) return -5;             /* FAT12: not supported */
    F.root_lba = F.fat_lba + F.nfats * F.fat_sz;           /* FAT16 fixed root folder */
    F.data_lba = F.root_lba + F.root_secs;
    F.free_hint = 2U;
    if (F.fat32) {
        F.root_clus = rd32(fat_tmp + 44);
        if (rd16(fat_tmp + 48) != 0U && rd16(fat_tmp + 48) != 0xFFFFU) {
            F.fsinfo_lba = part + rd16(fat_tmp + 48);
            if (blk_read(F.fsinfo_lba, fat_tmp) == 0 && rd32(fat_tmp) == 0x41615252U) {
                uint32_t h = rd32(fat_tmp + 492);
                if (h >= 2U && h <= F.nclus + 1U) F.free_hint = h;
            }
        }
    }
    return 0;
}

/* Today's date for the file entry: there is no clock yet, so use the firmware build date */
static uint16_t fat_build_date(void)
{
    static const char mon[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
    const char *d = __DATE__;                               /* "Oct 19 2026" */
    int m = 1, i, day, year;
    for (i = 0; i < 12; i++) if (memcmp(d, mon + i * 3, 3) == 0) m = i + 1;
    day  = (d[4] == ' ' ? 0 : d[4] - '0') * 10 + (d[5] - '0');
    year = (d[7] - '0') * 1000 + (d[8] - '0') * 100 + (d[9] - '0') * 10 + (d[10] - '0');
    if (year < 1980) year = 1980;
    return (uint16_t)(((year - 1980) << 9) | (m << 5) | day);
}
static uint16_t fat_build_time(void)
{
    const char *t = __TIME__;                               /* "18:54:07" */
    int h = (t[0] - '0') * 10 + (t[1] - '0'), mi = (t[3] - '0') * 10 + (t[4] - '0'), s = (t[6] - '0') * 10 + (t[7] - '0');
    return (uint16_t)((h << 11) | (mi << 5) | (s / 2));
}

/* Find the log file in the root folder, or create it.
   Returns 1 found, 0 created, -1 read/write error, -6 root folder full */
static int fat_find_or_create(void)
{
    char name[11];
    uint32_t clus = F.root_clus, base, nsec, s, lba, free_lba = 0;
    uint16_t off, free_off = 0xFFFFU;
    int end = 0, i;

    memcpy(name, LOG_NAME83, 11);
    base = F.fat32 ? fat_clus_lba(clus) : F.root_lba;
    nsec = F.fat32 ? F.spc : F.root_secs;
    for (i = 0; i < 100000 && !end; i++) {
        for (s = 0; s < nsec && !end; s++) {
            lba = base + s;
            if (blk_read(lba, fat_tmp) != 0) return -1;
            for (off = 0; off < 512U; off += 32U) {
                const uint8_t *e = fat_tmp + off;
                if (e[0] == 0x00) {                        /* end of the folder */
                    if (free_off == 0xFFFFU) { free_lba = lba; free_off = off; }
                    end = 1;
                    break;
                }
                if (e[0] == 0xE5) {                        /* deleted entry: reusable */
                    if (free_off == 0xFFFFU) { free_lba = lba; free_off = off; }
                    continue;
                }
                if (e[11] == 0x0F || (e[11] & 0x18)) continue;   /* long name, label, folder */
                if (memcmp(e, name, 11) == 0) {
                    F.dir_lba = lba;
                    F.dir_off = off;
                    F.first_clus = rd16(e + 26) | (F.fat32 ? ((uint32_t)rd16(e + 20) << 16) : 0U);
                    F.size = rd32(e + 28);
                    return 1;
                }
            }
        }
        if (end || !F.fat32) break;
        {
            uint32_t v;
            if (fat_get(clus, &v) != 0) return -1;
            if (fat_is_eoc(v) || v < 2U) break;
            clus = v;
            base = fat_clus_lba(clus);
        }
    }
    if (free_off == 0xFFFFU) return -6;

    if (blk_read(free_lba, fat_tmp) != 0) return -1;
    {
        uint8_t *e = fat_tmp + free_off;
        uint16_t d = fat_date_now ? fat_date_now : fat_build_date();
        uint16_t t = fat_date_now ? fat_time_now : fat_build_time();
        memset(e, 0, 32);
        memcpy(e, name, 11);
        e[11] = 0x20;                                       /* archive */
        wr16(e + 14, t); wr16(e + 16, d);                   /* created */
        wr16(e + 18, d);                                    /* last access */
        wr16(e + 22, t); wr16(e + 24, d);                   /* modified */
    }
    if (fat_write_verify(free_lba, fat_tmp) != 0) return -1;
    F.dir_lba = free_lba;
    F.dir_off = free_off;
    F.first_clus = 0;
    F.size = 0;
    return 0;
}

/* Open TRIPLOG.CSV for appending. Returns 1 existing, 0 new, <0 error (-7 broken chain) */
static int fat_open_log(void)
{
    int r = fat_find_or_create();
    uint32_t cbytes = (uint32_t)F.spc * 512U, k, v;
    if (r < 0) return r;
    F.cur_clus = F.first_clus;
    F.prev_clus = 0;
    F.tail_dirty = 0;
    F.tail_lba = 0;
    if (F.first_clus != 0U && F.size > 0U) {
        for (k = (F.size - 1U) / cbytes; k > 0U; k--) {      /* walk to the cluster holding the last byte */
            if (fat_get(F.cur_clus, &v) != 0) return -1;
            if (v < 2U || fat_is_eoc(v) || v > F.nclus + 1U) return -7;
            F.prev_clus = F.cur_clus;
            F.cur_clus = v;
        }
        F.tail_lba = fat_clus_lba(F.cur_clus) + ((F.size - 1U) / 512U) % F.spc;
        if ((F.size % 512U) != 0U && blk_read(F.tail_lba, fat_tail) != 0) return -1;
    } else if (F.first_clus == 0U) {
        F.size = 0;
    }
    return r;
}

/* Add bytes to the end of the file (in RAM; full sectors go to the card straight away) */
static int fat_append(const char *s, uint32_t n)
{
    while (n > 0U) {
        uint32_t off = F.size % 512U, k;
        if (off == 0U) {                                    /* starting a new sector */
            uint32_t sec_in_clus = (F.size / 512U) % F.spc;
            if (F.cur_clus == 0U) {
                uint32_t c;
                if (fat_alloc(0, &c) != 0) return -2;
                F.first_clus = F.cur_clus = c;
            } else if (sec_in_clus == 0U && F.size > 0U) {
                uint32_t c;
                if (fat_get(F.cur_clus, &c) != 0) return -1;
                if (c < 2U || c > F.nclus + 1U || fat_is_eoc(c)) {   /* no next cluster yet: take a free one */
                    if (fat_alloc(F.cur_clus, &c) != 0) return -2;
                }
                F.prev_clus = F.cur_clus;
                F.cur_clus = c;
            }
            F.tail_lba = fat_clus_lba(F.cur_clus) + sec_in_clus;
            memset(fat_tail, 0, sizeof fat_tail);
        }
        k = 512U - off;
        if (k > n) k = n;
        memcpy(fat_tail + off, s, k);
        F.size += k; s += k; n -= k;
        F.tail_dirty = 1;
        if ((F.size % 512U) == 0U) {                        /* sector full: write it now */
            if (fat_write_verify(F.tail_lba, fat_tail) != 0) return -1;
            F.tail_dirty = 0;
        }
    }
    return 0;
}

/* Put everything on the card: the last partial sector and the file size in the folder entry */
static int fat_flush(void)
{
    uint8_t *e;
    if (F.tail_dirty) {
        if (fat_write_verify(F.tail_lba, fat_tail) != 0) return -1;
        F.tail_dirty = 0;
    }
    if (blk_read(F.dir_lba, fat_tmp) != 0) return -1;
    e = fat_tmp + F.dir_off;
    if (rd32(e + 28) == F.size && (rd16(e + 26) | ((uint32_t)rd16(e + 20) << 16)) == F.first_clus) return 0;
    wr16(e + 26, (uint16_t)(F.first_clus & 0xFFFFU));
    if (F.fat32) wr16(e + 20, (uint16_t)(F.first_clus >> 16));
    wr32(e + 28, F.size);
    if (fat_date_now) { wr16(e + 22, fat_time_now); wr16(e + 24, fat_date_now); wr16(e + 18, fat_date_now); }
    e[11] |= 0x20;
    return fat_write_verify(F.dir_lba, fat_tmp) == 0 ? 0 : -1;
}

/* Read the end of the file back FROM THE CARD (not from RAM): the sector before the last
   (if any) into prev, and the last sector into cur. Byte counts go to *pn and *cn. */
static int fat_read_tail(uint8_t *prev, uint32_t *pn, uint8_t *cur, uint32_t *cn)
{
    uint32_t sec_in_clus;
    *pn = 0; *cn = 0;
    if (F.size == 0U || F.tail_lba == 0U) return 0;
    if (blk_read(F.tail_lba, cur) != 0) return -1;
    *cn = (F.size % 512U) ? (F.size % 512U) : 512U;
    if (F.size > *cn) {
        uint32_t plba = 0;
        sec_in_clus = ((F.size - 1U) / 512U) % F.spc;
        if (sec_in_clus > 0U) plba = F.tail_lba - 1U;
        else if (F.prev_clus != 0U) plba = fat_clus_lba(F.prev_clus) + F.spc - 1U;
        if (plba != 0U) {
            if (blk_read(plba, prev) != 0) return -1;
            *pn = 512U;
        }
    }
    return 0;
}
/* ==== FAT LOGGER END ==== */

/* ================================================================
 *  I2C1 on PB6 (SCL) / PB7 (SDA)  -- shared by the OLED and MPU6050
 * ================================================================ */
#define I2C_TIMEOUT_US 3000U

/* If a slave was reset mid-transfer it can hold SDA low. Clock it free, then send STOP. */
static void i2c_bus_clear(void)
{
    int i;
    GPIOB->OTYPER |= (1U << I2C_SCL) | (1U << I2C_SDA);
    pin_pull(GPIOB, I2C_SCL, 1);
    pin_pull(GPIOB, I2C_SDA, 1);
    PIN_HIGH(GPIOB, I2C_SCL);
    PIN_HIGH(GPIOB, I2C_SDA);
    pin_mode(GPIOB, I2C_SCL, 1);
    pin_mode(GPIOB, I2C_SDA, 1);
    delay_us(10);

    for (i = 0; i < 9 && !PIN_READ(GPIOB, I2C_SDA); i++) {
        PIN_LOW(GPIOB, I2C_SCL);  delay_us(5);
        PIN_HIGH(GPIOB, I2C_SCL); delay_us(5);
    }
    PIN_LOW(GPIOB, I2C_SCL);  delay_us(5);     /* STOP: SDA rises while SCL is high */
    PIN_LOW(GPIOB, I2C_SDA);  delay_us(5);
    PIN_HIGH(GPIOB, I2C_SCL); delay_us(5);
    PIN_HIGH(GPIOB, I2C_SDA); delay_us(5);
}

static void i2c_config(void)
{
    uint32_t mhz = g_pclk1 / 1000000U;

    I2C1->CR1 = I2C_CR1_SWRST;                  /* reset the peripheral */
    I2C1->CR1 = 0;
    I2C1->CR2 = mhz;                             /* APB1 clock in MHz */
#if I2C_FAST
    I2C1->CCR   = I2C_CCR_FS | ((g_pclk1 + 1199999U) / 1200000U);   /* 400 kHz, duty 2:1 */
    I2C1->TRISE = (mhz * 300U) / 1000U + 1U;                         /* 300 ns max rise  */
#else
    I2C1->CCR   = g_pclk1 / (2U * 100000U);                          /* 100 kHz          */
    I2C1->TRISE = mhz + 1U;                                          /* 1000 ns max rise */
#endif
    I2C1->CR1 = I2C_CR1_PE;
}

static void i2c_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    (void)RCC->APB1ENR;

    i2c_bus_clear();

    GPIOB->OSPEEDR |= (3U << (I2C_SCL * 2U)) | (3U << (I2C_SDA * 2U));
    GPIOB->AFR[I2C_SCL >> 3] = (GPIOB->AFR[I2C_SCL >> 3] & ~(0xFU << ((I2C_SCL & 7U) * 4U))) | (4U << ((I2C_SCL & 7U) * 4U));  /* AF4 */
    GPIOB->AFR[I2C_SDA >> 3] = (GPIOB->AFR[I2C_SDA >> 3] & ~(0xFU << ((I2C_SDA & 7U) * 4U))) | (4U << ((I2C_SDA & 7U) * 4U));
    pin_mode(GPIOB, I2C_SCL, 2);
    pin_mode(GPIOB, I2C_SDA, 2);

    i2c_config();
}

/* Wait for an SR1 flag. Returns 0 ok, -1 timeout, -2 no ACK from the device. */
static int i2c_wait(uint32_t flag)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t limit = I2C_TIMEOUT_US * g_cyc_per_us;
    while (!(I2C1->SR1 & flag)) {
        if (I2C1->SR1 & I2C_SR1_AF) {
            I2C1->SR1 &= ~I2C_SR1_AF;
            return -2;
        }
        if ((DWT->CYCCNT - start) > limit) return -1;
    }
    return 0;
}

static int i2c_fail(int r)
{
    I2C1->CR1 |= I2C_CR1_STOP;
    g_i2c_err++;
    if (r == -1) i2c_config();                  /* timeout: reset the peripheral */
    return r;
}

/* START + address. ADDR is left set; the caller clears it by reading SR2. */
static int i2c_begin(uint8_t addr, uint8_t rd)
{
    uint32_t start = DWT->CYCCNT;
    int r;
    while (I2C1->CR1 & I2C_CR1_STOP) {          /* previous STOP still going out */
        if ((DWT->CYCCNT - start) > I2C_TIMEOUT_US * g_cyc_per_us) return -1;
    }
    I2C1->CR1 |= I2C_CR1_START;
    if ((r = i2c_wait(I2C_SR1_SB)) != 0) return r;
    I2C1->DR = (uint8_t)((addr << 1) | rd);
    return i2c_wait(I2C_SR1_ADDR);
}

/* Write: first byte (register or control byte), then len bytes from data */
static int i2c_write(uint8_t addr, uint8_t first, const uint8_t *data, uint32_t len)
{
    int r;
    if ((r = i2c_begin(addr, 0)) != 0) return i2c_fail(r);
    (void)I2C1->SR2;
    if ((r = i2c_wait(I2C_SR1_TXE)) != 0) return i2c_fail(r);
    I2C1->DR = first;
    while (len--) {
        if ((r = i2c_wait(I2C_SR1_TXE)) != 0) return i2c_fail(r);
        I2C1->DR = *data++;
    }
    if ((r = i2c_wait(I2C_SR1_BTF)) != 0) return i2c_fail(r);
    I2C1->CR1 |= I2C_CR1_STOP;
    return 0;
}

/* Read n bytes starting at register reg */
static int i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *buf, uint32_t n)
{
    int r;
    if ((r = i2c_begin(addr, 0)) != 0) return i2c_fail(r);
    (void)I2C1->SR2;
    if ((r = i2c_wait(I2C_SR1_TXE)) != 0) return i2c_fail(r);
    I2C1->DR = reg;
    if ((r = i2c_wait(I2C_SR1_BTF)) != 0) return i2c_fail(r);

    if ((r = i2c_begin(addr, 1)) != 0) return i2c_fail(r);   /* repeated START */
    if (n == 1) {
        I2C1->CR1 &= ~I2C_CR1_ACK;              /* NACK the only byte */
        (void)I2C1->SR2;
        I2C1->CR1 |= I2C_CR1_STOP;
    } else {
        I2C1->CR1 |= I2C_CR1_ACK;
        (void)I2C1->SR2;
    }
    while (n) {
        if (n == 1 && (I2C1->CR1 & I2C_CR1_ACK)) {
            I2C1->CR1 &= ~I2C_CR1_ACK;          /* NACK the last byte, then STOP */
            I2C1->CR1 |= I2C_CR1_STOP;
        }
        if ((r = i2c_wait(I2C_SR1_RXNE)) != 0) return i2c_fail(r);
        *buf++ = (uint8_t)I2C1->DR;
        n--;
    }
    return 0;
}

/* Returns 1 if a device ACKs at addr, 0 if nobody answers, -1 if the bus is stuck (no START) */
static int i2c_probe(uint8_t addr)
{
    int r = i2c_begin(addr, 0);
    if (r == 0) {
        (void)I2C1->SR2;
        I2C1->CR1 |= I2C_CR1_STOP;
        return 1;
    }
    I2C1->CR1 |= I2C_CR1_STOP;
    if (r == -1) { i2c_config(); return -1; }
    return 0;
}

/* Wire check: with PB6/PB7 switched to the internal pull-DOWN, only a module's own
   pull-up resistor can hold a line HIGH. 1 = the wire reaches a powered module. */
static void i2c_wire_check(uint32_t *scl_ok, uint32_t *sda_ok)
{
    pin_mode(GPIOB, I2C_SCL, 0);
    pin_mode(GPIOB, I2C_SDA, 0);
    pin_pull(GPIOB, I2C_SCL, 2);
    pin_pull(GPIOB, I2C_SDA, 2);
    delay_us(200);
    *scl_ok = PIN_READ(GPIOB, I2C_SCL);
    *sda_ok = PIN_READ(GPIOB, I2C_SDA);
    pin_pull(GPIOB, I2C_SCL, 1);
    pin_pull(GPIOB, I2C_SDA, 1);
    pin_mode(GPIOB, I2C_SCL, 2);
    pin_mode(GPIOB, I2C_SDA, 2);
    delay_us(50);
    i2c_config();
}

static void i2c_scan(void)
{
    uint8_t a;
    int found = 0, stuck = 0, r;
    uint32_t scl = PIN_READ(GPIOB, I2C_SCL), sda = PIN_READ(GPIOB, I2C_SDA);

    uint32_t scl_mod = 0, sda_mod = 0;
    uart_printf("I2C lines: SCL(PB%u)=%lu SDA(PB%u)=%lu  (both should be 1)\r\n",
                (unsigned)I2C_SCL, (unsigned long)scl, (unsigned)I2C_SDA, (unsigned long)sda);
    i2c_wire_check(&scl_mod, &sda_mod);
    uart_printf("Wire check: PB%u(SCL)->module %s, PB%u(SDA)->module %s\r\n",
                (unsigned)I2C_SCL, scl_mod ? "CONNECTED" : "NOT CONNECTED",
                (unsigned)I2C_SDA, sda_mod ? "CONNECTED" : "NOT CONNECTED");
    uart_puts("I2C scan:");
    for (a = 0x08; a < 0x78; a++) {
        r = i2c_probe(a);
        if (r == 1) { uart_printf(" 0x%02X", a); found++; }
        else if (r == -1) stuck++;
    }
    if (found)          uart_puts("  (OLED = 0x3C, MPU6050 = 0x68)\r\n");
    else if (stuck)     uart_printf(" bus stuck (%d timeouts) - a line is held low or shorted\r\n", stuck);
    else                uart_puts(" no device answered - devices not connected to PB6/PB7, or not powered\r\n");
    if (!scl) uart_puts("  -> SCL is LOW: PB6 line shorted to GND, or SCL/SDA wired to a GND/VCC hole\r\n");
    if (!sda) uart_puts("  -> SDA is LOW: PB7 line shorted to GND, or SCL/SDA wired to a GND/VCC hole\r\n");
}

/* Re-run the OLED and MPU6050 setup without pressing reset (phone command 'r') */
static void i2c_reinit_devices(void);

/* ================================================================
 *  SSD1306 OLED, 128 x 64, I2C address 0x3C
 * ================================================================ */
static uint8_t fb[128 * 64 / 8];               /* frame buffer: 8 pages of 128 bytes */

/* 5x7 font, ASCII 32..126; code 127 is drawn as a degree sign */
static const uint8_t font5x7[96][5] = {
    {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},{0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},{0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00},{0x00,0x41,0x22,0x1C,0x00},{0x08,0x2A,0x1C,0x2A,0x08},{0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},{0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},{0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},{0x00,0x36,0x36,0x00,0x00},{0x00,0x56,0x36,0x00,0x00},
    {0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},{0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E},{0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},{0x3E,0x41,0x49,0x49,0x7A},
    {0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},{0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40},{0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},
    {0x63,0x14,0x08,0x14,0x63},{0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43},{0x00,0x7F,0x41,0x41,0x00},
    {0x02,0x04,0x08,0x10,0x20},{0x00,0x41,0x41,0x7F,0x00},{0x04,0x02,0x01,0x02,0x04},{0x40,0x40,0x40,0x40,0x40},
    {0x00,0x01,0x02,0x04,0x00},{0x20,0x54,0x54,0x54,0x78},{0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},
    {0x38,0x44,0x44,0x48,0x7F},{0x38,0x54,0x54,0x54,0x18},{0x08,0x7E,0x09,0x01,0x02},{0x0C,0x52,0x52,0x52,0x3E},
    {0x7F,0x08,0x04,0x04,0x78},{0x00,0x44,0x7D,0x40,0x00},{0x20,0x40,0x44,0x3D,0x00},{0x7F,0x10,0x28,0x44,0x00},
    {0x00,0x41,0x7F,0x40,0x00},{0x7C,0x04,0x18,0x04,0x78},{0x7C,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},
    {0x7C,0x14,0x14,0x14,0x08},{0x08,0x14,0x14,0x18,0x7C},{0x7C,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x20,0x7C},{0x1C,0x20,0x40,0x20,0x1C},{0x3C,0x40,0x30,0x40,0x3C},
    {0x44,0x28,0x10,0x28,0x44},{0x0C,0x50,0x50,0x50,0x3C},{0x44,0x64,0x54,0x4C,0x44},{0x00,0x08,0x36,0x41,0x00},
    {0x00,0x00,0x7F,0x00,0x00},{0x00,0x41,0x36,0x08,0x00},{0x08,0x04,0x08,0x10,0x08},{0x06,0x09,0x09,0x06,0x00}
};
#define DEG "\x7F"                              /* use as: "25" DEG "C" */

static int oled_cmds(const uint8_t *c, uint32_t n) { return i2c_write(OLED_ADDR, 0x00, c, n); }

static int oled_init(void)
{
    static const uint8_t init_seq[] = {
        0xAE,             /* display off                         */
        0xD5, 0x80,       /* clock divide                        */
        0xA8, 0x3F,       /* multiplex: 64 rows                  */
        0xD3, 0x00,       /* display offset 0                    */
        0x40,             /* start line 0                        */
        0x8D, 0x14,       /* charge pump on                      */
        0x20, 0x00,       /* horizontal addressing mode          */
        0xA1,             /* flip horizontally                   */
        0xC8,             /* flip vertically                     */
        0xDA, 0x12,       /* COM pins for 128x64                 */
        0x81, 0xCF,       /* contrast                            */
        0xD9, 0xF1,       /* pre-charge                          */
        0xDB, 0x40,       /* VCOMH                               */
        0xA4,             /* show RAM contents                   */
        0xA6,             /* normal (not inverted)               */
        0x2E,             /* scrolling off                       */
        0xAF              /* display on                          */
    };
    return oled_cmds(init_seq, sizeof init_seq);
}

static int oled_flush(void)
{
    static const uint8_t window[] = { 0x21, 0, 127, 0x22, 0, 7 };   /* columns 0-127, pages 0-7 */
    int r = oled_cmds(window, sizeof window);
    if (r == 0) r = i2c_write(OLED_ADDR, 0x40, fb, sizeof fb);
    return r;
}

static void fb_clear(void) { memset(fb, 0, sizeof fb); }

static void fb_px(int x, int y, uint8_t on)
{
    if (x < 0 || x > 127 || y < 0 || y > 63) return;
    if (on) fb[x + (y / 8) * 128] |=  (uint8_t)(1U << (y & 7));
    else    fb[x + (y / 8) * 128] &= (uint8_t)~(1U << (y & 7));
}

static void fb_fill(int x, int y, int w, int h, uint8_t on)
{
    int i, j;
    for (j = y; j < y + h; j++) for (i = x; i < x + w; i++) fb_px(i, j, on);
}

static void fb_char(int x, int y, char ch, int scale, uint8_t on)
{
    int col, row;
    uint8_t c = (uint8_t)ch;
    if (c < 32 || c > 127) c = '?';
    for (col = 0; col < 5; col++) {
        uint8_t bits = font5x7[c - 32][col];
        for (row = 0; row < 8; row++) {
            if (bits & (1U << row)) fb_fill(x + col * scale, y + row * scale, scale, scale, on);
        }
    }
}

/* Draws a string, returns the x position after it */
static int fb_text(int x, int y, const char *s, int scale, uint8_t on)
{
    while (*s) { fb_char(x, y, *s++, scale, on); x += 6 * scale; }
    return x;
}

/* ================================================================
 *  MPU6050, I2C address 0x68
 * ================================================================ */
static float g_ref[3] = { 0.0f, 0.0f, 1.0f };   /* "upright" gravity direction */
static uint8_t g_fall_samples = 0;
static uint32_t g_last_event_ms = 0;
static uint8_t g_mpu_fail_run = 0;

static int mpu_write(uint8_t reg, uint8_t val) { return i2c_write(MPU_ADDR, reg, &val, 1); }

static int mpu_init(void)
{
    uint8_t who = 0;
    if (i2c_read_regs(MPU_ADDR, 0x75, &who, 1) != 0) return -1;   /* WHO_AM_I */
    g_mpu_who = who;
    if (mpu_write(0x6B, 0x80) != 0) return -1;   /* device reset          */
    delay_ms(100);
    if (mpu_write(0x6B, 0x01) != 0) return -1;   /* wake, clock = gyro X  */
    delay_ms(10);
    mpu_write(0x1A, 0x02);                       /* low-pass filter ~94 Hz  */
    mpu_write(0x19, 0x09);                       /* sample rate 100 Hz      */
    mpu_write(0x1B, 0x08);                       /* gyro +-500 deg/s        */
    mpu_write(0x1C, 0x10);                       /* accel +-8 g, 4096 LSB/g */
    return 0;
}

static int mpu_read_accel(int16_t a[3])
{
    uint8_t b[6];
    if (i2c_read_regs(MPU_ADDR, 0x3B, b, 6) != 0) return -1;      /* ACCEL_XOUT_H .. ZOUT_L */
    a[0] = (int16_t)((b[0] << 8) | b[1]);
    a[1] = (int16_t)((b[2] << 8) | b[3]);
    a[2] = (int16_t)((b[4] << 8) | b[5]);
    return 0;
}

static uint32_t isqrt32(uint32_t v)
{
    uint32_t r = 0, bit = 1UL << 30;
    while (bit > v) bit >>= 2;
    while (bit) {
        if (v >= r + bit) { v -= r + bit; r = (r >> 1) + bit; }
        else r >>= 1;
        bit >>= 2;
    }
    return r;
}

static uint32_t accel_mg(const int16_t a[3])
{
    uint32_t sq = (uint32_t)((int32_t)a[0] * a[0]) + (uint32_t)((int32_t)a[1] * a[1])
                + (uint32_t)((int32_t)a[2] * a[2]);
    return isqrt32(sq) * 1000U / 4096U;       /* 4096 LSB per g at +-8 g */
}

/* Average 32 readings with the box still: that direction becomes "upright" */
static void mpu_calibrate(void)
{
    int32_t sum[3] = { 0, 0, 0 };
    int16_t a[3];
    int i, n = 0;
    float len;
    for (i = 0; i < 32; i++) {
        if (mpu_read_accel(a) == 0) { sum[0] += a[0]; sum[1] += a[1]; sum[2] += a[2]; n++; }
        delay_ms(10);
    }
    if (n == 0) return;
    len = sqrtf((float)sum[0] * sum[0] + (float)sum[1] * sum[1] + (float)sum[2] * sum[2]);
    if (len < 1.0f) return;
    g_ref[0] = sum[0] / len; g_ref[1] = sum[1] / len; g_ref[2] = sum[2] / len;
    g_tilted = 0;
}

static void set_event(const char *text)
{
    strncpy(g_event, text, sizeof g_event - 1);
    g_event[sizeof g_event - 1] = '\0';
    g_event_until = g_ms + 5000U;              /* show it for 5 s */
    g_last_event_ms = g_ms;
    log_event(text);
}

static void mpu_sample(void)
{
    int16_t a[3];
    uint32_t mg;
    char msg[24];

    if (mpu_read_accel(a) != 0) {
        if (++g_mpu_fail_run >= 20) {
            g_mpu_ok = 0;
            uart_puts("MPU6050 stopped answering - check SCL/SDA wiring\r\n");
        }
        return;
    }
    g_mpu_fail_run = 0;

    mg = accel_mg(a);
    g_accel_mg = mg;
    if (mg > g_peak_mg) g_peak_mg = mg;
    if (mg > g_log_peak_mg) g_log_peak_mg = mg;

    /* Shock: a hit or a landing shows up as a short spike in total g */
    if (mg > SHOCK_MG && (g_ms - g_last_event_ms) > 1000U) {
        g_shock_count++;
        snprintf(msg, sizeof msg, "SHOCK %lu.%lu g", (unsigned long)(mg / 1000U), (unsigned long)((mg % 1000U) / 100U));
        set_event(msg);
        uart_printf("[%lu s] %s\r\n", (unsigned long)(g_ms / 1000U), msg);
        buzzer(80);
    }

    /* Free fall: total g near zero for 4 samples in a row (40 ms) */
    if (mg < FREEFALL_MG) {
        if (++g_fall_samples == 4 && (g_ms - g_last_event_ms) > 1000U) {
            set_event("DROPPED");
            uart_printf("[%lu s] FREE FALL - box dropped\r\n", (unsigned long)(g_ms / 1000U));
        }
    } else {
        g_fall_samples = 0;
    }

    /* Tilt: angle between now and the calibrated "upright" direction */
    if (mg > 700U && mg < 1300U) {            /* only when the box is roughly still */
        float n = sqrtf((float)a[0] * a[0] + (float)a[1] * a[1] + (float)a[2] * a[2]);
        float dot = (a[0] * g_ref[0] + a[1] * g_ref[1] + a[2] * g_ref[2]) / n;
        if (dot > 1.0f) dot = 1.0f;
        if (dot < -1.0f) dot = -1.0f;
        g_tilt_deg = (int)(acosf(dot) * 57.2958f + 0.5f);

        if (!g_tilted && g_tilt_deg > TILT_WARN_DEG) {
            g_tilted = 1;
            uart_printf("[%lu s] TILTED %d deg\r\n", (unsigned long)(g_ms / 1000U), g_tilt_deg);
            log_event("TILTED");
        } else if (g_tilted && g_tilt_deg < TILT_WARN_DEG - 10) {
            g_tilted = 0;
            uart_printf("[%lu s] Upright again\r\n", (unsigned long)(g_ms / 1000U));
            log_event("UPRIGHT");
        }
    }
}

/* ================================================================
 *  ColdTrace logic
 * ================================================================ */
static uint8_t temp_too_hot(void)  { return g_temp_valid && g_temp_c10 >= g_hi_alarm; }
static uint8_t temp_too_cold(void) { return g_temp_valid && g_tmin_c10 <  g_lo_alarm; }
static uint8_t temp_near_limit(void)
{
    return g_temp_valid && (g_temp_c10 >= g_hi_warn || g_tmin_c10 < g_lo_warn);
}

static state_t compute_state(void)
{
    if (g_door_open) return ST_ALARM;
    if (temp_too_hot() || temp_too_cold()) return ST_ALARM;
    if (g_frozen) return ST_ALARM;                        /* latched until a new trip */
    if (g_mkt_n > 0U && life_pct() == 0) return ST_ALARM; /* heat budget used up */
    if (time_before(g_event_until)) return ST_WARN;       /* recent shock / drop */
    if (g_pred_active) return ST_WARN;                    /* alarm predicted soon */
    if (g_tilted) return ST_WARN;
    if (g_probe_fault) return ST_WARN;
    if (g_mkt_n > 0U && life_pct() <= 25) return ST_WARN;
    if (!g_temp_valid || temp_near_limit()) return ST_WARN;
    return ST_OK;
}

static void update_state(void)
{
    state_t s = compute_state();
    if (s != g_state) {
        g_state = s;
        uart_printf("[%lu s] State -> %s\r\n", (unsigned long)(g_ms / 1000U), state_name[s]);
        log_event(s == ST_OK ? "STATE OK" : s == ST_WARN ? "STATE WARNING" : "STATE ALARM");
    }
    leds(s == ST_OK, s == ST_WARN, s == ST_ALARM);
}

/* The bottom line of the screen: the most important thing right now */
static const char *alert_text(uint8_t *inverse)
{
    *inverse = 1;
    if (g_door_open) return "DOOR OPEN";
    if (temp_too_hot())  return "TOO WARM";
    if (temp_too_cold()) return "TOO COLD";
    if (g_frozen) return "FROZEN-SHAKE TEST";
    if (g_mkt_n > 0U && life_pct() == 0) return "VVM DISCARD";
    if (time_before(g_event_until)) return g_event;
    if (g_pred_active) {
        static char pred[22];
        snprintf(pred, sizeof pred, "TOO %s IN %dm", g_pred_active == 1 ? "WARM" : "COLD", (int)(g_pred_min + 0.5f));
        return pred;
    }
    if (g_tilted) return "TILTED";
    if (g_probe_fault) return "PROBES DISAGREE";
    if (g_mkt_n > 0U && life_pct() <= 25) return "LIFE LOW";
    if (g_temp_valid && g_temp_c10 >= g_hi_warn) return "TEMP HIGH";
    if (g_temp_valid && g_tmin_c10 < g_lo_warn) return "TEMP LOW";
    if (!g_temp_valid) return "NO TEMP SENSOR";
    if (!g_dht_valid) return "DHT11 ERROR";
    *inverse = 0;
    return "All OK";
}

static void oled_draw(void)
{
    char line[24];
    int16_t t = g_temp_c10;
    uint8_t inv;
    const char *alert;
    int x;

    fb_clear();

    /* Title and state */
    fb_text(0, 0, "ColdTrace", 1, 1);
    fb_text(62, 0, g_sd_ok ? "SD" : "noSD", 1, 1);
    fb_text(128 - (int)strlen(state_short[g_state]) * 6, 0, state_short[g_state], 1, 1);

    /* Big temperature: the payload temperature the alarms use */
    if (g_temp_valid) c10_str(line, sizeof line, t);
    else              strcpy(line, "--.-");
    x = fb_text(0, 12, line, 2, 1);
    fb_text(x, 12, DEG "C", 2, 1);

    /* Humidity, top right */
    fb_text(92, 12, "RH", 1, 1);
    if (g_dht_valid) snprintf(line, sizeof line, "%u%%", (unsigned)g_hum);
    else             strcpy(line, "--");
    fb_text(92, 22, line, 1, 1);

    /* Info line, rotating every 3 s: probes / heat budget + MKT / date and time */
    if ((g_ms / 3000U) % 3U == 1U) {
        char a[8];
        if (g_mkt_n > 0U) snprintf(line, sizeof line, "Life %d%% MKT %s", life_pct(), c10_str(a, sizeof a, mkt_c10()));
        else              strcpy(line, "Life --  MKT --");
    } else if ((g_ms / 3000U) % 3U == 2U) {
        dt_t d;
        if (rtc_get(&d) == 0) snprintf(line, sizeof line, "%02u-%02u-%02u %02u:%02u:%02u", d.d, d.mo, d.y % 100U, d.h, d.mi, d.s);
        else                  strcpy(line, g_rtc_ok ? "Clock not set (t)" : "No clock crystal");
    } else {
        char a[8], b[8], c[8];
        if (g_probe_n == 0) {
            snprintf(line, sizeof line, "Air %s  no probe", g_dht_valid ? c10_str(a, sizeof a, g_air_c10) : "--");
        } else {
            if (g_dht_valid) snprintf(c, sizeof c, "%d", g_air_c10 / 10); else strcpy(c, "--");
            snprintf(line, sizeof line, "P1 %s P2 %s Air%s",
                     g_probe_ok[0] ? c10_str(a, sizeof a, g_probe_c10[0]) : "--",
                     (g_probe_n > 1 && g_probe_ok[1]) ? c10_str(b, sizeof b, g_probe_c10[1]) : "--", c);
            line[sizeof line - 1] = '\0';
        }
    }
    fb_text(0, 34, line, 1, 1);

    /* Door and motion */
    if (g_mpu_ok) {
        snprintf(line, sizeof line, "Door %s T%d" DEG " %lu.%lug", g_door_open ? "OPEN" : "shut", g_tilt_deg,
                 (unsigned long)(g_accel_mg / 1000U), (unsigned long)((g_accel_mg % 1000U) / 100U));
    } else {
        snprintf(line, sizeof line, "Door %s  no MPU", g_door_open ? "OPEN" : "shut");
    }
    fb_text(0, 44, line, 1, 1);

    /* Alert line, white-on-black when something needs attention */
    alert = alert_text(&inv);
    if (inv) fb_fill(0, 55, 128, 9, 1);
    fb_text(2, 56, alert, 1, (uint8_t)!inv);

    if (oled_flush() != 0) {
        g_oled_ok = 0;
        uart_puts("OLED stopped answering - check SCL/SDA wiring\r\n");
    }
}

static void print_band(void)
{
    char a[8], b[8], c[8], d[8];
    if (TEMP_PROFILE == 2 && !g_band_set) {
        uart_puts("Mode: DEMO - safe band will centre on the first temperature reading\r\n");
        return;
    }
    uart_printf("Mode: %s  safe %s to %s C (amber below %s or from %s C)\r\n",
                TEMP_PROFILE == 1 ? "VACCINE COLD CHAIN" : TEMP_PROFILE == 0 ? "ROOM TEST" : "DEMO",
                c10_str(a, sizeof a, g_lo_alarm), c10_str(b, sizeof b, g_hi_alarm),
                c10_str(c, sizeof c, g_lo_warn), c10_str(d, sizeof d, g_hi_warn));
}

/* Demo mode: make the current temperature the middle of the safe band */
static void set_demo_band(int16_t centre)
{
    g_lo_alarm = (int16_t)(centre - DEMO_ALARM_C10);
    g_lo_warn  = (int16_t)(centre - DEMO_WARN_C10);
    g_hi_warn  = (int16_t)(centre + DEMO_WARN_C10);
    g_hi_alarm = (int16_t)(centre + DEMO_ALARM_C10);
    g_band_set = 1;
    print_band();
}

static void trip_reset(void)
{
    g_excursion_s = 0;
    g_door_opens = 0;
    g_shock_count = 0;
    g_min_c10 = 32767;
    g_max_c10 = -32768;
    g_trip_start_ms = g_ms;
    g_mkt_sum = 0.0;
    g_mkt_n = 0;
    g_vvm_used = 0.0f;
    g_frozen = 0;
    g_probe_spikes = 0;
    g_pred_count = 0;
    if (TEMP_PROFILE == 2) g_band_set = 0;               /* demo: next reading sets a fresh band */
    trip_save();
}

static void trip_report(void)
{
    uint32_t dur = (g_ms - g_trip_start_ms) / 1000U;
    int16_t lo = g_min_c10, hi = g_max_c10;
    uart_puts("\r\n===== TRIP REPORT =====\r\n");
    uart_printf("Duration       : %lu min %lu s\r\n", (unsigned long)(dur / 60U), (unsigned long)(dur % 60U));
    {
        char a[8], b[8];
        uart_printf("Safe range     : %s to %s C%s\r\n", c10_str(a, sizeof a, g_lo_alarm), c10_str(b, sizeof b, g_hi_alarm),
                    TEMP_PROFILE == 2 ? " (demo band)" : "");
    }
    if (hi >= lo) {
        uart_printf("Min / max temp : %d.%d / %d.%d C\r\n",
                    lo / 10, (lo < 0 ? -lo : lo) % 10, hi / 10, (hi < 0 ? -hi : hi) % 10);
    } else {
        uart_puts("Min / max temp : no readings yet\r\n");
    }
    uart_printf("Measured by    : %s\r\n", g_probe_n ? "DS18B20 probes (+-0.5 C)" : "DHT11 air sensor only (+-2 C)");
    uart_printf("Out of range   : %lu min %lu s\r\n", (unsigned long)(g_excursion_s / 60U), (unsigned long)(g_excursion_s % 60U));
    uart_printf("Lid openings   : %lu\r\n", (unsigned long)g_door_opens);
    uart_printf("Shocks         : %lu\r\n", (unsigned long)g_shock_count);
    if (g_mkt_n > 0U) {
        char a[8];
        uart_printf("MKT            : %s C (pharma mean kinetic temperature)\r\n", c10_str(a, sizeof a, mkt_c10()));
        uart_printf("Heat budget    : %d %% left (VVM%d model%s)\r\n", life_pct(), VVM_DAYS_AT_37,
                    TEMP_PROFILE == 2 ? ", demo time x500" : "");
    }
    uart_printf("Frozen         : %s\r\n", g_frozen ? "YES - shake test needed" : "no");
    uart_printf("Early warnings : %lu predictions, %lu probe spikes ignored\r\n", (unsigned long)g_pred_count, (unsigned long)g_probe_spikes);
    if (g_sd_ok) uart_printf("Log            : TRIPLOG.CSV, %lu rows this power-up, last code %s\r\n", (unsigned long)g_sd_rows, g_chain);
    uart_printf("Verdict        : %s\r\n",
                (g_frozen || (g_mkt_n > 0U && life_pct() == 0)) ? "REJECT - frozen or heat budget used up" :
                g_excursion_s == 0 ? "PASS - stayed in range" : "CHECK - temperature excursion recorded");
    uart_puts("=======================\r\n");
}

static void report(void)
{
    char p1[8], p2[8], air[8];
    char mk[8];
    uart_printf("[%lu s] P1=%s P2=%s C  Air=%s C  H=%u %%  door=%s  tilt=%d deg  peak=%lu.%02lu g  state=%s  life=%d%%  mkt=%s\r\n",
                (unsigned long)(g_ms / 1000U),
                (g_probe_n > 0 && g_probe_ok[0]) ? c10_str(p1, sizeof p1, g_probe_c10[0]) : "--",
                (g_probe_n > 1 && g_probe_ok[1]) ? c10_str(p2, sizeof p2, g_probe_c10[1]) : "--",
                g_dht_valid ? c10_str(air, sizeof air, g_air_c10) : "--",
                (unsigned)g_hum,
                g_door_open ? "OPEN" : "closed",
                g_tilt_deg,
                (unsigned long)(g_peak_mg / 1000U), (unsigned long)((g_peak_mg % 1000U) / 10U),
                state_name[g_state], life_pct(), g_mkt_n ? c10_str(mk, sizeof mk, mkt_c10()) : "--");
}


/* ================================================================
 *  SD card log: TRIPLOG.CSV
 * ================================================================ */
static void sd_stop(const char *why, int err)
{
    g_sd_ok = 0;
    g_sd_errors++;
    PIN_HIGH(GPIOA, SD_CS);
    uart_printf("[%lu s] SD: %s (error %d) - logging stopped. Send m to restart it.\r\n",
                (unsigned long)(g_ms / 1000U), why, err);
}

/* Today's date/time for the file's folder entry, when the clock is set */
static void sd_stamp(void)
{
    dt_t t;
    if (rtc_get(&t) != 0) { fat_date_now = 0; return; }
    fat_date_now = (uint16_t)(((t.y - 1980U) << 9) | ((uint32_t)t.mo << 5) | t.d);
    fat_time_now = (uint16_t)(((uint32_t)t.h << 11) | ((uint32_t)t.mi << 5) | (t.s / 2U));
}

/* One CSV row:
   time_s,date_time,probe1_C,probe2_C,air_C,humidity_pct,door,tilt_deg,peak_g,mkt_C,life_pct,state,event,chain
   chain = first 8 hex digits of HMAC-SHA256(LOG_KEY, previous row's chain + this row up to the last comma) */
static void sd_log_row(const char *event)
{
    char line[200], dt[24], p1[8] = "", p2[8] = "", air[8] = "", hum[6] = "", tilt[8] = "", peak[10] = "", mk[8] = "", life[6] = "";
    int n, r;

    if (!g_sd_ok) return;
    if (g_probe_n > 0 && g_probe_ok[0]) c10_str(p1, sizeof p1, g_probe_c10[0]);
    if (g_probe_n > 1 && g_probe_ok[1]) c10_str(p2, sizeof p2, g_probe_c10[1]);
    if (g_dht_valid) { c10_str(air, sizeof air, g_air_c10); snprintf(hum, sizeof hum, "%u", (unsigned)g_hum); }
    if (g_mpu_ok) {
        snprintf(tilt, sizeof tilt, "%d", g_tilt_deg);
        snprintf(peak, sizeof peak, "%lu.%02lu", (unsigned long)(g_log_peak_mg / 1000U), (unsigned long)((g_log_peak_mg % 1000U) / 10U));
    }
    if (g_mkt_n > 0U) { c10_str(mk, sizeof mk, mkt_c10()); snprintf(life, sizeof life, "%d", life_pct()); }
    n = snprintf(line, sizeof line - 12U, "%lu,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,",
                 (unsigned long)(g_ms / 1000U), rtc_str(dt, sizeof dt), p1, p2, air, hum, g_door_open ? "open" : "closed",
                 tilt, peak, mk, life, state_name[g_state], event);
    if (n < 0 || n >= (int)sizeof line - 12) n = (int)strlen(line);
    {
        char code[9];
        chain_code(g_chain, line, (uint32_t)n, code);
        memcpy(line + n, code, 8);
        line[n + 8] = '\r'; line[n + 9] = '\n'; line[n + 10] = '\0';
        n += 10;
        sd_stamp();
        r = fat_append(line, (uint32_t)n);
        if (r == -2) { sd_stop("card is full", r); return; }
        if (r == 0) memcpy(g_chain, code, 9);
    }
    if (r == 0) r = fat_flush();
    if (r != 0) { sd_stop(fat_verify_err ? "read-back did not match what was written" : "write failed - card removed?", r); return; }
    g_sd_rows++;
}

/* After a restart, carry on the chain from the last code already on the card */
static void chain_recover(void)
{
    uint32_t pn, cn, back;
    char last[10];
    int i;
    strcpy(g_chain, "00000000");
    if (F.size < 12U || fat_read_tail(fat_tmp, &pn, fat_vbuf, &cn) != 0) return;
    for (i = 0; i < 10; i++) {                           /* the file's last 10 bytes: 8 hex digits + CR LF */
        back = 10U - (uint32_t)i;
        if (back <= cn)           last[i] = (char)fat_vbuf[cn - back];
        else if (back - cn <= pn) last[i] = (char)fat_tmp[pn - (back - cn)];
        else return;
    }
    if (last[8] != '\r' || last[9] != '\n') return;
    for (i = 0; i < 8; i++) {
        if (!((last[i] >= '0' && last[i] <= '9') || (last[i] >= 'a' && last[i] <= 'f'))) return;
    }
    memcpy(g_chain, last, 8);
    g_chain[8] = '\0';
}

static const char *fat_mount_msg(int r)
{
    switch (r) {
    case -1: return "card answered but a sector could not be read";
    case -2: case -3: return "no FAT file system found - format the card as FAT32 on a laptop";
    case -4: return "unusual sector size - format the card as FAT32";
    case -5: return "card is exFAT/NTFS (or tiny FAT12) - format it as FAT32";
    default: return "could not open the file system";
    }
}

/* Start the card: wake it up, mount FAT, open TRIPLOG.CSV, write a BOOT row */
static void sd_start(void)
{
    int r;
    g_sd_ok = 0;
    if (sd_init() != 0) {
        uart_puts("SD card: no card answered. Check the card is pushed in, VCC->5V, GND, CS->A4, SCK->A5, MISO->A6, MOSI->A7\r\n");
        return;
    }
    r = fat_mount();
    if (r != 0) { uart_printf("SD card: %s (error %d)\r\n", fat_mount_msg(r), r); return; }
    sd_stamp();
    r = fat_open_log();
    if (r < 0) {
        uart_printf("SD card: could not open TRIPLOG.CSV (error %d)%s\r\n", r, r == -6 ? " - root folder full" : "");
        return;
    }
    g_sd_ok = 1;
    if (r == 1) chain_recover();
    else strcpy(g_chain, "00000000");
    if (r == 0) {
        static const char hdr[] = "time_s,date_time,probe1_C,probe2_C,air_C,humidity_pct,door,tilt_deg,peak_g,mkt_C,life_pct,state,event,chain\r\n";
        if (fat_append(hdr, sizeof hdr - 1U) != 0 || fat_flush() != 0) { sd_stop("could not write the header", -1); return; }
    }
    uart_printf("SD card: %s, %s, %lu MB. %s TRIPLOG.CSV (%lu bytes so far)\r\n",
                (g_sd_type & CT_BLOCK) ? "SDHC" : "SDSC", F.fat32 ? "FAT32" : "FAT16",
                (unsigned long)(F.vol_secs / 2048U), r == 0 ? "Created" : "Appending to", (unsigned long)F.size);
    sd_log_row("BOOT");
    if (g_sd_ok) uart_puts("SD card: BOOT row written and read back OK - logging every 5 s\r\n");
}

static void sd_info(void)
{
    if (!g_sd_ok) {
        uart_puts("SD: not running - starting it...\r\n");
        sd_start();
        if (!g_sd_ok) return;
    }
    uart_printf("SD card  : %s, %s, %lu MB, %lu-byte clusters\r\n",
                (g_sd_type & CT_BLOCK) ? "SDHC" : "SDSC", F.fat32 ? "FAT32" : "FAT16",
                (unsigned long)(F.vol_secs / 2048U), (unsigned long)F.spc * 512UL);
    uart_printf("Log file : TRIPLOG.CSV, %lu bytes, %lu rows written since power-on\r\n",
                (unsigned long)F.size, (unsigned long)g_sd_rows);
    uart_printf("Checks   : %lu sectors written, each read back: %lu mismatches, %lu errors\r\n",
                (unsigned long)fat_sectors_written, (unsigned long)fat_verify_err, (unsigned long)g_sd_errors);
    uart_printf("Chain    : last row code %s (check the file on a laptop with verify_log.py)\r\n", g_chain);
}

/* Print the last lines of TRIPLOG.CSV as read back FROM THE CARD */
static void sd_show_tail(void)
{
    uint32_t pn, cn, i, start_c, start_p;
    int nl = 0;
    if (!g_sd_ok) { uart_puts("SD: not running (send m to start it)\r\n"); return; }
    if (fat_flush() != 0 || fat_read_tail(fat_tmp, &pn, fat_vbuf, &cn) != 0) { sd_stop("could not read the card back", -1); return; }
    if (cn == 0U) { uart_puts("TRIPLOG.CSV is empty\r\n"); return; }

    /* walk back over 6 line ends = the last 5 complete lines */
    start_c = 0; start_p = pn;
    for (i = cn; i > 0U; i--) {
        if (fat_vbuf[i - 1U] == '\n' && ++nl == 6) { start_c = i; break; }
    }
    if (nl < 6 && pn > 0U) {
        for (i = pn; i > 0U; i--) {
            if (fat_tmp[i - 1U] == '\n' && ++nl == 6) { start_p = i - 1U + 1U; break; }
            start_p = i - 1U;
        }
    }
    uart_puts("---- last lines of TRIPLOG.CSV, read back from the SD card ----\r\n");
    if (nl < 6 || start_c == 0U) {
        for (i = start_p; i < pn; i++) uart_putc((char)fat_tmp[i]);
    }
    for (i = start_c; i < cn; i++) uart_putc((char)fat_vbuf[i]);
    uart_printf("---- card copy %s the firmware's copy (%lu bytes in file) ----\r\n",
                memcmp(fat_vbuf, fat_tail, cn) == 0 ? "MATCHES" : "DIFFERS FROM", (unsigned long)F.size);
}

static void sd_eject(void)
{
    if (g_sd_ok && fat_flush() != 0) { sd_stop("last write failed", -1); return; }
    g_sd_ok = 0;
    PIN_HIGH(GPIOA, SD_CS);
    uart_puts("SD: everything is saved - safe to take the card out. Send m to start logging again.\r\n");
}

static void splash(const char *l1, const char *l2)
{
    if (!g_oled_ok) return;
    fb_clear();
    fb_text(10, 12, "ColdTrace", 2, 1);
    fb_text(0, 40, l1, 1, 1);
    fb_text(0, 52, l2, 1, 1);
    oled_flush();
}

static void i2c_reinit_devices(void)
{
    i2c_config();
    i2c_scan();
    g_oled_ok = (uint8_t)(oled_init() == 0);
    uart_printf("OLED: %s\r\n", g_oled_ok ? "OK" : "not found at 0x3C");
    g_mpu_ok = (uint8_t)(mpu_init() == 0);
    if (g_mpu_ok) { mpu_calibrate(); uart_printf("MPU6050: OK (WHO_AM_I = 0x%02X)\r\n", (unsigned)g_mpu_who); }
    else uart_puts("MPU6050: not found at 0x68\r\n");
}

int main(void)
{
    uint32_t last_dht, last_blink = 0, last_mpu = 0, last_oled = 0, last_state = 0, last_log = 0;
    char     tline[28];                                    /* 't' command: date/time being typed */
    uint8_t  tline_on = 0, tline_n = 0;
    uint32_t tline_ms = 0;
    uint32_t door_check_at = 0;
    uint8_t  door_pending = 0;
    int c, i;

    clock_init();
    g_cyc_per_us = g_sysclk / 1000000U;
    DBGMCU->CR |= DBGMCU_CR_DBG_SLEEP;                     /* keep the debugger connected while asleep */
    SysTick_Config(g_sysclk / 1000U);
    dwt_init();
    gpio_init();
    uart_init();
    reed_irq_init();

    /* ---- Self-test: LEDs, buzzer, Bluetooth ---- */
    for (i = 0; i < 3; i++) {
        PIN_LOW(GPIOC, BOARD_LED);  delay_ms(150);
        PIN_HIGH(GPIOC, BOARD_LED); delay_ms(150);
    }
    leds(1, 0, 0); delay_ms(400);
    leds(0, 1, 0); delay_ms(400);
    leds(0, 0, 1); delay_ms(400);
    leds(0, 0, 0);
    buzzer(150);

    uart_puts("\r\n=== ColdTrace self-test (v5 final: analytics + tamper-proof log) ===\r\n");
    uart_printf("Clock: %lu MHz (%s)\r\n", (unsigned long)(g_sysclk / 1000000U),
                g_sysclk == 84000000U ? "crystal + PLL OK" : "internal HSI - crystal did not start");

    /* ---- I2C devices ---- */
    i2c_init();
    i2c_scan();

    g_oled_ok = (uint8_t)(oled_init() == 0);
    uart_printf("OLED: %s\r\n", g_oled_ok ? "OK" : "not found at 0x3C");
    splash("Starting...", "Keep the box still");

    g_mpu_ok = (uint8_t)(mpu_init() == 0);
    if (g_mpu_ok) {
        uart_printf("MPU6050: OK (WHO_AM_I = 0x%02X%s)\r\n", (unsigned)g_mpu_who,
                    g_mpu_who == 0x68 ? "" : ", clone chip - still usable");
        mpu_calibrate();
        uart_puts("Tilt calibrated: current position = upright\r\n");
    } else {
        uart_puts("MPU6050: not found at 0x68\r\n");
    }

    ow_init_pin();
    delay_ms(2);
    ow_search_all();
    probes_print_ids();
    probes_start();

    g_door_open = reed_door_open();
    uart_printf("Door: %s\r\n", g_door_open ? "OPEN" : "closed");

    rtc_init();                                            /* clock + trip memory */
    if (!g_rtc_ok) uart_puts("Clock: 32.768 kHz crystal did not start - no date/time, trip not kept over a reset\r\n");
    else {
        char dt[24];
        if (rtc_time_set()) uart_printf("Clock: %s\r\n", rtc_str(dt, sizeof dt));
        else uart_puts("Clock: running but not set - send t2026-10-21 09:30:00 (your date and time)\r\n");
        if (trip_restore()) uart_printf("Trip resumed after a reset: %lu min so far, heat budget %d %% left (z = new trip)\r\n",
                                        (unsigned long)((g_ms - g_trip_start_ms) / 60000U), life_pct());
        else trip_save();
    }
    uart_printf("Heat budget model: VVM%d%s\r\n", VVM_DAYS_AT_37, TEMP_PROFILE == 2 ? " (demo: time runs x500)" : "");

    sd_start();                                            /* SD card log */
    g_logq_n = 0;                                          /* start-up messages are not events */
    last_log = g_ms;
    uart_puts("Commands: s = status, x = trip report, z = new trip, d = re-centre demo band, p = find probes, m = SD card info, l = last lines on SD, e = eject SD, t = clock (t2026-10-21 09:30:00), b = beep, c = recalibrate tilt, i = I2C scan, r = restart OLED/MPU, h = help\r\n");
    print_band();

    delay_ms(800);                                         /* DHT11 warm-up, first probe reading */
    last_dht = g_ms - DHT_PERIOD_MS;                       /* first reading straight away */

    /* ---- Main loop ---- */
    while (1) {
        /* Heartbeat on PC13 */
        if ((g_ms - last_blink) >= 500U) {
            last_blink = g_ms;
            GPIOC->ODR ^= (1U << BOARD_LED);
        }

        /* Door: the interrupt raises a flag; read the level 30 ms later to ignore bounce */
        if (g_door_irq) {
            g_door_irq = 0;
            door_pending = 1;
            door_check_at = g_ms + 30U;
        }
        if (door_pending && !time_before(door_check_at)) {
            uint8_t open = reed_door_open();
            door_pending = 0;
            if (open != g_door_open) {
                g_door_open = open;
                uart_printf("[%lu s] DOOR %s\r\n", (unsigned long)(g_ms / 1000U), open ? "OPEN" : "CLOSED");
                if (open) g_door_opens++;
                log_event(open ? "DOOR OPEN" : "DOOR CLOSED");
                update_state();
                if (open) buzzer(300);
            }
        }

        /* MPU6050 every 10 ms */
        if (g_mpu_ok && (g_ms - last_mpu) >= 10U) {
            last_mpu = g_ms;
            mpu_sample();
        }

        /* DHT11 every 2 s */
        if ((g_ms - last_dht) >= DHT_PERIOD_MS) {
            int16_t t = 0;
            uint8_t h = 0;
            int err;

            last_dht = g_ms;
            probes_cycle();                                /* DS18B20 payload temperature */
            err = dht11_read(&t, &h);                      /* DHT11 humidity + air temperature */
            if (err == 0) {
                g_air_c10 = t;
                g_hum = h;
                g_dht_ok++;
                g_dht_valid = 1;
            } else {
                g_dht_err++;
                g_dht_last_error = err;
                g_dht_valid = 0;
                uart_printf("[%lu s] DHT11 error %d\r\n", (unsigned long)(g_ms / 1000U), err);
            }

            update_temps();
            if (TEMP_PROFILE == 2 && !g_band_set && g_temp_valid) set_demo_band(g_temp_c10);
            analytics_update();                            /* MKT, heat budget, prediction, freeze */
            if (g_temp_valid) {
                if (g_tmin_c10 < g_min_c10) g_min_c10 = g_tmin_c10;
                if (g_temp_c10 > g_max_c10) g_max_c10 = g_temp_c10;
                if (temp_too_hot() || temp_too_cold()) g_excursion_s += DHT_PERIOD_MS / 1000U;
            }

            update_state();
            report();
            trip_save();                                   /* survive a reset */
            g_peak_mg = 0;                                 /* start a new peak window */
            if (!g_door_open && (temp_too_hot() || temp_too_cold())) {
                uart_puts(temp_too_hot() ? "!!! TOO WARM - above safe range !!!\r\n"
                                         : "!!! TOO COLD - freezing risk !!!\r\n");
                buzzer(200);
            }
        }

        /* State and LEDs every 100 ms (clears shock warnings after 5 s) */
        if ((g_ms - last_state) >= 100U) {
            last_state = g_ms;
            update_state();
        }

        /* SD card: queued events first, then a regular row every SD_LOG_PERIOD_MS */
        if (g_logq_n > 0U) {
            if (g_sd_ok) sd_log_row(g_logq[0]);
            g_logq_n--;
            memmove(g_logq[0], g_logq[1], (size_t)g_logq_n * sizeof g_logq[0]);
        } else if ((g_ms - last_log) >= SD_LOG_PERIOD_MS) {
            last_log = g_ms;
            sd_log_row("");
            g_log_peak_mg = 0;
        }

        /* OLED every 500 ms */
        if (g_oled_ok && (g_ms - last_oled) >= 500U) {
            last_oled = g_ms;
            oled_draw();
        }

        /* Commands from the phone */
        c = uart_getc();
        if (tline_on) {                                    /* collecting "t2026-10-21 09:30:00" */
            if (c >= 0 && c != '\r' && c != '\n' && tline_n < sizeof tline - 1U) { tline[tline_n++] = (char)c; tline_ms = g_ms; }
            if (c == '\r' || c == '\n' || (c < 0 && (g_ms - tline_ms) > 1500U)) {
                uint32_t v[6] = { 0, 0, 0, 0, 0, 0 };
                int k = 0, digits = 0;
                uint8_t j;
                char dt[24];
                tline[tline_n] = '\0';
                tline_on = 0;
                for (j = 0; j <= tline_n && k < 6; j++) {   /* pull out up to six numbers */
                    if (tline[j] >= '0' && tline[j] <= '9') { v[k] = v[k] * 10U + (uint32_t)(tline[j] - '0'); digits = 1; }
                    else if (digits) { k++; digits = 0; }
                }
                if (k == 0) {
                    if (rtc_time_set()) uart_printf("Clock: %s\r\n", rtc_str(dt, sizeof dt));
                    else uart_puts("Clock not set. Send: t2026-10-21 09:30:00\r\n");
                } else if (k < 5 || rtc_set(v[0], v[1], v[2], v[3], v[4], v[5]) != 0) {
                    uart_puts(g_rtc_ok ? "Could not read that. Send: t2026-10-21 09:30:00\r\n" : "No clock crystal running\r\n");
                } else {
                    uart_printf("Clock set: %s\r\n", rtc_str(dt, sizeof dt));
                    log_event("CLOCK SET");
                }
            }
            c = -1;
        }
        if (c == 't') { tline_on = 1; tline_n = 0; tline_ms = g_ms; }
        else if (c == 's') report();
        else if (c == 'b') buzzer(150);
        else if (c == 'i') i2c_scan();
        else if (c == 'r') i2c_reinit_devices();
        else if (c == 'x') trip_report();
        else if (c == 'd') {
            if (TEMP_PROFILE != 2)  uart_puts("'d' only works in demo mode (TEMP_PROFILE 2)\r\n");
            else if (!g_temp_valid) uart_puts("No temperature reading yet\r\n");
            else { uart_puts("Demo band re-centred on the current temperature\r\n"); set_demo_band(g_temp_c10); }
        }
        else if (c == 'p') { ow_search_all(); probes_print_ids(); probes_start(); }
        else if (c == 'z') { trip_reset(); log_event("NEW TRIP"); uart_puts("Trip record cleared - new trip started\r\n"); }
        else if (c == 'm') sd_info();
        else if (c == 'l') sd_show_tail();
        else if (c == 'e') sd_eject();
        else if (c == 'c') {
            if (g_mpu_ok) { mpu_calibrate(); uart_puts("Tilt recalibrated\r\n"); }
            else uart_puts("MPU6050 not available\r\n");
        }
        else if (c == 'h') uart_puts("s = status, x = trip report, z = new trip, d = re-centre demo band, p = find probes, m = SD card info, l = last lines on SD, e = eject SD, t = clock (t2026-10-21 09:30:00), b = beep, c = recalibrate tilt, i = I2C scan, r = restart OLED/MPU, h = help\r\n");

#if LOW_POWER
        __WFI();                                           /* sleep until the next tick or interrupt */
#endif
    }
}
