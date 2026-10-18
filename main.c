/*
 * ColdTrace - Stage 1 firmware (v2: + OLED + MPU6050)
 * Board : STM32F401 "Black Pill" (WeAct V3), 25 MHz crystal
 * Tool  : Keil uVision 5, CMSIS-CORE + Device:Startup only (register level, no HAL)
 *
 * Every reset it runs a self-test:
 *   1. PC13 onboard LED blinks 3 times      -> clock + flashing work
 *   2. Green, amber, red LEDs light in turn  -> PB12 / PB13 / PB14
 *   3. Buzzer beeps once                     -> PB10 open-drain (5V buzzer), or PB1 if BUZZER_OPEN_DRAIN = 0
 *   4. Banner sent to the phone              -> HC-05 on USART2 (PA2 TX, PA3 RX), 9600 baud
 *   5. I2C scan of PB6/PB7, reported on the phone (expect 0x3C OLED and 0x68 MPU6050)
 *   6. OLED splash, then MPU6050 tilt calibration (keep the box still for ~1 s)
 *
 * Then it runs the mini-ColdTrace loop:
 *   - Reed switch on PA10 (EXTI10 interrupt): DOOR OPEN / CLOSED + beep
 *   - DHT11 on PB15 every 2 s
 *   - MPU6050 every 10 ms: shock (> SHOCK_MG), free fall (drop), tilt from the calibrated position
 *   - OLED shows temperature, humidity, door, tilt, g-force and the current alert
 *   - Green = OK, amber = warning (temp high, shock, tilt), red = alarm (door open, temp too high)
 *   - Phone commands: s = status, b = beep, c = recalibrate tilt, i = I2C scan, h = help
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
#define TEMP_WARN_C10           310   /* 31.0 C - indoor test values, set 2-3 C above room   */
#define TEMP_ALARM_C10          330   /* 33.0 C - cold-chain build will use 8.0 C later      */
#define SHOCK_MG                2500U /* total acceleration above 2.5 g counts as a shock    */
#define FREEFALL_MG             350U  /* below 0.35 g for 40 ms counts as a drop             */
#define TILT_WARN_DEG           45    /* tilt from the calibrated position that warns        */
#define I2C_FAST                1     /* 1 = 400 kHz, 0 = 100 kHz (try 0 if I2C is flaky)     */
#define I2C_SCL_ON_PB8          1     /* 1 = I2C clock on PB8 instead of PB6                 */
#define DHT_PERIOD_MS           2000U
#define UART_BAUD               9600U

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
volatile int16_t  g_temp_c10 = 0;      /* temperature x10, e.g. 284 = 28.4 C */
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
volatile uint32_t g_i2c_err = 0;

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
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
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
    uint32_t sr = USART2->SR;
    if (sr & USART_SR_RXNE) return (int)(USART2->DR & 0xFFU);
    if (sr & USART_SR_ORE)  (void)USART2->DR;      /* clear overrun */
    return -1;
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
        } else if (g_tilted && g_tilt_deg < TILT_WARN_DEG - 10) {
            g_tilted = 0;
            uart_printf("[%lu s] Upright again\r\n", (unsigned long)(g_ms / 1000U));
        }
    }
}

/* ================================================================
 *  ColdTrace logic
 * ================================================================ */
static state_t compute_state(void)
{
    if (g_door_open) return ST_ALARM;
    if (g_dht_valid && g_temp_c10 >= TEMP_ALARM_C10) return ST_ALARM;
    if (time_before(g_event_until)) return ST_WARN;       /* recent shock / drop */
    if (g_tilted) return ST_WARN;
    if (!g_dht_valid || g_temp_c10 >= TEMP_WARN_C10) return ST_WARN;
    return ST_OK;
}

static void update_state(void)
{
    state_t s = compute_state();
    if (s != g_state) {
        g_state = s;
        uart_printf("[%lu s] State -> %s\r\n", (unsigned long)(g_ms / 1000U), state_name[s]);
    }
    leds(s == ST_OK, s == ST_WARN, s == ST_ALARM);
}

/* The bottom line of the screen: the most important thing right now */
static const char *alert_text(uint8_t *inverse)
{
    *inverse = 1;
    if (g_door_open) return "DOOR OPEN";
    if (g_dht_valid && g_temp_c10 >= TEMP_ALARM_C10) return "TEMP ALARM";
    if (time_before(g_event_until)) return g_event;
    if (g_tilted) return "TILTED";
    if (g_dht_valid && g_temp_c10 >= TEMP_WARN_C10) return "TEMP HIGH";
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
    fb_text(128 - (int)strlen(state_short[g_state]) * 6, 0, state_short[g_state], 1, 1);

    /* Big temperature */
    if (g_dht_valid) snprintf(line, sizeof line, "%d.%d", t / 10, (t < 0 ? -t : t) % 10);
    else             strcpy(line, "--.-");
    x = fb_text(0, 12, line, 2, 1);
    fb_text(x, 12, DEG "C", 2, 1);

    /* Humidity, top right */
    fb_text(92, 12, "RH", 1, 1);
    if (g_dht_valid) snprintf(line, sizeof line, "%u%%", (unsigned)g_hum);
    else             strcpy(line, "--");
    fb_text(92, 22, line, 1, 1);

    /* Door */
    snprintf(line, sizeof line, "Door  %s", g_door_open ? "OPEN" : "closed");
    fb_text(0, 34, line, 1, 1);

    /* Motion */
    if (g_mpu_ok) {
        snprintf(line, sizeof line, "Tilt %d" DEG "  %lu.%02lu g", g_tilt_deg,
                 (unsigned long)(g_accel_mg / 1000U), (unsigned long)((g_accel_mg % 1000U) / 10U));
    } else {
        strcpy(line, "MPU6050 not found");
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

static void report(void)
{
    int16_t t = g_temp_c10;
    uart_printf("[%lu s] T=%d.%d C  H=%u %%  door=%s  tilt=%d deg  peak=%lu.%02lu g  state=%s\r\n",
                (unsigned long)(g_ms / 1000U),
                t / 10, (t < 0 ? -t : t) % 10,
                (unsigned)g_hum,
                g_door_open ? "OPEN" : "closed",
                g_tilt_deg,
                (unsigned long)(g_peak_mg / 1000U), (unsigned long)((g_peak_mg % 1000U) / 10U),
                state_name[g_state]);
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
    uint32_t last_dht, last_blink = 0, last_mpu = 0, last_oled = 0, last_state = 0;
    uint32_t door_check_at = 0;
    uint8_t  door_pending = 0;
    int c, i;

    clock_init();
    g_cyc_per_us = g_sysclk / 1000000U;
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

    uart_puts("\r\n=== ColdTrace stage 1 self-test (v2) ===\r\n");
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

    g_door_open = reed_door_open();
    uart_printf("Door: %s\r\n", g_door_open ? "OPEN" : "closed");
    uart_puts("Commands: s = status, b = beep, c = recalibrate tilt, i = I2C scan, r = restart OLED/MPU, h = help\r\n");

    delay_ms(500);                                         /* DHT11 needs ~1 s after power-up */
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
            err = dht11_read(&t, &h);
            if (err == 0) {
                g_temp_c10 = t;
                g_hum = h;
                g_dht_ok++;
                g_dht_valid = 1;
            } else {
                g_dht_err++;
                g_dht_last_error = err;
                g_dht_valid = 0;
                uart_printf("[%lu s] DHT11 error %d\r\n", (unsigned long)(g_ms / 1000U), err);
            }

            update_state();
            if (g_dht_valid) report();
            g_peak_mg = 0;                                 /* start a new peak window */
            if (g_dht_valid && g_temp_c10 >= TEMP_ALARM_C10 && !g_door_open) {
                uart_puts("!!! TEMP ALARM !!!\r\n");
                buzzer(200);
            }
        }

        /* State and LEDs every 100 ms (clears shock warnings after 5 s) */
        if ((g_ms - last_state) >= 100U) {
            last_state = g_ms;
            update_state();
        }

        /* OLED every 500 ms */
        if (g_oled_ok && (g_ms - last_oled) >= 500U) {
            last_oled = g_ms;
            oled_draw();
        }

        /* Commands from the phone */
        c = uart_getc();
        if (c == 's')      report();
        else if (c == 'b') buzzer(150);
        else if (c == 'i') i2c_scan();
        else if (c == 'r') i2c_reinit_devices();
        else if (c == 'c') {
            if (g_mpu_ok) { mpu_calibrate(); uart_puts("Tilt recalibrated\r\n"); }
            else uart_puts("MPU6050 not available\r\n");
        }
        else if (c == 'h') uart_puts("s = status, b = beep, c = recalibrate tilt, i = I2C scan, r = restart OLED/MPU, h = help\r\n");
    }
}
