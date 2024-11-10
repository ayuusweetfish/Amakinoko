#include <stm32g0xx_hal.h>
#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define MUMU_ROM_SIZE 1024
#define MUMU_RAM_SIZE 512
#include "../../misc/mumu/mumu.h"

#define REV 2

extern uint32_t _sidata, _sdata, _edata;
#define FLASH_START_ADDR    0x08000000
#define FW_END_ADDR         ((uint32_t)&_sidata + ((uint32_t)&_edata - (uint32_t)&_sdata))
#define STORAGE_START_ADDR  ((FW_END_ADDR + FLASH_PAGE_SIZE - 1) & ~(FLASH_PAGE_SIZE - 1))
#define STORAGE_SIZE        4096
#define FLASH_END_ADDR      (FLASH_START_ADDR + (32 * 1024))
_Static_assert(STORAGE_SIZE % FLASH_PAGE_SIZE == 0);

#define STORAGE_ROM_LEN_OFFS  (STORAGE_SIZE - 4)
#define STORAGE_SRC_LEN_OFFS  (STORAGE_SIZE - 2)
#define STORAGE_ROM_OFFS      (0)
#define STORAGE_SRC_OFFS      (MUMU_ROM_SIZE)
#define STORAGE_ROM_LEN       ((uint16_t *)(STORAGE_START_ADDR + STORAGE_ROM_LEN_OFFS))
#define STORAGE_SRC_LEN       ((uint16_t *)(STORAGE_START_ADDR + STORAGE_SRC_LEN_OFFS))
#define STORAGE_ROM_START     ((void *)(STORAGE_START_ADDR + STORAGE_ROM_OFFS))
#define STORAGE_SRC_START     ((void *)(STORAGE_START_ADDR + STORAGE_SRC_OFFS))
#define STORAGE_SRC_MAX_LEN   (STORAGE_SIZE - 4 - STORAGE_SRC_OFFS)

// #define RELEASE
#ifndef RELEASE
#define _release_inline
static uint8_t swv_buf[256];
static size_t swv_buf_ptr = 0;
__attribute__ ((noinline, used))
void swv_trap_line()
{
  *(volatile char *)swv_buf;
}
static inline void swv_putchar(uint8_t c)
{
  // ITM_SendChar(c);
  if (c == '\n') {
    swv_buf[swv_buf_ptr >= sizeof swv_buf ?
      (sizeof swv_buf - 1) : swv_buf_ptr] = '\0';
    swv_trap_line();
    swv_buf_ptr = 0;
  } else if (++swv_buf_ptr <= sizeof swv_buf) {
    swv_buf[swv_buf_ptr - 1] = c;
  }
}
__attribute__ ((format(printf, 1, 2)))
static void swv_printf(const char *restrict fmt, ...)
{
  char s[256];
  va_list args;
  va_start(args, fmt);
  int r = vsnprintf(s, sizeof s, fmt, args);
  for (int i = 0; i < r && i < sizeof s - 1; i++) swv_putchar(s[i]);
  if (r >= sizeof s) {
    for (int i = 0; i < 3; i++) swv_putchar('.');
    swv_putchar('\n');
  }
}
#else
#define _release_inline inline
#define swv_printf(...)
#endif

__attribute__ ((section(".RamFunc")))
static inline void spin_delay(uint32_t cycles)
{
  __asm__ volatile (
    "   cmp %[cycles], #5\n"
    "   ble 2f\n"
    "   sub %[cycles], #5\n"
    "   lsr %[cycles], #2\n"
    "1: sub %[cycles], #1\n"
    "   nop\n"
    "   bne 1b\n"   // 2 cycles if taken
    "2: \n"
    : [cycles] "+l" (cycles)
    : // No output
    : "cc"
  );
}
__attribute__ ((section(".RamFunc")))
static inline void delay_us(uint32_t us)
{
  spin_delay(us * 64);
}

static I2C_HandleTypeDef i2c1;
static TIM_HandleTypeDef tim3;
static UART_HandleTypeDef uart2;

static uint8_t rx_byte;

static void run();
static void tx_readings_if_connected();

__attribute__ ((section(".RamFunc")))
static void storage_write(uint32_t offs, const uint8_t data[256])
{
  // Silently break if offset not of a multiple of fast-write burst length (32 double-words)
  if (offs % (32 * 8) != 0) return;
  // Silently break if data is not properly aligned
  if ((uint32_t)&data[0] % 8 != 0) return;

  HAL_FLASH_Unlock();

  // Fast programming only works with the last erased page
  // https://community.st.com/t5/stm32-mcus-embedded-software/stm32g0-and-flash-typeprogram-fast-fail/m-p/122457/highlight/true#M6251
  // https://community.st.com/t5/stm32-mcus-products/stm32g0-flash-program-fast-on-non-last-erased-page/td-p/72284
  if (offs % FLASH_PAGE_SIZE == 0) {
    uint32_t page_err;
    HAL_FLASHEx_Erase(&(FLASH_EraseInitTypeDef){
      .TypeErase = FLASH_TYPEERASE_PAGES,
      .Banks = FLASH_BANK_1,
      .Page = (STORAGE_START_ADDR - FLASH_START_ADDR + offs) / FLASH_PAGE_SIZE,
      .NbPages = 1,
    }, &page_err);
  }

  __disable_irq();
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_FAST, STORAGE_START_ADDR + offs, (uint32_t)&data[0]);
  __enable_irq();
  HAL_FLASH_Lock();
}

// Pull a set of pins to a given level and set them as input
static inline void pull_electrodes_port(GPIO_TypeDef *port, uint32_t pins, bool level)
{
  GPIO_InitTypeDef gpio_init = (GPIO_InitTypeDef){
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Pin = pins,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  };
  HAL_GPIO_Init(port, &gpio_init);
  HAL_GPIO_WritePin(port, pins, level);
  gpio_init.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(port, &gpio_init);
}

#define N_ELECTRODES 4

#if REV == 1
  #define BTN_OUT_PORT  GPIOC
  #define BTN_OUT_PIN   GPIO_PIN_15
  #define TOUCH_PINS_A  ((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3))
  #define TOUCH_MASK_   1 << 0, 1 << 1, 1 << 2, 1 << 3,
#elif REV == 2
  #define BTN_OUT_PORT  GPIOA
  #define BTN_OUT_PIN   GPIO_PIN_6
  #define TOUCH_PINS_A  ((1 << 7) | (1 << 8) | (1 << 11) | (1 << 12))
  #define TOUCH_MASK_   1 << 7, 1 << 8, 1 << 11, 1 << 12,
#endif

static const uint16_t TOUCH_MASK[N_ELECTRODES] = { TOUCH_MASK_ };

static inline void pull_electrodes(bool level)
{
  pull_electrodes_port(GPIOA, TOUCH_PINS_A, level);
}

#pragma GCC push_options
#pragma GCC optimize("O3")
static inline void cap_sense(uint16_t cap_sum[N_ELECTRODES])
{
  struct record_t {
    uint16_t t;
    uint16_t v;
  } record[16];

  static const uint16_t FULL_MASK =
    TOUCH_MASK[0] | TOUCH_MASK[1] | TOUCH_MASK[2] | TOUCH_MASK[3];

  uint16_t cap[4] = { 0 };
  for (int i = 0; i < N_ELECTRODES; i++) cap_sum[i] = 0;

  inline void toggle(const bool level) {
    pull_electrodes(1 - level); // Pull to the opposite level before reading
    int n_records = 0;
    uint16_t last_v = (level == 1 ? ~FULL_MASK : FULL_MASK);
    record[n_records] = (struct record_t){.t = (uint16_t)-1, .v = last_v};
    __disable_irq();
    HAL_GPIO_WritePin(BTN_OUT_PORT, BTN_OUT_PIN, level);
    for (int i = 0; i < 100; i++) {
      uint32_t combined_v = GPIOA->IDR;
      uint16_t cur_v = (level == 1 ? (combined_v | last_v) : (combined_v & last_v));
      if (last_v != cur_v) n_records++;
      record[n_records] = (struct record_t){.t = i, .v = cur_v};
      last_v = cur_v;
    }
    __enable_irq();
    for (int j = 0; j < N_ELECTRODES; j++) cap[j] = 0xffff;
    for (int i = 1; i <= n_records; i++) {
      uint16_t t = record[i - 1].t + 1;
      uint16_t diff = record[i - 1].v ^ record[i].v;
      for (int j = 0; j < N_ELECTRODES; j++)
        if (diff & TOUCH_MASK[j]) cap[j] = t;
    }
    for (int j = 0; j < N_ELECTRODES; j++)
      if (cap_sum[j] == 0xffff || cap[j] == 0xffff) cap_sum[j] = 0xffff;
      else cap_sum[j] += cap[j];
  }
  for (int its = 0; its < 5; its++) {
    toggle(1);
    toggle(0);
  }

  // Maintain a base value for each of the buttons

  // The base value increases by 1 every 16 iterations (~ 0.5 second)
  static const uint32_t BASE_MULT = 16;
  static uint32_t base[N_ELECTRODES] = { UINT32_MAX };
  if (base[0] == UINT32_MAX) {
    for (int j = 0; j < N_ELECTRODES; j++) base[j] = cap_sum[j] * BASE_MULT;
  } else {
    for (int j = 0; j < N_ELECTRODES; j++) {
      base[j] += 1;
      uint32_t lower = cap_sum[j] * BASE_MULT;
      if (base[j] > lower)
        base[j] = lower + (base[j] - lower) * 3 / 4;
    }
  }
  for (int j = 0; j < N_ELECTRODES; j++) {
    uint32_t base_scaled = base[j] / BASE_MULT;
    cap_sum[j] = (cap_sum[j] > base_scaled ? cap_sum[j] - base_scaled : 0);
  }
}
#pragma GCC pop_options

static inline void adc_start()
{
  while (!(ADC1->CR & ADC_CR_ADEN)) {
    ADC1->CR = (ADC1->CR & ~(ADC_CR_ADCAL | ADC_CR_ADSTP | ADC_CR_ADSTART | ADC_CR_ADDIS)) | ADC_CR_ADEN;
  }
}
static inline uint32_t adc_poll_value()
{
  ADC1->ISR = (ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR);  // All bits of ADC_ISR are r/cw1
  ADC1->CR = ADC_CR_ADSTART;  // All bits of ADC_CR are r/s
  while (!(ADC1->ISR & ADC_FLAG_EOC)) { }
  return ADC1->DR;
}
static inline void adc_stop()
{
  ADC1->CR = (ADC1->CR & ~(ADC_CR_ADCAL | ADC_CR_ADSTP | ADC_CR_ADSTART | ADC_CR_ADDIS)) | ADC_CR_ADSTP;
  while (ADC1->CR & ADC_CR_ADSTART) { }
  ADC1->CR = (ADC1->CR & ~(ADC_CR_ADCAL | ADC_CR_ADSTP | ADC_CR_ADSTART | ADC_CR_ADDIS)) | ADC_CR_ADDIS;
  while (ADC1->CR & ADC_CR_ADEN) { }
}

#if REV == 1
  #define LED_OUT_PORT  GPIOB
  #define LED_OUT_PIN   (1 << 9)
#elif REV == 2
  #define LED_OUT_PORT  GPIOA
  #define LED_OUT_PIN   (1 << 5)
  #define CC1_ADC_CHANNEL 1
  #define CC2_ADC_CHANNEL 4
#endif

struct sensors_readings {
  uint32_t t;   // (LPS22HH, SHT30) Temperature (0.01 degC)
  uint32_t p;   // (LPS22HH) Pressure (Pa)
  uint32_t h;   // (SHT30) Humidity (0.01 %RH)
  uint32_t i;   // (BH1750FVI) Illuminance (lx)
  uint8_t c[4]; // Capacitive sensing
};
static struct sensors_readings last_readings;
static bool last_readings_valid = false;
static uint32_t frame_n = 0;

static inline uint16_t offs_clamp_16(uint32_t x, uint32_t offs)
{
  if (x < offs) return 0;
  x -= offs;
  return (x > 0xffff ? 0xffff : x);
}

#define TX_READINGS_LEN 14
static void fill_tx_readings(uint8_t buf[TX_READINGS_LEN])
{
  buf[ 0] = (frame_n >> 8) & 0xff;
  buf[ 1] = (frame_n >> 0) & 0xff;
  buf[ 2] = (last_readings.t >> 8) & 0xff;
  buf[ 3] = (last_readings.t >> 0) & 0xff;
  buf[ 4] = (last_readings.p >> 8) & 0xff;
  buf[ 5] = (last_readings.p >> 0) & 0xff;
  buf[ 6] = (last_readings.h >> 8) & 0xff;
  buf[ 7] = (last_readings.h >> 0) & 0xff;
  buf[ 8] = (last_readings.i >> 8) & 0xff;
  buf[ 9] = (last_readings.i >> 0) & 0xff;
  buf[10] = last_readings.c[0];
  buf[11] = last_readings.c[1];
  buf[12] = last_readings.c[2];
  buf[13] = last_readings.c[3];
}

int main()
{
  HAL_Init();

  // ======== GPIO ========
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef gpio_init;

  // SWD (PA13, PA14)
  gpio_init.Pin = GPIO_PIN_13 | GPIO_PIN_14;
  gpio_init.Mode = GPIO_MODE_AF_PP; // Pass over control to AF peripheral
  gpio_init.Alternate = GPIO_AF0_SWJ;
  gpio_init.Pull = GPIO_PULLUP;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio_init);

  // ======== Clocks ========
  // Voltage scaling mostly affects clocks (HSE, PLL) and flash latency
  // Refer to RM0454 Rev. 5
  // - Ch. 4.1.4, "Dynamic voltage scaling management"
  // - Ch. 3.3.4, "FLASH read access latency"
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitTypeDef osc_init = { 0 };
  osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  osc_init.HSIState = RCC_HSI_ON;
  osc_init.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  osc_init.PLL.PLLState = RCC_PLL_ON;
  osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  osc_init.PLL.PLLM = RCC_PLLM_DIV1;  // VCO input 16 MHz (2.66 ~ 16 MHz)
  osc_init.PLL.PLLN = 8;              // VCO output 128 MHz (64 ~ 344 MHz)
  osc_init.PLL.PLLP = RCC_PLLP_DIV2;  // PLLPCLK 64 MHz
  osc_init.PLL.PLLR = RCC_PLLR_DIV2;  // PLLRCLK 64 MHz
  HAL_RCC_OscConfig(&osc_init);

  RCC_ClkInitTypeDef clk_init = { 0 };
  clk_init.ClockType =
    RCC_CLOCKTYPE_SYSCLK |
    RCC_CLOCKTYPE_HCLK |
    RCC_CLOCKTYPE_PCLK1;
  clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  // 64 MHz
  clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;         // 64 MHz
  clk_init.APB1CLKDivider = RCC_HCLK_DIV1;          // 64 MHz
  HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_2);

#if REV == 2
  // ======== UART ========
  HAL_GPIO_Init(GPIOA, &(GPIO_InitTypeDef){
    .Pin = GPIO_PIN_2 | GPIO_PIN_3,
    .Mode = GPIO_MODE_AF_PP,
    .Alternate = GPIO_AF1_USART2,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  });
  __HAL_RCC_USART2_CLK_ENABLE();
  uart2 = (UART_HandleTypeDef){
    .Instance = USART2,
    .Init = (UART_InitTypeDef){
      .BaudRate = 921600,
      .WordLength = UART_WORDLENGTH_8B,
      .StopBits = UART_STOPBITS_1,
      .Parity = UART_PARITY_NONE,
      .Mode = UART_MODE_TX_RX,
      .HwFlowCtl = UART_HWCONTROL_NONE,
      .OverSampling = UART_OVERSAMPLING_16,
    },
  };
  HAL_UART_Init(&uart2);
  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  HAL_UART_Receive_IT(&uart2, &rx_byte, 1);
#endif

  // ======== Capacitive touch sensing ========
  // BTN_OUT
  gpio_init = (GPIO_InitTypeDef){
    .Pin = BTN_OUT_PIN,
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  };
  HAL_GPIO_Init(BTN_OUT_PORT, &gpio_init);
  HAL_GPIO_WritePin(BTN_OUT_PORT, BTN_OUT_PIN, 0);

  // For the electrodes, refer to `pull_electrodes()`

  uint32_t cc_mV_max = 0;
#if defined(CC1_ADC_CHANNEL) && defined(CC2_ADC_CHANNEL)
{
  // ======== ADC for CC pins ========
  __HAL_RCC_ADC_CLK_ENABLE();
  ADC_HandleTypeDef adc1 = {
    .Instance = ADC1,
    .Init = {
      .ClockPrescaler = ADC_CLOCK_ASYNC_DIV1,
      .Resolution = ADC_RESOLUTION_12B,
      .DataAlign = ADC_DATAALIGN_RIGHT,
      .ScanConvMode = ADC_SCAN_SEQ_FIXED,
      .EOCSelection = ADC_EOC_SINGLE_CONV,
      .LowPowerAutoWait = DISABLE,
      .LowPowerAutoPowerOff = ENABLE,
      .ContinuousConvMode = DISABLE,
      .NbrOfConversion = 1,
      .DiscontinuousConvMode = DISABLE,
      .ExternalTrigConv = ADC_SOFTWARE_START,
      .TriggerFrequencyMode = ADC_TRIGGER_FREQ_LOW,
    },
  };
  HAL_ADC_Init(&adc1);
  HAL_ADCEx_Calibration_Start(&adc1);
  adc_stop();

  ADC1->SMPR = 0b110; // 79.5 cycles

  // Internal reference
  ADC1_COMMON->CCR |= ADC_CCR_VREFEN;
  ADC1->CHSELR = (1 << 13);
  adc_start();
  uint32_t adc_vrefint = adc_poll_value();
  adc_stop();
  ADC1_COMMON->CCR &= ~ADC_CCR_VREFEN;
  // CC1
  ADC1->CHSELR = (1 << CC1_ADC_CHANNEL);
  adc_start();
  uint32_t adc_cc1 = adc_poll_value();
  adc_stop();
  // CC2
  ADC1->CHSELR = (1 << CC2_ADC_CHANNEL);
  adc_start();
  uint32_t adc_cc2 = adc_poll_value();
  adc_stop();

  uint32_t vrefint_cal = *VREFINT_CAL_ADDR;
  uint32_t cc1_mV = (uint32_t)(3000ULL * adc_cc1 * vrefint_cal / (4095 * adc_vrefint));
  uint32_t cc2_mV = (uint32_t)(3000ULL * adc_cc2 * vrefint_cal / (4095 * adc_vrefint));
  cc_mV_max = (cc1_mV > cc2_mV ? cc1_mV : cc2_mV);
  swv_printf("CC voltage %lu | %lu %lu %lu\n", cc_mV_max, adc_cc1, adc_cc2, adc_vrefint);

  __HAL_RCC_ADC_CLK_DISABLE();
}
#endif

  // ======== Timer ========
  __HAL_RCC_TIM3_CLK_ENABLE();
  tim3 = (TIM_HandleTypeDef){
    .Instance = TIM3,
    .Init = {
      .Prescaler = 1 - 1,
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 80 - 1, // 800 kHz
      .ClockDivision = TIM_CLOCKDIVISION_DIV1,
      .RepetitionCounter = 0,
    },
  };
  HAL_TIM_Base_Init(&tim3);
  HAL_TIM_Base_Start(&tim3);

  // ======== I2C ========
  __HAL_RCC_I2C1_CLK_ENABLE();
  i2c1 = (I2C_HandleTypeDef){
    .Instance = I2C1,
    .Init = {
      // RM0454 Rev 5, pp. 711, 726, 738 (examples), 766
      // APB = 64 MHz     -- 0.015625 us
      // PRESC = 15       -- 0.25 us
      // SCLH = SCLL = 4  -- 1.25 us  (400 kHz)
      // SCLDEL = SDADEL = 1
      // <PRESC>0<SCLDEL><SDADEL><SCLH>~<SCLL>~
      .Timing = 0xF0110404,
      .AddressingMode = I2C_ADDRESSINGMODE_7BIT,
    },
  };
  HAL_I2C_Init(&i2c1);
  HAL_GPIO_Init(GPIOB, &(GPIO_InitTypeDef){
    .Pin = GPIO_PIN_6 | GPIO_PIN_7,
    .Alternate = GPIO_AF6_I2C1,
    .Mode = GPIO_MODE_AF_OD,
    .Pull = GPIO_PULLUP,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  });

  // Cannot be a static assertion, since this would require a link-time check
  if (STORAGE_START_ADDR + STORAGE_SIZE > FLASH_END_ADDR) {
    swv_printf("Insufficient storage!\n");
  }
  swv_printf("data end = 0x%08lx, storage start = 0x%08lx, page = %u\n",
    FW_END_ADDR, STORAGE_START_ADDR, FLASH_PAGE_SIZE);
if (0) {
  __attribute__ ((aligned(8)))
  static uint8_t data[257];
  memcpy(data, (const uint8_t []){0x4a, 0x30, 0x20, 0x10, 0x55, 0xaa, 0x55, 0xaa, 0x77}, 9);
  storage_write(0, data);
  memset(data, 0, 256);
  storage_write(256, data);
  storage_write(512, data);
  storage_write(768, data);
  for (uint32_t w = 1024; w < 3840; w += 256) {
    for (uint32_t i = 0; i < 16; i++)
      snprintf((char *)(data + i * 16), 17, "test - %08lx\n", w + (i * 16));
    storage_write(w, data);
  }
  data[252] = 0x09; data[253] = 0x00; // 9 bytes
  data[254] = 0xa4; data[255] = 0x06; // 1700 bytes
  storage_write(3840, data);
}

  bool check_device_ready(uint8_t addr, const char *name)
  {
    HAL_StatusTypeDef device_ready = HAL_I2C_IsDeviceReady(&i2c1, addr, 3, 1000);
    if (device_ready != HAL_OK) {
      swv_printf("%s %lu (%lu)\n", name, (uint32_t)device_ready, (uint32_t)i2c1.ErrorCode);
      return false;
    }
    return true;
  }
  check_device_ready(0b0100011 << 1, "BH1750FVI");
  check_device_ready(0b1000100 << 1, "SHT30");
  check_device_ready(0b1011100 << 1, "LPS22HH");

  void sensors_start()
  {
    uint32_t result;

    // LPS22HH: One-shot
    // CTRL_REG2 = IF_ADD_INC | ONE_SHOT
    result = HAL_I2C_Mem_Write(&i2c1, 0b1011100 << 1, 0x11, I2C_MEMADD_SIZE_8BIT, (uint8_t []){0b00010001}, 1, 1000);
    if (result != HAL_OK) swv_printf("LPS22HH write %lu %lu\n", result, i2c1.ErrorCode);

    // SHT30: Clock stretching disabled, high repeatability
    result = HAL_I2C_Master_Transmit(&i2c1, 0b1000100 << 1, (uint8_t []){0x24, 0x00}, 2, 1000);
    if (result != HAL_OK) swv_printf("SHT30 write %lu %lu\n", result, i2c1.ErrorCode);

    // BH1750FVI: One Time L-Resolution Mode
    result = HAL_I2C_Master_Transmit(&i2c1, 0b0100011 << 1, (uint8_t []){0b00100011}, 1, 1000);
    if (result != HAL_OK) swv_printf("BH1750FVI write %lu %lu\n", result, i2c1.ErrorCode);
  }

  bool sensors_read(struct sensors_readings *r)
  {
    uint32_t result;
    uint8_t buf[10];

    // LPS22HH
    result = HAL_I2C_Mem_Read(&i2c1, 0b1011100 << 1, 0x27, I2C_MEMADD_SIZE_8BIT, buf, 6, 1000);
    if (result != HAL_OK) return false;
    uint8_t status = buf[0] & 0x03; // STATUS should have T_DA | P_DA set
    if (status != 0x03) return false;
    uint32_t reading_p  =
      ((uint32_t)buf[1] <<  0) |
      ((uint32_t)buf[2] <<  8) |
      ((uint32_t)buf[3] << 16);
    uint32_t reading_t1 =
      ((uint32_t)buf[4] <<  0) |
      ((uint32_t)buf[5] <<  8);
    uint32_t p  = reading_p * 100 / 4096;
    uint32_t t1 = reading_t1;

    // SHT30
    result = HAL_I2C_Master_Receive(&i2c1, 0b1000100 << 1, buf, 6, 1000);
    uint32_t reading_t2 = (((uint32_t)buf[0] << 8) | buf[1]);
    uint32_t reading_h  = (((uint32_t)buf[3] << 8) | buf[4]);
    uint32_t t2 = -4500 + reading_t2 * 17500 / 65535;
    uint32_t h  = 10000 * reading_h / 65535;

    // BH1750FVI
    result = HAL_I2C_Master_Receive(&i2c1, 0b0100011 << 1, buf, 2, 1000);
    if (result != HAL_OK) return false;
    uint32_t i = ((((uint32_t)buf[0] << 8) | buf[1]) * 5 + 3) / 6;

    uint16_t cap[4];
    cap_sense(cap);

    uint32_t t = (t1 + t2) / 2;
    swv_printf("p=%lu t=%lu %lu h=%lu i=%lu | ", p, t1, t2, h, i);
    for (int j = 0; j < 4; j++) swv_printf("%3d%c", cap[j] < 999 ? cap[j] : 999, j == 3 ? '\n' : ' ');

    r->t = offs_clamp_16(t, 0);
    r->p = offs_clamp_16(p, 65536);
    r->h = offs_clamp_16(h, 0);
    r->i = offs_clamp_16(i, 0);
    for (int j = 0; j < 4; j++) r->c[j] = (cap[j] > 0xff ? 0xff : cap[j]);
    return true;
  }

if (1) {
  static mumu_vm_t m;

  static const uint32_t c[] = {
    0x30010000, // movi 1, 0
    0x30020001, // movi 2, 1
    0x30040001, // movi 4, 1
    0x30050000, // movi 5, 0
    0x86000404, // brile 0, 4, Bf
                // A:
    0x11010102, // add 1, 2
    0x11020201, // add 2, 1
    0x32000002, // subi 0, 2
    0x830004fc, // brigt 0, 4, Ab
                // B:
    0x81000502, // brie 0, 5, Cf
    0x10000202, // mov 0, 2
    0x00000000, // sc 0
                // C:
    0x10000101, // mov 0, 1
    0x00000000, // sc 0
  };

  swv_printf("running!\n");
  m.c = c;
  uint32_t t0 = HAL_GetTick();
  for (int i = 0; i < 10000; i++) {
    m.pc = 0;
    m.m[0] = 15;
    mumu_run(&m); // 36 instructions
  }
  swv_printf("%lu\n", HAL_GetTick() - t0);  // 352 = 1M instructions per second
  swv_printf("%08lx\n", m.m[0]);  // 610 = 0x00000262
}

  HAL_GPIO_WritePin(LED_OUT_PORT, LED_OUT_PIN, 1);
  HAL_GPIO_Init(LED_OUT_PORT, &(GPIO_InitTypeDef){
    .Pin = LED_OUT_PIN,
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  });

  sensors_start();

  uint32_t tick = HAL_GetTick();
  uint32_t last_sensors_start = tick;
  uint32_t last_sensors_tx = tick - 1000;
  while (1) {
    __disable_irq();
    run();
    __enable_irq();

    uint32_t cur = HAL_GetTick();
    if (cur >= last_sensors_start + 20) {
      struct sensors_readings r;
      bool valid = sensors_read(&r);
      if (valid) {
        sensors_start();
        last_sensors_start = cur;
        last_readings = r;
        last_readings_valid = true;
        if (cur - last_sensors_tx >= 100) {
          tx_readings_if_connected();
          last_sensors_tx = cur;
        }
      } else {
        swv_printf("Reading invalid! Check connections\n");
      }
    }
    while ((cur = HAL_GetTick()) - tick < 10)
      HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    tick = cur;
  }
}

// Within tolerance of both WS2812 and WS2812B
#define OUTPUT_BIT(_port, _bit, _set) do { \
  uint32_t bit = (_bit); \
  uint32_t wait = ((_set) ? 48 : 24); \
  while ((TIM3->SR & TIM_SR_UIF) == 0) { } \
  TIM3->SR = ~TIM_SR_UIF; \
  (_port)->BSRR = bit; \
  while (TIM3->CNT < wait) { } \
  (_port)->BSRR = bit << 16; \
} while (0)

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__ ((section(".RamFunc")))
void run()
{
  frame_n++;

  static const uint8_t seq[9][3] = {
    {3, 0, 0},
    {2, 1, 0},
    {1, 2, 0},
    {0, 3, 0},
    {0, 2, 1},
    {0, 1, 2},
    {0, 0, 3},
    {1, 0, 2},
    {2, 0, 1},
  };
  static const uint8_t seq_len = (sizeof seq) / (sizeof seq[0]);

#define N 24
  static uint32_t frame = 0, frame_subdiv = 0;
  if (++frame_subdiv == 8) {
    frame_subdiv = 0;
    frame++;
    if (frame == seq_len /* N + 2 */) frame = 0;
  }

  uint8_t buf[N][3];
  for (int i = 0, f = frame; i < N; i++, f = (f == seq_len - 1 ? 0 : f + 1)) {
    uint8_t g = 0, r = 0, b = 0;

    uint8_t f1 = (f == seq_len - 1 ? 0 : f + 1);
    g = (seq[f][0] * (8 - frame_subdiv) + seq[f1][0] * frame_subdiv) + 1;
    r = (seq[f][1] * (8 - frame_subdiv) + seq[f1][1] * frame_subdiv) + 1;
    b = (seq[f][2] * (8 - frame_subdiv) + seq[f1][2] * frame_subdiv) + 1;

    buf[i][0] = g * 5;
    buf[i][1] = r * 5;
    buf[i][2] = b * 5;
  }
if (0) {
  for (int i = 0; i < N; i++) {
    buf[i][0] = buf[i][1] = buf[i][2] = 0;
  }
  for (int i = 0; i < N; i++) if (i >= (int)frame - 2 && i <= frame) {
    uint8_t value = 120;
    if (i == (int)frame - 2) value = (7 - frame_subdiv) * 15;
    if (i == frame + 0) value = frame_subdiv * 15;
    buf[i][0] = value >> 1;
    buf[i][2] = value;
  }
}

  LED_OUT_PORT->BSRR = LED_OUT_PIN << 16; // Reset code, drive low
  delay_us(60); // Nominal length is 50 us, leave some tolerance

  TIM3->SR = ~TIM_SR_UIF;
  for (int i = 0; i < N; i++) {
    // G
    uint8_t g = buf[i][0];
    OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, 0);
    OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, g & 64);
    OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, g & 32);
    OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, g & 16);
    OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, g & 8);
    OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, g & 4);
    OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, g & 2);
    OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, g & 1);
    // R
    uint8_t r = buf[i][1];
    OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, 0);
    OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, r & 64);
    OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, r & 32);
    OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, r & 16);
    OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, r & 8);
    OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, r & 4);
    OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, r & 2);
    OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, r & 1);
    // B
    uint8_t b = buf[i][2];
    OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, 0);
    OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, b & 64);
    OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, b & 32);
    OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, b & 16);
    OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, b & 8);
    OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, b & 4);
    OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, b & 2);
    OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, b & 1);
  }

  LED_OUT_PORT->BSRR = LED_OUT_PIN; // Release line, write high, to avoid continuous current
}
#pragma GCC pop_options

// For Rev. 2, this is set with any well-formatted packet and cleared with a goodbyte packet (0xBB).
// Turned off during packet reception; stays off after a broken packet, but
// transmission restarts once re-initialisation happens.
// For future revisions, this can be based on CH343's ACT# signal or possibly hardware flow control.
static bool rx_connected = false;

static uint8_t rx_len = 0;
static uint8_t rx_buf[256];
static uint8_t rx_ptr = 0;

static uint8_t encode_len(uint8_t *a, uint16_t x)
{
  if (x < 128) { a[0] = x; return 1; }
  else { a[0] = 128 | (x / 256); a[1] = x % 256; return 2; }
}

static uint8_t rx_flash = 0;  // 0 - inactive, 1 - ROM, 2 - source
static uint16_t rx_rom_len, rx_src_len;
static uint8_t flash_data[256];

#pragma GCC push_options
#pragma GCC optimize("O3")
static void serial_rx_process_byte(uint8_t c)
{
  static uint32_t last_timestamp = (uint32_t)-3;
  uint32_t t = HAL_GetTick();
  if (t - last_timestamp >= 3) {
    // Reset
    rx_len = 0;
  }
  last_timestamp = t;

  if (rx_len == 0) {
    if (c == 0) {
      // Ignore empty packet
    } else {
      // Receive payload
      rx_len = c;
      rx_ptr = 0;
      rx_connected = false;
    }
  } else {
    rx_buf[rx_ptr++] = c;
    if (rx_ptr == rx_len) {
      // Packet complete! Process payload
      rx_connected = true;

      if (rx_len == 1 && rx_buf[0] == 0xAA) {
        delay_us(1000);
        // Transmit first readings
        uint8_t readings_buf[TX_READINGS_LEN];
        fill_tx_readings(readings_buf);
        HAL_UART_Transmit(&uart2, &(uint8_t){ 11 + TX_READINGS_LEN }, 1, HAL_MAX_DELAY);
        HAL_UART_Transmit(&uart2, (uint8_t *)"\x55" "Amakinoko" "\x20", 11, HAL_MAX_DELAY);
        HAL_UART_Transmit(&uart2, readings_buf, TX_READINGS_LEN, HAL_MAX_DELAY);
        // Transmit program binary and source text
        uint8_t l[2];
        uint16_t rom_size = *STORAGE_ROM_LEN;
        uint16_t src_size = *STORAGE_SRC_LEN;
        if (rom_size > MUMU_ROM_SIZE || src_size > STORAGE_SRC_MAX_LEN)
          rom_size = src_size = 0;
        HAL_UART_Transmit(&uart2, l, encode_len(l, rom_size), HAL_MAX_DELAY);
        HAL_UART_Transmit(&uart2, STORAGE_ROM_START, rom_size, HAL_MAX_DELAY);
        HAL_UART_Transmit(&uart2, l, encode_len(l, src_size), HAL_MAX_DELAY);
        HAL_UART_Transmit(&uart2, STORAGE_SRC_START, src_size, HAL_MAX_DELAY);
      }

      else if (rx_len == 1 && rx_buf[0] == 0xBB) {
        rx_connected = false;
      }

      // Wait for next packet
      rx_len = 0;
    }
  }
}
#pragma GCC pop_options

void tx_readings_if_connected()
{
  if (!rx_connected) return;

  uint8_t readings_buf[TX_READINGS_LEN];
  fill_tx_readings(readings_buf);
  __disable_irq();
  HAL_UART_Transmit(&uart2, (uint8_t []){ TX_READINGS_LEN + 1 }, 1, HAL_MAX_DELAY);
  HAL_UART_Transmit(&uart2, (uint8_t []){ 0x56 }, 1, HAL_MAX_DELAY);
  HAL_UART_Transmit(&uart2, readings_buf, TX_READINGS_LEN, HAL_MAX_DELAY);
  __enable_irq();
}

void SysTick_Handler()
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

void USART2_IRQHandler()
{
  HAL_UART_IRQHandler(&uart2);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *_uart2)
{
  serial_rx_process_byte(rx_byte);
  HAL_UART_Receive_IT(&uart2, &rx_byte, 1);
}

void NMI_Handler() { while (1) { } }
void HardFault_Handler() { while (1) { } }
void SVC_Handler() { while (1) { } }
void PendSV_Handler() { while (1) { } }
void WWDG_IRQHandler() { while (1) { } }
void RTC_TAMP_IRQHandler() { while (1) { } }
void FLASH_IRQHandler() { while (1) { } }
void RCC_IRQHandler() { while (1) { } }
void EXTI0_1_IRQHandler() { while (1) { } }
void EXTI2_3_IRQHandler() { while (1) { } }
void EXTI4_15_IRQHandler() { while (1) { } }
void DMA1_Channel1_IRQHandler() { while (1) { } }
void DMA1_Channel2_3_IRQHandler() { while (1) { } }
void DMA1_Ch4_5_DMAMUX1_OVR_IRQHandler() { while (1) { } }
void ADC1_IRQHandler() { while (1) { } }
void TIM1_BRK_UP_TRG_COM_IRQHandler() { while (1) { } }
void TIM1_CC_IRQHandler() { while (1) { } }
void TIM3_IRQHandler() { while (1) { } }
void TIM14_IRQHandler() { while (1) { } }
void TIM16_IRQHandler() { while (1) { } }
void TIM17_IRQHandler() { while (1) { } }
void I2C1_IRQHandler() { while (1) { } }
void I2C2_IRQHandler() { while (1) { } }
void SPI1_IRQHandler() { while (1) { } }
void SPI2_IRQHandler() { while (1) { } }
void USART1_IRQHandler() { while (1) { } }
