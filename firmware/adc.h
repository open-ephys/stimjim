#pragma once

#include <hardware/spi.h>
#include <hardware/gpio.h>
#include <pico/time.h>
#include <stdbool.h>
#include "spi_inline.h"

#define MICROAMPS_PER_ADC (20e6 / (1.0 + (49.4 / 1.8)) / 100.0 / (float)(1 << 13)) // 20V / (100*(1+49.9k/1.8k))V/A / 2^13 = 0.85000 uA
#define MILLIVOLTS_PER_ADC (20e3 / (float)(1 << 13)) // 20V / 2^13 = 2.44141 mV
// #define ADC_BAUDRATE (10U * 1000U * 1000U)

__attribute__ ((always_inline)) static inline int16_t __time_critical_func(adc_read)(bool ch, bool line) {
  const uint8_t cs[2] = { CSB_A, CSB_B };
  spi_inst_t *spi[2] = { SPI_A, SPI_B };
  uint16_t rx, tx;

  // spi_set_baudrate(spi, ADC_BAUDRATE);
  spi_set_format(spi[ch], 16, SPI_CPOL_1, SPI_CPHA_0, SPI_MSB_FIRST);

  // configure adc and select which adc input from which to read
  gpio_put(cs[ch], false);
  tx = 0x8010 | (line << 10);
  spi_write16_blocking_inline(spi[ch], &tx, 1);
  gpio_put(cs[ch], true);

  // necessary, otherwise calibrations yield unreliable results
  for (uint32_t i = 0; i < 100; i++)
      asm volatile("nop\n\t");

  // read value from adc
  gpio_put(cs[ch], false);
  tx = 0;
  spi_write16_read16_blocking_inline(spi[ch], &tx, &rx, 1);
  gpio_put(cs[ch], true);

  return (int16_t)(rx << 3) >> 3; // extend sign bit
}