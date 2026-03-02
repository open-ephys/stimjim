#pragma once

#include <stdbool.h>
#include "spi_inline.h"

#define MICROAMPS_PER_DAC (20e6 / 3000.0 / (float)(1 << 16)) // 20V / (3000V/A) / 2^16 = 0.10173 uA
#define MILLIVOLTS_PER_DAC (20e3 * (1.0 + ( 5.0 / 10.0 )) / (float)(1 << 16)) // 20V * (1 + 5/10) / 2^16 = 0.45776 mV
// #define DAC_BAUDRATE (30U * 1000U * 1000U)

void dac_init(bool ch);
void dac_update_output(bool ch, int16_t amp);

__attribute__ ((always_inline)) static inline void __time_critical_func(dac_latch)(bool ch) {
  const uint8_t nldac[2] = { NLDAC_A, NLDAC_B };
  gpio_put(nldac[ch], false);
  asm volatile("nop\n\t" "nop\n\t" "nop\n\t");
  gpio_put(nldac[ch], true);
}

__attribute__ ((always_inline)) static inline void __time_critical_func(dac_write)(bool ch, uint8_t reg, uint16_t data) {
  const uint8_t cs[2] = { CSA_A, CSA_B };
  const uint8_t buffer[3] = {reg, (uint8_t)(data >> 8), (uint8_t)data};
  spi_inst_t *spi[2] = { SPI_A, SPI_B };
  spi_set_format(spi[ch], 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
  gpio_put(cs[ch], false);
  spi_write_blocking_inline(spi[ch], buffer, 3);
  gpio_put(cs[ch], true);
}