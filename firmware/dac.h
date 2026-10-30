#pragma once

#include "pio_spi.h"
#include <hardware/gpio.h>
#include <stdint.h>

#define DAC_REG_RANGE     (1u << 3)
#define DAC_RANGE_PM5V    3u
#define DAC_REG_POWER     (2u << 3)
#define DAC_POWER_ON      1u
#define DAC_REG_CTRL      (3u << 3)
#define DAC_CFG(reg, val) (((uint32_t)(reg) << 16) | (uint32_t)(val))

__always_inline static inline void dacs_latch(const uint64_t nldac_mask) {
    gpio_clr_mask64(nldac_mask);
    // hold low ≥3 cycles to meet DAC min NLDAC pulse width requirement
    __asm volatile("nop\n\t" "nop\n\t" "nop\n\t");
    gpio_set_mask64(nldac_mask);
}

__always_inline static inline void dac_write(uint8_t ch, const uint32_t src) {
    PIO pio = pio_spi_cfg[ch].pio;
    uint8_t sm = pio_spi_state[ch].sm_active;
    while (pio_sm_is_tx_fifo_full(pio, sm));
    pio_sm_put(pio, sm, src << 8);
    pio_sm_set_enabled(pio, sm, true);
}

__always_inline static inline void dacs_write_blocking(const uint32_t val0, const uint32_t val1) {
    pio_spi_select_dac(0);
    pio_spi_select_dac(1);
    dac_write(0, val0);
    dac_write(1, val1);
    while (!pio_spi_is_done(0) || !pio_spi_is_done(1));
    pio_spi_deselect_dac(0);
    pio_spi_deselect_dac(1);
}

void dac_write_blocking(uint8_t ch, const uint32_t val);
void dacs_init(void);
