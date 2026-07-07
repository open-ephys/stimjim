#pragma once

#include "pio_spi.h"
#include <hardware/gpio.h>
#include <stdint.h>

#define DAC_REG_RANGE     0x08u
#define DAC_RANGE_PM10V   4u
#define DAC_REG_POWER     0x10u
#define DAC_POWER_ON      1u
#define DAC_CFG(reg, val) (((uint32_t)(reg) << 16) | (uint32_t)(val))

__always_inline static inline void dacs_latch(void) {
    sio_hw->gpio_clr    = (1u << NLDAC_B);
    sio_hw->gpio_hi_clr = (1u << (NLDAC_A - 32u));
    // hold low ≥3 cycles to meet DAC min NLDAC pulse width requirement
    __asm volatile("nop\n\t" "nop\n\t" "nop\n\t");
    sio_hw->gpio_set    = (1u << NLDAC_B);
    sio_hw->gpio_hi_set = (1u << (NLDAC_A - 32u));
}

__always_inline static inline bool dac_write(uint8_t ch, const uint32_t src) {
    PIO pio = pio_spi_cfg[ch].pio;
    uint8_t sm = pio_spi_state[ch].sm_active;
    if (pio_sm_is_tx_fifo_full(pio, sm)) return false;
    pio_sm_put(pio, sm, src << 8);
    pio_sm_set_enabled(pio, sm, true);
    return true;
}

__always_inline static inline void dac_write_blocking(uint8_t ch, const uint32_t val) {
    pio_spi_select_dac(ch);
    dac_write(ch, val);
    while (!pio_spi_is_done(ch));
    pio_spi_deselect_dac(ch);
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

void dacs_init(void);
