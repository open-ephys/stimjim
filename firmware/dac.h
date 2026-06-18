#pragma once

#include "pio_spi.h"
#include <hardware/gpio.h>
#include <stdint.h>

#define DAC_REG_RANGE     0x08u
#define DAC_RANGE_PM10V   4u
#define DAC_REG_POWER     0x10u
#define DAC_POWER_ON      1u
#define DAC_CFG(reg, val) (((uint32_t)(reg) << 16) | (uint32_t)(val))

__always_inline static inline void dacs_latch(const uint64_t mask) {
    gpio_clr_mask64(mask);
    // hold low ≥3 cycles to meet DAC min NLDAC pulse width requirement
    __asm volatile("nop\n\t" "nop\n\t" "nop\n\t");
    gpio_set_mask64(mask);
}

__always_inline static inline bool dac_write(const pio_spi_t *ch, const uint32_t src) {
    if (pio_sm_is_tx_fifo_full(ch->pio, ch->sm_active)) return false;
    pio_sm_put(ch->pio, ch->sm_active, src << 8);
    pio_sm_set_enabled(ch->pio, ch->sm_active, true);
    return true;
}

__always_inline static inline void dac_write_blocking(pio_spi_t *p, const uint32_t val) {
    pio_spi_select_dac(p);
    dac_write(p, val);
    while (!pio_spi_is_done(p));
    pio_spi_deselect_dac(p);
}

__always_inline static inline void dacs_write_blocking(pio_spi_t *p, const uint32_t val0, const uint32_t val1) {
    pio_spi_select_dac(&p[0]);
    pio_spi_select_dac(&p[1]);
    dac_write(&p[0], val0);
    dac_write(&p[1], val1);
    while (!pio_spi_is_done(&p[0]) || !pio_spi_is_done(&p[1]));
    pio_spi_deselect_dac(&p[0]);
    pio_spi_deselect_dac(&p[1]);
}

void dacs_init(pio_spi_t *p);
