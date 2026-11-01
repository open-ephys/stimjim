#pragma once

#include "pio_spi.h"
#include <pico/time.h>
#include <stdint.h>

#define AD7321_CTRL_WRITE (1u << 15)
#define AD7321_CTRL_ADD0  (1u << 10)
#define AD7321_CTRL_REF   (1u << 4)

#define ADC_BASE_CONFIG  (AD7321_CTRL_WRITE | AD7321_CTRL_REF)
#define ADC_CURRENT_LINE AD7321_CTRL_ADD0

__always_inline static inline bool adc_get_value(uint8_t ch, int16_t *dst) {
    PIO pio = pio_spi_cfg[ch].pio;
    uint8_t sm = pio_spi_state[ch].sm_active;
    if (pio_sm_is_rx_fifo_empty(pio, sm)) return false;
    *dst = ((int16_t)((uint16_t)pio_sm_get(pio, sm) << 3)) >> 3;
    return true;
}

__always_inline static inline void adc_read(uint8_t ch) {
    PIO pio = pio_spi_cfg[ch].pio;
    uint8_t sm = pio_spi_state[ch].sm_active;
    while (pio_sm_is_tx_fifo_full(pio, sm));
    pio_sm_put(pio, sm, 0);
    pio_sm_set_enabled(pio, sm, true);
}

__always_inline static inline void adc_write(uint8_t ch, const uint16_t src) {
    PIO pio = pio_spi_cfg[ch].pio;
    uint8_t sm = pio_spi_state[ch].sm_active;
    while (pio_sm_is_tx_fifo_full(pio, sm));
    pio_sm_put(pio, sm, (uint32_t)src << 16);
    pio_sm_set_enabled(pio, sm, true);
}

void adc_write_blocking(uint8_t ch, const uint16_t config);
bool adc_read_blocking(uint8_t ch, int16_t *val);
void adcs_write_blocking(const uint16_t cfg0, const uint16_t cfg1);

__always_inline static inline bool adcs_read_get_value_blocking(int16_t *vals) {
    pio_spi_select_adc(0);
    pio_spi_select_adc(1);
    adc_read(0);
    adc_read(1);
    while (!pio_spi_is_done(0) || !pio_spi_is_done(1));
    const bool ok0 = adc_get_value(0, &vals[0]);
    const bool ok1 = adc_get_value(1, &vals[1]);
    pio_spi_deselect_adc(0);
    pio_spi_deselect_adc(1);
    return ok0 && ok1;
}
