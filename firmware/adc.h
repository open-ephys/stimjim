#pragma once

#include "pio_spi.h"
#include <pico/time.h>
#include <stdint.h>

#define ADC_BASE_CONFIG  0x8010u
#define ADC_CURRENT_LINE 0x0400u

__always_inline static inline bool adc_get_value(uint8_t ch, int16_t *dst) {
    PIO pio = pio_spi_cfg[ch].pio;
    uint8_t sm = pio_spi_state[ch].sm_active;
    if (pio_sm_is_rx_fifo_empty(pio, sm)) return false;
    *dst = ((int16_t)((uint16_t)pio_sm_get(pio, sm) << 3)) >> 3;
    return true;
}

__always_inline static inline bool adc_read(uint8_t ch) {
    PIO pio = pio_spi_cfg[ch].pio;
    uint8_t sm = pio_spi_state[ch].sm_active;
    if (pio_sm_is_tx_fifo_full(pio, sm)) return false;
    pio_sm_put(pio, sm, 0);
    pio_sm_set_enabled(pio, sm, true);
    return true;
}

__always_inline static inline bool adc_write(uint8_t ch, const uint16_t src) {
    PIO pio = pio_spi_cfg[ch].pio;
    uint8_t sm = pio_spi_state[ch].sm_active;
    if (pio_sm_is_tx_fifo_full(pio, sm)) return false;
    pio_sm_put(pio, sm, (uint32_t)src << 16);
    pio_sm_set_enabled(pio, sm, true);
    return true;
}

__always_inline static inline void adc_write_blocking(uint8_t ch, const uint16_t config) {
    pio_spi_select_adc(ch);
    adc_write(ch, config);
    while (!pio_spi_is_done(ch));
    pio_spi_deselect_adc(ch);
}

__always_inline static inline int16_t adc_read_blocking(uint8_t ch) {
    int16_t val;
    pio_spi_select_adc(ch);
    adc_read(ch);
    while (!pio_spi_is_done(ch));
    adc_get_value(ch, &val);
    pio_spi_deselect_adc(ch);
    sleep_us(2);
    return val;
}

__always_inline static inline void adcs_write_blocking(const uint16_t cfg0, const uint16_t cfg1) {
    pio_spi_select_adc(0);
    pio_spi_select_adc(1);
    adc_write(0, cfg0);
    adc_write(1, cfg1);
    while (!pio_spi_is_done(0) || !pio_spi_is_done(1));
    pio_spi_deselect_adc(0);
    pio_spi_deselect_adc(1);
}

__always_inline static inline void adcs_read_get_value_blocking(int16_t *vals) {
    pio_spi_select_adc(0);
    pio_spi_select_adc(1);
    adc_read(0);
    adc_read(1);
    while (!pio_spi_is_done(0) || !pio_spi_is_done(1));
    adc_get_value(0, &vals[0]);
    adc_get_value(1, &vals[1]);
    pio_spi_deselect_adc(0);
    pio_spi_deselect_adc(1);
}
