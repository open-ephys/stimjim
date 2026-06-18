#pragma once

#include "pio_spi.h"
#include <pico/time.h>
#include <stdint.h>

#define ADC_BASE_CONFIG  0x8010u
#define ADC_CURRENT_LINE 0x0400u

__always_inline static inline bool adc_get_value(const pio_spi_t *ch, int16_t *dst) {
    if (pio_sm_is_rx_fifo_empty(ch->pio, ch->sm_active)) return false;
    *dst = ((int16_t)((uint16_t)pio_sm_get(ch->pio, ch->sm_active) << 3)) >> 3;
    return true;
}

__always_inline static inline bool adc_read(const pio_spi_t *ch) {
    if (pio_sm_is_tx_fifo_full(ch->pio, ch->sm_active)) return false;
    pio_sm_put(ch->pio, ch->sm_active, 0);
    pio_sm_set_enabled(ch->pio, ch->sm_active, true);
    return true;
}

__always_inline static inline bool adc_write(const pio_spi_t *ch, const uint16_t src) {
    if (pio_sm_is_tx_fifo_full(ch->pio, ch->sm_active)) return false;
    pio_sm_put(ch->pio, ch->sm_active, (uint32_t)src << 16);
    pio_sm_set_enabled(ch->pio, ch->sm_active, true);
    return true;
}

__always_inline static inline void adc_write_blocking(pio_spi_t *p, const uint16_t config) {
    pio_spi_select_adc(p);
    adc_write(p, config);
    while (!pio_spi_is_done(p));
    pio_spi_deselect_adc(p);
}

__always_inline static inline int16_t adc_read_blocking(pio_spi_t *p) {
    int16_t val;
    pio_spi_select_adc(p);
    adc_read(p);
    while (!pio_spi_is_done(p));
    adc_get_value(p, &val);
    pio_spi_deselect_adc(p);
    sleep_us(2);
    return val;
}

__always_inline static inline void adcs_write_blocking(pio_spi_t *p, const uint16_t cfg0, const uint16_t cfg1) {
    pio_spi_select_adc(&p[0]);
    pio_spi_select_adc(&p[1]);
    adc_write(&p[0], cfg0);
    adc_write(&p[1], cfg1);
    while (!pio_spi_is_done(&p[0]) || !pio_spi_is_done(&p[1]));
    pio_spi_deselect_adc(&p[0]);
    pio_spi_deselect_adc(&p[1]);
}

__always_inline static inline void adcs_read_get_value_blocking(pio_spi_t *p, int16_t *vals) {
    pio_spi_select_adc(&p[0]);
    pio_spi_select_adc(&p[1]);
    adc_read(&p[0]);
    adc_read(&p[1]);
    while (!pio_spi_is_done(&p[0]) || !pio_spi_is_done(&p[1]));
    adc_get_value(&p[0], &vals[0]);
    adc_get_value(&p[1], &vals[1]);
    pio_spi_deselect_adc(&p[0]);
    pio_spi_deselect_adc(&p[1]);
}
