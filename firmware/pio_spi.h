#pragma once

#include <hardware/pio.h>
#include "spi.pio.h"

#define SM_DAC 0
#define SM_ADC 1

// struct for defining pio interface
typedef struct {
    PIO pio;
    bool sm_active;
    uint offset_dac;
    uint offset_adc;
    const uint8_t miso;
    const uint8_t mosi;
    const uint8_t sck;
    const uint8_t cs_dac;
    const uint8_t cs_adc;
    const uint8_t base;
} pio_spi_t;

// not time-critical functions for initialization

void pio_spi_init(pio_spi_t *pio_spi_ch);
bool dac_write_config(pio_spi_t *ch, uint32_t src);

// time-critical inline functions for pulse train timing

static inline bool pio_spi_is_done(const pio_spi_t *ch) {
    return (ch->pio->fdebug >> (PIO_FDEBUG_TXSTALL_LSB + ch->sm_active)) & 1u;
}

static inline void pio_spi_wait_done(const pio_spi_t *ch) {
    while (!pio_spi_is_done(ch))
        tight_loop_contents();
}

static inline void pio_spi_select_dac(pio_spi_t *ch) {
    pio_sm_set_enabled(ch->pio, SM_DAC, false);
    ch->sm_active = SM_DAC;
    pio_sm_restart(ch->pio, SM_DAC);
    pio_sm_exec(ch->pio, SM_DAC, pio_encode_jmp(ch->offset_dac));
    ch->pio->fdebug = (1u << (PIO_FDEBUG_TXSTALL_LSB + SM_DAC));
    gpio_put(ch->cs_dac, 0);
}

static inline void pio_spi_select_adc(pio_spi_t *ch) {
    pio_sm_set_enabled(ch->pio, SM_ADC, false);
    ch->sm_active = SM_ADC;
    while (!pio_sm_is_rx_fifo_empty(ch->pio, SM_ADC))
        (void)pio_sm_get(ch->pio, SM_ADC);
    pio_sm_restart(ch->pio, SM_ADC);
    pio_sm_exec(ch->pio, SM_ADC, pio_encode_jmp(ch->offset_adc));
    ch->pio->fdebug = (1u << (PIO_FDEBUG_TXSTALL_LSB + SM_ADC));
    gpio_put(ch->cs_adc, 0);
}

static inline void pio_spi_deselect_dac(pio_spi_t *ch) {
    gpio_put(ch->cs_dac, 1);
    pio_sm_set_enabled(ch->pio, SM_DAC, false);
}

static inline void pio_spi_deselect_adc(pio_spi_t *ch) {
    gpio_put(ch->cs_adc, 1);
    pio_sm_set_enabled(ch->pio, SM_ADC, false);
}

static inline bool adc_get_value(const pio_spi_t *ch, int16_t *dst) {
    if (pio_sm_is_rx_fifo_empty(ch->pio, ch->sm_active)) return false;
    *dst = ((int16_t)((uint16_t)pio_sm_get(ch->pio, ch->sm_active) << 3)) >> 3; // sign-extend
    return true;
}

static inline bool adc_read(pio_spi_t *ch) {
    if (pio_sm_is_tx_fifo_full(ch->pio, ch->sm_active))
        return false;
    pio_sm_put(ch->pio, ch->sm_active, 0);
    pio_sm_set_enabled(ch->pio, ch->sm_active, true);
    return true;
}

static inline bool adc_write(pio_spi_t *ch, const uint16_t src) {
    if (pio_sm_is_tx_fifo_full(ch->pio, ch->sm_active)) 
        return false;

    pio_sm_put(ch->pio, ch->sm_active, (uint32_t)src << 16);
    pio_sm_set_enabled(ch->pio, ch->sm_active, true);
    return true;
}

// dac_write_config is the more general case of dac_write_output which only
// accepts 16-bit numbers. The benefit of using dac_write_output is that you
// don't have to cast the src argument with (uint16_t) to transmit negative
// values correctly.
static inline bool dac_write_output(pio_spi_t *ch, int16_t src) {
    pio_sm_put(ch->pio, ch->sm_active, (uint16_t)src << 8);
    pio_sm_set_enabled(ch->pio, ch->sm_active, true);
    return true;
}