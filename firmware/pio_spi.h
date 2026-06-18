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

// time-critical inline functions for pulse train timing

__always_inline static inline bool pio_spi_is_done(const pio_spi_t *ch) {
    return (ch->pio->fdebug >> (PIO_FDEBUG_TXSTALL_LSB + ch->sm_active)) & 1u;
}

__always_inline static inline void pio_spi_wait_done(const pio_spi_t *ch) {
    while (!pio_spi_is_done(ch))
        tight_loop_contents();
}

__always_inline static inline void pio_spi_select_dac(pio_spi_t *ch) {
    pio_sm_set_enabled(ch->pio, SM_DAC, false);
    ch->sm_active = SM_DAC;
    pio_sm_restart(ch->pio, SM_DAC);
    pio_sm_exec(ch->pio, SM_DAC, pio_encode_jmp(ch->offset_dac));
    ch->pio->fdebug = (1u << (PIO_FDEBUG_TXSTALL_LSB + SM_DAC));
    gpio_put(ch->cs_dac, 0);
}

__always_inline static inline void pio_spi_select_adc(pio_spi_t *ch) {
    pio_sm_set_enabled(ch->pio, SM_ADC, false);
    ch->sm_active = SM_ADC;
    while (!pio_sm_is_rx_fifo_empty(ch->pio, SM_ADC))
        (void)pio_sm_get(ch->pio, SM_ADC);
    pio_sm_restart(ch->pio, SM_ADC);
    pio_sm_exec(ch->pio, SM_ADC, pio_encode_jmp(ch->offset_adc));
    ch->pio->fdebug = (1u << (PIO_FDEBUG_TXSTALL_LSB + SM_ADC));
    gpio_put(ch->cs_adc, 0);
}

__always_inline static inline void pio_spi_deselect_dac(const pio_spi_t *ch) {
    gpio_put(ch->cs_dac, 1);
    pio_sm_set_enabled(ch->pio, SM_DAC, false);
}

__always_inline static inline void pio_spi_deselect_adc(const pio_spi_t *ch) {
    gpio_put(ch->cs_adc, 1);
    pio_sm_set_enabled(ch->pio, SM_ADC, false);
}

