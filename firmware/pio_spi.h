#pragma once

#include <hardware/pio.h>
#include "spi.pio.h"

#define SM_DAC 0
#define SM_ADC 1

typedef struct {
    PIO pio;
    uint8_t miso, mosi, sck;
    uint8_t cs_dac, cs_adc;
    uint8_t base;
} pio_spi_cfg_t;

typedef struct {
    uint8_t sm_active;
    uint offset_dac;
    uint offset_adc;
} pio_spi_state_t;

extern const pio_spi_cfg_t pio_spi_cfg[2];
extern pio_spi_state_t pio_spi_state[2];

void pio_spi_init(uint8_t ch);

__always_inline static inline bool pio_spi_is_done(uint8_t ch) {
    return (pio_spi_cfg[ch].pio->fdebug >> (PIO_FDEBUG_TXSTALL_LSB + pio_spi_state[ch].sm_active)) & 1u;
}

__always_inline static inline void pio_spi_wait_done(uint8_t ch) {
    while (!pio_spi_is_done(ch))
        tight_loop_contents();
}

__always_inline static inline void pio_spi_select_dac(uint8_t ch) {
    const pio_spi_cfg_t *cfg = &pio_spi_cfg[ch];
    pio_spi_state_t *state = &pio_spi_state[ch];
    pio_sm_set_enabled(cfg->pio, SM_DAC, false);
    state->sm_active = SM_DAC;
    pio_sm_restart(cfg->pio, SM_DAC);
    pio_sm_exec(cfg->pio, SM_DAC, pio_encode_jmp(state->offset_dac));
    cfg->pio->fdebug = (1u << (PIO_FDEBUG_TXSTALL_LSB + SM_DAC));
    gpio_put(cfg->cs_dac, 0);
}

__always_inline static inline void pio_spi_select_adc(uint8_t ch) {
    const pio_spi_cfg_t *cfg = &pio_spi_cfg[ch];
    pio_spi_state_t *state = &pio_spi_state[ch];
    pio_sm_set_enabled(cfg->pio, SM_ADC, false);
    state->sm_active = SM_ADC;
    while (!pio_sm_is_rx_fifo_empty(cfg->pio, SM_ADC))
        (void)pio_sm_get(cfg->pio, SM_ADC);
    pio_sm_restart(cfg->pio, SM_ADC);
    pio_sm_exec(cfg->pio, SM_ADC, pio_encode_jmp(state->offset_adc));
    cfg->pio->fdebug = (1u << (PIO_FDEBUG_TXSTALL_LSB + SM_ADC));
    gpio_put(cfg->cs_adc, 0);
}

__always_inline static inline void pio_spi_deselect_dac(uint8_t ch) {
    gpio_put(pio_spi_cfg[ch].cs_dac, 1);
    pio_sm_set_enabled(pio_spi_cfg[ch].pio, SM_DAC, false);
}

__always_inline static inline void pio_spi_deselect_adc(uint8_t ch) {
    gpio_put(pio_spi_cfg[ch].cs_adc, 1);
    pio_sm_set_enabled(pio_spi_cfg[ch].pio, SM_ADC, false);
}
