#include "pio_spi.h"

const pio_spi_cfg_t pio_spi_cfg[2] __attribute__((section(".data"))) = {
    {   .pio = pio0, .base = 16,
        .miso = SPI_MISO_A, .mosi = SPI_MOSI_A, .sck = SPI_SCK_A,
        .cs_dac = CSA_A, .cs_adc = CSB_A },
    {   .pio = pio1, .base = 0,
        .miso = SPI_MISO_B, .mosi = SPI_MOSI_B, .sck = SPI_SCK_B,
        .cs_dac = CSA_B, .cs_adc = CSB_B },
};

pio_spi_state_t pio_spi_state[2] = { 0 };

void pio_spi_init(uint8_t ch)
{
    const pio_spi_cfg_t *cfg = &pio_spi_cfg[ch];
    pio_spi_state_t *state = &pio_spi_state[ch];
    pio_set_gpio_base(cfg->pio, cfg->base);

    state->offset_dac = pio_add_program(cfg->pio, &pio_spi_mode1_program);
    state->offset_adc = pio_add_program(cfg->pio, &pio_spi_mode2_program);

    pio_spi_mode1_init(cfg->pio, 0, state->offset_dac, cfg->mosi, cfg->sck);
    pio_spi_mode2_init(cfg->pio, 1, state->offset_adc, cfg->miso, cfg->mosi, cfg->sck);
}
