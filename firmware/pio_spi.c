#include "pio_spi.h"

void pio_spi_init(pio_spi_t *pio_spi_ch)
{
    pio_set_gpio_base(pio_spi_ch->pio, pio_spi_ch->base);

    pio_spi_ch->offset_dac = pio_add_program(pio_spi_ch->pio, &pio_spi_mode1_program);
    pio_spi_ch->offset_adc = pio_add_program(pio_spi_ch->pio, &pio_spi_mode2_program);

    pio_spi_mode1_init(pio_spi_ch->pio, 0, pio_spi_ch->offset_dac, pio_spi_ch->mosi, pio_spi_ch->sck);
    pio_spi_mode2_init(pio_spi_ch->pio, 1, pio_spi_ch->offset_adc, pio_spi_ch->miso, pio_spi_ch->mosi, pio_spi_ch->sck);
}

bool dac_write_config(pio_spi_t *ch, uint32_t src) {
    if (pio_sm_is_tx_fifo_full(ch->pio, ch->sm_active))
        return false;
    pio_sm_put(ch->pio, ch->sm_active, src << 8);
    pio_sm_set_enabled(ch->pio, ch->sm_active, true);
    return true;
}