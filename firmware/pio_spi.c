#include "pio_spi.h"

void pio_spi_init(pio_spi_t *pio_spi_ch)
{
    pio_set_gpio_base(pio_spi_ch->pio, pio_spi_ch->base);

    pio_spi_ch->offset_dac = pio_add_program(pio_spi_ch->pio, &pio_spi_mode1_program);
    pio_spi_ch->offset_adc = pio_add_program(pio_spi_ch->pio, &pio_spi_mode2_program);

    pio_spi_mode1_init(pio_spi_ch->pio, 0, pio_spi_ch->offset_dac, pio_spi_ch->mosi, pio_spi_ch->sck);
    pio_spi_mode2_init(pio_spi_ch->pio, 1, pio_spi_ch->offset_adc, pio_spi_ch->miso, pio_spi_ch->mosi, pio_spi_ch->sck);
}

