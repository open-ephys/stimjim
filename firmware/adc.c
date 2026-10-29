#include "adc.h"

void __not_in_flash_func(adc_write_blocking)(uint8_t ch, const uint16_t config) {
    pio_spi_select_adc(ch);
    adc_write(ch, config);
    while (!pio_spi_is_done(ch));
    pio_spi_deselect_adc(ch);
}

int16_t __not_in_flash_func(adc_read_blocking)(uint8_t ch) {
    int16_t val = 0;
    pio_spi_select_adc(ch);
    adc_read(ch);
    while (!pio_spi_is_done(ch));
    adc_get_value(ch, &val);
    pio_spi_deselect_adc(ch);
    sleep_us(2);
    return val;
}

void __not_in_flash_func(adcs_write_blocking)(const uint16_t cfg0, const uint16_t cfg1) {
    pio_spi_select_adc(0);
    pio_spi_select_adc(1);
    adc_write(0, cfg0);
    adc_write(1, cfg1);
    while (!pio_spi_is_done(0) || !pio_spi_is_done(1));
    pio_spi_deselect_adc(0);
    pio_spi_deselect_adc(1);
}
