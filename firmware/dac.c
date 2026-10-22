#include "dac.h"

void __not_in_flash_func(dac_write_blocking)(uint8_t ch, const uint32_t val) {
    pio_spi_select_dac(ch);
    dac_write(ch, val);
    while (!pio_spi_is_done(ch));
    pio_spi_deselect_dac(ch);
}

void dacs_init(void) {
    // First frame lost sometimes, start with a no-op command
    dacs_write_blocking(DAC_CFG(DAC_REG_CTRL, 0), DAC_CFG(DAC_REG_CTRL, 0));
    dacs_write_blocking(DAC_CFG(DAC_REG_RANGE, DAC_RANGE_PM5V), DAC_CFG(DAC_REG_RANGE, DAC_RANGE_PM5V));
    dacs_write_blocking(DAC_CFG(DAC_REG_POWER, DAC_POWER_ON), DAC_CFG(DAC_REG_POWER, DAC_POWER_ON));
}
