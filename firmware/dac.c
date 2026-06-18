#include "dac.h"

void dacs_init(pio_spi_t *p) {
    dacs_write_blocking(p, DAC_CFG(DAC_REG_RANGE, DAC_RANGE_PM10V), DAC_CFG(DAC_REG_RANGE, DAC_RANGE_PM10V));
    dacs_write_blocking(p, DAC_CFG(DAC_REG_POWER, DAC_POWER_ON), DAC_CFG(DAC_REG_POWER, DAC_POWER_ON));
}
