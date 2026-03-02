#include <hardware/gpio.h>
#include <hardware/spi.h>
#include "dac.h"

void dac_init(bool ch) {
  dac_write(ch, 8, 4); // output-range reg -> +/-10V
  dac_write(ch, 16, 1); // power-control reg -> DAC_A powered
}

void dac_update_output(bool ch, int16_t amp) {
  const uint8_t nldac[2] = { NLDAC_A, NLDAC_B };
  dac_write(ch, 0, amp);
  asm volatile("nop\n\t" "nop\n\t" "nop\n\t");
  asm volatile("nop\n\t" "nop\n\t" "nop\n\t");
  asm volatile("nop\n\t" "nop\n\t" "nop\n\t");
  asm volatile("nop\n\t" "nop\n\t" "nop\n\t");
  asm volatile("nop\n\t" "nop\n\t" "nop\n\t");
  gpio_put(nldac[ch], false);
  asm volatile("nop\n\t" "nop\n\t" "nop\n\t");
  gpio_put(nldac[ch], true);
}