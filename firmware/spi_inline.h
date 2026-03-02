#pragma once

#include <hardware/spi.h>
#include <stdint.h>

__attribute__ ((always_inline)) static inline int __not_in_flash_func(spi_write_blocking_inline)(spi_inst_t *spi, const uint8_t *src, size_t len) {
    invalid_params_if(HARDWARE_SPI, 0 > (int)len);
    // Write to TX FIFO whilst ignoring RX, then clean up afterward. When RX
    // is full, PL022 inhibits RX pushes, and sets a sticky flag on
    // push-on-full, but continues shifting. Safe if SSPIMSC_RORIM is not set.
    for (size_t i = 0; i < len; ++i) {
        while (!spi_is_writable(spi))
            tight_loop_contents();
        spi_get_hw(spi)->dr = (uint32_t)src[i];
    }
    // Drain RX FIFO, then wait for shifting to finish (which may be *after*
    // TX FIFO drains), then drain RX FIFO again
    while (spi_is_readable(spi))
        (void)spi_get_hw(spi)->dr;
    while (spi_get_hw(spi)->sr & SPI_SSPSR_BSY_BITS)
        tight_loop_contents();
    while (spi_is_readable(spi))
        (void)spi_get_hw(spi)->dr;

    // Don't leave overrun flag set
    spi_get_hw(spi)->icr = SPI_SSPICR_RORIC_BITS;

    return (int)len;
}

__attribute__ ((always_inline)) static inline int __not_in_flash_func(spi_write16_blocking_inline)(spi_inst_t *spi, const uint16_t *src, size_t len) {
    invalid_params_if(HARDWARE_SPI, 0 > (int)len);
    // Deliberately overflow FIFO, then clean up afterward, to minimise amount
    // of APB polling required per halfword
    for (size_t i = 0; i < len; ++i) {
        while (!spi_is_writable(spi))
            tight_loop_contents();
        spi_get_hw(spi)->dr = (uint32_t)src[i];
    }

    while (spi_is_readable(spi))
        (void)spi_get_hw(spi)->dr;
    while (spi_get_hw(spi)->sr & SPI_SSPSR_BSY_BITS)
        tight_loop_contents();
    while (spi_is_readable(spi))
        (void)spi_get_hw(spi)->dr;

    // Don't leave overrun flag set
    spi_get_hw(spi)->icr = SPI_SSPICR_RORIC_BITS;

    return (int)len;
}

__attribute__ ((always_inline)) static inline int __not_in_flash_func(spi_write16_read16_blocking_inline)(spi_inst_t *spi, const uint16_t *src, uint16_t *dst, size_t len) {
    invalid_params_if(HARDWARE_SPI, 0 > (int)len);
    // Never have more transfers in flight than will fit into the RX FIFO,
    // else FIFO will overflow if this code is heavily interrupted.
    const size_t fifo_depth = 8;
    size_t rx_remaining = len, tx_remaining = len;

    while (rx_remaining || tx_remaining) {
        if (tx_remaining && spi_is_writable(spi) && rx_remaining < tx_remaining + fifo_depth) {
            spi_get_hw(spi)->dr = (uint32_t) *src++;
            --tx_remaining;
        }
        if (rx_remaining && spi_is_readable(spi)) {
            *dst++ = (uint16_t) spi_get_hw(spi)->dr;
            --rx_remaining;
        }
    }

    return (int)len;
}