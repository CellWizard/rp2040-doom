/*
 * implement a DMA tx-only SPI for LCD
 */

#include <stdio.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"

#include "hardware/resets.h"

#include "disp_lcd.h"
#include "hw_spi0.h"


#define SPI_DEV             spi0


int spi0_dmatx;
dma_channel_config spi0_dmatx_conf;


void spi0_init(uint mosi, uint sck, uint nss, uint32_t clock, spi0_mode_t mode)
{
    spi_cpol_t cpol;
    spi_cpha_t cpha;

    switch (mode) {
        case SPI0_MODE1: cpol = SPI_CPOL_0; cpha = SPI_CPHA_1; break;
        case SPI0_MODE2: cpol = SPI_CPOL_1; cpha = SPI_CPHA_0; break;
        case SPI0_MODE3: cpol = SPI_CPOL_1; cpha = SPI_CPHA_1; break;
        default:         cpol = SPI_CPOL_0; cpha = SPI_CPHA_0; break;
    }

    // reset SPI
    reset_block(SPI_DEV == spi0 ? RESETS_RESET_SPI0_BITS : RESETS_RESET_SPI1_BITS);
    unreset_block_wait(SPI_DEV == spi0 ? RESETS_RESET_SPI0_BITS : RESETS_RESET_SPI1_BITS);

    // set up SPI
    spi_set_baudrate(SPI_DEV, clock);
    spi_set_format(SPI_DEV, 8, cpol, cpha, SPI_MSB_FIRST);
    hw_set_bits(&spi_get_hw(SPI_DEV)->dmacr, SPI_SSPDMACR_TXDMAE_BITS | SPI_SSPDMACR_RXDMAE_BITS);

    // enable SPI
    hw_set_bits(&spi_get_hw(SPI_DEV)->cr1, SPI_SSPCR1_SSE_BITS);

    // connect SPI GPIOs
    gpio_init(nss);
    gpio_set_dir(nss, GPIO_OUT);
    gpio_put(nss, 1);
    gpio_set_function(sck, GPIO_FUNC_SPI);
    gpio_set_function(mosi, GPIO_FUNC_SPI);

    // get unused DMA
    // spi0_dmatx = dma_claim_unused_channel(true);
    spi0_dmatx = 1;     // todo: fix hardcoded DMA channel

    // We set the outbound DMA to transfer from a memory buffer to the SPI
    // transmit FIFO paced by the SPI TX FIFO DREQ
    spi0_dmatx_conf = dma_channel_get_default_config(spi0_dmatx);
    channel_config_set_transfer_data_size(&spi0_dmatx_conf, DMA_SIZE_8);
    channel_config_set_dreq(&spi0_dmatx_conf, spi_get_dreq(spi0, true));
}

void spi0_tx(const uint8_t *buf, uint32_t len)
{
    dma_channel_configure(spi0_dmatx, &spi0_dmatx_conf,
                          &spi_get_hw(spi0)->dr, // write address
                          buf,      // read address
                          len,      // element count (of size transfer_data_size)
                          true);    // do we start now?
}

void spi0_tx_block_until_done()
{
    dma_channel_wait_for_finish_blocking(spi0_dmatx);
}
