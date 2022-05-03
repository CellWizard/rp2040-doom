/*
 * implement a DMA tx-only SPI for LCD
 */

#include <stdio.h>
#include <stdint.h>

#include "i_video.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#include "hardware/resets.h"
#include "hardware/irq.h"

#include "disp_lcd.h"
#include "hw_spi0_video.h"


#define SPI_DEV             spi0
extern uint8_t display_frame_index;
extern uint8_t display_overlay_index;
extern uint8_t display_video_type;
extern uint8_t __aligned(4) frame_buffer[2][SCREENWIDTH*SCREENHEIGHT];
extern uint16_t palette[256];
int spi0_dmatx;
dma_channel_config spi0_dmatx_conf;
extern semaphore_t display_frame_freed;
#ifndef DMA_IRQ_0
#define DMA_IRQ_0 11
#endif
void spi0_video_init(uint mosi, uint sck, uint nss, uint32_t clock, spi0_mode_t mode)
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
uint32_t sent_pixels=0;
uint32_t pixels_to_send=320*200;
uint8_t* main_pixel_buf_ptr=0;
uint8_t* status_pixel_buf_ptr=0;
uint16_t* palette_buf_ptr=0;
__aligned(4) uint16_t pixel_data_0[40];
__aligned(4) uint16_t pixel_data_1[40];
int last_pixel_data=0;
static void __not_in_flash_func(prepare_pixel_data)() {
    last_pixel_data ^= 1;
    uint16_t* pixel_data = last_pixel_data ? pixel_data_1 : pixel_data_0;
    uint8_t* pptr=0;
    if (display_video_type == VIDEO_TYPE_WIPE) {
        if ((sent_pixels / 320) >= (200-32)) {
                pptr=status_pixel_buf_ptr+(sent_pixels-(32*320));
        } else {
                pptr=main_pixel_buf_ptr+sent_pixels;
        }
        assert(wipe_yoffsets && wipe_linelookup);
        uint16_t *d = pixel_data; 
        for (int i = 0; i < 40; i++) {
            int rel = (sent_pixels/320) - wipe_yoffsets[i+(sent_pixels%320)];
            if (rel < 0) {
                d[i] = palette[pptr[i]];
		uint16_t tmp=pixel_data[i]>>8;
                pixel_data[i] = (pixel_data[i] << 8) | tmp;
            } else {
                const uint8_t *flip;
#if PICO_ON_DEVICE
                flip = (const uint8_t *)wipe_linelookup[rel];
#else
                flip = &frame_buffer[0][0] + wipe_linelookup[rel];
#endif
                // todo better protection here
                if (flip >= &frame_buffer[0][0] && flip < &frame_buffer[0][0] + 2 * SCREENWIDTH * MAIN_VIEWHEIGHT) {
                    d[i] = palette[flip[i+(sent_pixels%320)]];
		    uint16_t tmp=pixel_data[i]>>8;
                    pixel_data[i] = (pixel_data[i] << 8) | tmp;
                }
            }
        }
        sent_pixels+=40;
    } 
    else 
    {
        if ((sent_pixels / 320) >= (200))
        {
        	for (int i=0; i<40; i++) {
            	pixel_data[i]=0x0;
            }
            sent_pixels+=40;
        } 
        else 
        {
             if ((sent_pixels / 320) >= (200-32)) {
                    pptr=status_pixel_buf_ptr+(sent_pixels-(32*320));
             } else {
                    pptr=main_pixel_buf_ptr+sent_pixels;
             }    
             for (int i=0; i<40; i++) {
                    pixel_data[i] = palette_buf_ptr[pptr[i]];
                    uint16_t tmp=pixel_data[i]>>8;
                    pixel_data[i] = (pixel_data[i] << 8) | tmp;
             }    
             sent_pixels+=40;
        }
    }
//    last_pixel_data ^= 1;
}
static void __not_in_flash_func(spi0_video_dma_handler)() {
    dma_channel_acknowledge_irq0(spi0_dmatx);
    if (sent_pixels <= pixels_to_send + 40) {
        dma_channel_configure(spi0_dmatx, &spi0_dmatx_conf,
                          &spi_get_hw(spi0)->dr, // write address
                          last_pixel_data ? pixel_data_1 : pixel_data_0,      // read address
                          40*2,      // element count (of size transfer_data_size)
                          true);    // do we start now?
	prepare_pixel_data();
    } else {
	    while (spi_get_hw(spi0)->sr & 0x10);
	    sent_pixels=0;
	    sem_release(&display_frame_freed);
    }
}
void spi0_video_tx(const uint8_t *buf, const uint8_t* status_buf, const uint16_t* palette, uint32_t len)
{
    palette_buf_ptr=palette;
    status_pixel_buf_ptr=status_buf;
    main_pixel_buf_ptr=buf;
    sent_pixels=0;
    pixels_to_send=len>>1;
    last_pixel_data=1;
    prepare_pixel_data(); 
    prepare_pixel_data(); 
    if (irq_get_exclusive_handler(DMA_IRQ_0)) {
	    irq_remove_handler(DMA_IRQ_0,irq_get_exclusive_handler(DMA_IRQ_0));
    }
    irq_set_exclusive_handler(DMA_IRQ_0, spi0_video_dma_handler);
    irq_set_enabled(DMA_IRQ_0,true);
    dma_channel_set_irq0_enabled(spi0_dmatx, true);
    dma_channel_configure(spi0_dmatx, &spi0_dmatx_conf,
                          &spi_get_hw(spi0)->dr, // write address
                          pixel_data_0,      // read address
                          40*2,      // element count (of size transfer_data_size)
                          true);    // do we start now?
}

void spi0_video_tx_block_until_done()
{
    dma_channel_wait_for_finish_blocking(spi0_dmatx);
}
