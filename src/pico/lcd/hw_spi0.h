#ifndef __PICO_LCD_HW_SPI0_H


#include "hardware/spi.h"
#include "hardware/dma.h"


typedef enum  {
    SPI0_MODE0 = 0,
    SPI0_MODE1,
    SPI0_MODE2,
    SPI0_MODE3
} spi0_mode_t;


extern int spi0_dmatx;



void spi0_init(uint mosi, uint sck, uint nss, uint32_t clock, spi0_mode_t mode);
void spi0_tx(const uint8_t *buf, uint32_t len);


#endif