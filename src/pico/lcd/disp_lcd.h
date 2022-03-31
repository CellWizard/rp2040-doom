#ifndef __PICO_LCD_DISP_LCD_H


#define LCD_SPI_CLOCK           (20 * 1000000)      // max rate 15MHz in write mode per DS, 32MHz seen in the wild

#define LCD_SPI_PIN_MOSI        7
#define LCD_SPI_PIN_SCK         6
#define LCD_SPI_PIN_NSS         5

#define LCD_PIN_RESET           3
#define LCD_PIN_DC              4

#define ST7789_TYPE_320x240		0
#define ST7789_TYPE_240x240		1
#define ST7789_TYPE_240x135		2



void lcd_gpio_init();

void st7789_init(const uint16_t width, const uint16_t height, const uint8_t rotation);
void st7789_set_addr_window(int16_t x, int16_t y, uint16_t w, uint16_t h);

void st7789_hw_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);


#endif