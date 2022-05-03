#include "pico/stdlib.h"
#include "pico/binary_info.h"

#include "hardware/gpio.h"

#include "disp_lcd.h"
#include "hw_spi0_video.h"


void lcd_gpio_init()
{
    // add metadata for picotool
    bi_decl(bi_1pin_with_name(LCD_SPI_PIN_MOSI, "LCD: SPI0 MOSI"));
    bi_decl(bi_1pin_with_name(LCD_SPI_PIN_SCK,  "LCD: SPI0 SCK"));
    bi_decl(bi_1pin_with_name(LCD_SPI_PIN_NSS,  "LCD: SPI0 !CS"));

    bi_decl(bi_1pin_with_name(LCD_PIN_RESET,    "LCD: Reset"));
    bi_decl(bi_1pin_with_name(LCD_PIN_DC,       "LCD: Data/Command"));

    gpio_init(LCD_PIN_RESET);
    gpio_init(LCD_PIN_DC);
#ifdef CW_BACKLIGHT
    gpio_init(LCD_PIN_BL);
    gpio_set_dir(LCD_PIN_BL, GPIO_OUT);
    gpio_put(LCD_PIN_BL, 1);
#endif
    gpio_set_dir(LCD_PIN_RESET, GPIO_OUT);
    gpio_set_dir(LCD_PIN_DC,    GPIO_OUT);

    gpio_put(LCD_PIN_RESET, 0);
    gpio_put(LCD_PIN_DC, 0);
}
