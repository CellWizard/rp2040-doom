#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"

#include "disp_lcd.h"
#include "hw_spi0_video.h"


// platform-specific
#define ST7789_SPI_SS(x)          gpio_put(LCD_SPI_PIN_NSS, x)
//#define ST7789_SPI_Write(x, l)    spi0_tx(x, l)
//#define ST7789_SPI_Block()        asm volatile("nop \n nop \n nop"); while (spi_is_busy(spi0)) {tight_loop_contents();}
#define ST7789_SPI_Write(x, l)	  spi_write_blocking(spi0, x, l)
#define ST7789_SPI_Block()


#define ST7789_HW_RST(x)          gpio_put(LCD_PIN_RESET, x)
#define ST7789_HW_DC(x)           gpio_put(LCD_PIN_DC, x)

#define ST7789_Wait(x)			  busy_wait_us_32(x * 1000);


// lcd defines
#define DELAY         	    0x80    // special signifier for command lists

#define ST7789_MAX_COLS		320
#define ST7789_MAX_ROWS		240

#define ST77XX_NOP        	0x00
#define ST77XX_SWRESET    	0x01
#define ST77XX_RDDID      	0x04
#define ST77XX_RDDST      	0x09

#define ST77XX_SLPIN      	0x10
#define ST77XX_SLPOUT     	0x11
#define ST77XX_PTLON      	0x12
#define ST77XX_NORON      	0x13

#define ST77XX_INVOFF     	0x20
#define ST77XX_INVON      	0x21
#define ST77XX_DISPOFF    	0x28
#define ST77XX_DISPON     	0x29
#define ST77XX_CASET      	0x2A
#define ST77XX_RASET      	0x2B
#define ST77XX_RAMWR      	0x2C
#define ST77XX_RAMRD      	0x2E    // cannot be used in serial mode

#define ST77XX_PTLAR      	0x30
#define ST77XX_VSCRDEF    	0x33
#define ST77XX_TEOFF      	0x34
#define ST77XX_TEON       	0x35
#define ST77XX_MADCTL     	0x36
#define ST77XX_VSCSAD     	0x37
#define ST77XX_IDMOFF     	0x38
#define ST77XX_IDMON      	0x39    // idle mode reduces color depth to 8 color mode
#define ST77XX_COLMOD     	0x3A

#define COLMOD_12BPP      	0x53
#define COLMOD_16BPP      	0x55
#define COLMOD_18BPP      	0x56
#define COLMOD_16BPPTRUNC 	0x57

#define MADCTL_BIT_MY      	0x80
#define MADCTL_BIT_MX      	0x40
#define MADCTL_BIT_MV      	0x20
#define MADCTL_BIT_ML      	0x10
#define MADCTL_BIT_RGB     	0x08
#define MADCTL_BIT_MH      	0x04

#define MADCTL_TOP_TO_BOT 	0
#define MADCTL_BOT_TO_TOP 	MADCTL_MY
#define MADCTL_LFT_TO_RGT 	0
#define MADCTL_RGT_TO_LFT 	MADCTL_MX
#define MADCTL_PC_NORMAL  	0
#define MADCTL_PC_REVERSE 	MADCTL_MV
#define MADCTL_REFRESH_TB 	0
#define MADCTL_REFRESH_BT 	MADCTL_ML
#define MADCTL_RGB        	0
#define MADCTL_BGR        	MADCTL_RGB
#define MADCTL_DLATCH_LTR 	0
#define MADCTL_DLATCH_RTL 	MADCTL_MH

#define ST77XX_RDID1      	0xDA
#define ST77XX_RDID2      	0xDB
#define ST77XX_RDID3      	0xDC
#define ST77XX_RDID4      	0xDD

// Some ready-made 16-bit ('565') color settings:
#define ST7789_BLACK      	0x0000
#define ST7789_BLUE       	0x001F
#define ST7789_RED        	0xF800
#define ST7789_GREEN      	0x07E0
#define ST7789_CYAN       	0x07FF
#define ST7789_MAGENTA    	0xF81F
#define ST7789_YELLOW     	0xFFE0
#define ST7789_WHITE      	0xFFFF

typedef struct st7789_conf_t {
	uint16_t w, h;		// virtual width and height
	uint16_t real_w, real_h;
	uint16_t xoff, yoff;
	uint16_t colstart, colstart2;
	uint16_t rowstart, rowstart2;
	uint8_t  rotation;
} st7789_conf_t;

static st7789_conf_t st7789;

// SCREEN INITIALIZATION ***************************************************

// generic init code that should work on any st7789-type display
const uint8_t cmd_init_std[] = {	// Init commands for 7789 screens
    8,                              //  8 commands in list:
    ST77XX_SWRESET,    DELAY,       //  1: Software reset, no args + delay
    	150,                        //     150ms
    ST77XX_SLPOUT,     DELAY,       //  2: Out of sleep mode (put there in reset), no args + delay
      	150,                        //     150ms
    ST77XX_COLMOD, 1 | DELAY,       //  3: Set color mode, 1 arg + delay:
      	COLMOD_16BPP, 10,          	//     16-bit color, 10ms delay
	ST77XX_CASET,  4,               //  4: Column addr set, 4 args, no delay:
    	0, 0,				        //     xstart
		0, 239,		                //     xend
    ST77XX_RASET, 4,                //  5: Row addr set, 4 args, no delay:
      	0, 0,                       //     ystart
	  	(319 >> 8), (319 & 0xff),   //     yend
	ST77XX_INVON,      DELAY,      	//  6: hack (not sure why this is here) + delay (are the delays necessary?)
		10,
	ST77XX_NORON,      DELAY,      	//  7: Normal display on + delay
		10,
	ST77XX_DISPON,     DELAY,      	//  8: Main screen turn on + delay
		10
};

//*************************** User Functions ***************************//
void st7789_hw_draw_pixel(uint8_t x, uint8_t y, uint16_t color);
//void drawHLine(uint8_t x, uint8_t y, uint8_t w, uint16_t color);
//void drawVLine(uint8_t x, uint8_t y, uint8_t h, uint16_t color);
void st7789_hw_fill_screen(uint16_t color);
void st7789_set_rotation(uint8_t rotation);
void st7789_set_addr_window_full();
void st7789_invert_display(bool i);


//************************* Non User Functions *************************//
static void lcd_send_init();
static void write_cmd(uint8_t cmd);

/////////////////////////////////////////////////////////////////////////

/**************************************************************************/
/*!
    @brief  Write a single command byte to the display. Chip-select and
            transaction must have been previously set -- this ONLY sets
            the device to COMMAND mode, issues the byte and then restores
            DATA mode. There is no corresponding explicit writeData()
            function -- just use ST7789_SPI_Write().
    @param  cmd  8-bit command to write.
*/
/**************************************************************************/
static void write_cmd(uint8_t cmd)
{
    ST7789_HW_DC(0);

	ST7789_SPI_SS(0);
    ST7789_SPI_Write(&cmd, 1);
    ST7789_SPI_Block();	
	ST7789_SPI_SS(1);

    ST7789_HW_DC(1);
}

/**************************************************************************/
/*!
    @brief  Companion code to the initiliazation tables. Reads and issues
            a series of LCD commands stored in ROM byte array.
    @param  addr  Flash memory array with commands and data to send
*/
/**************************************************************************/
static void lcd_send_init()
{
    uint8_t count, args;
    uint8_t ms;

	uint8_t display = 0;

	const uint8_t *c = cmd_init_std;
	count = *c++;					// command count

	while (count--) {			// For each command...
		write_cmd(*c++);		// send command byte as command			
		args  = *c++;     		// get argument count
		ms    = args & DELAY;	// If high bit set, delay follows args
		args &= ~DELAY;         // mask delay bit
		if (args) {
			ST7789_SPI_SS(0);
			ST7789_SPI_Write(c, args);
			ST7789_SPI_Block();	// wait until writing done
			ST7789_SPI_SS(1);
			c += args;			// move pointer forward
		}

		if (ms) {				// are we delaying?
			ms = *c++;		 	// get delay time
			ST7789_Wait(ms);	// and then wait
		}
	}
}

/**************************************************************************/
/*!
    @brief  Initialization code for ST7789 display
*/
/**************************************************************************/
void st7789_init(const uint16_t width, const uint16_t height, const uint8_t rotation) {
	// deselect lcd
	ST7789_SPI_SS(1);

	// reset lcd the slow but guaranteed way
    ST7789_HW_RST(0);
	ST7789_Wait(150);
    ST7789_HW_RST(1);
	ST7789_Wait(180);

	st7789.real_w = width;
	st7789.real_h = height;
	
	st7789.colstart = st7789.colstart2 = 0;
	st7789.rowstart = st7789.rowstart2 = 0;

	// set configured values
	if (width == 240) {		// 240x240 square screens
		st7789.rowstart = 320 - 240;
	}
	if (width == 135 || height == 135) {	// 135x240, 240x135 screens
		st7789.colstart  = (uint16_t)((240 - width + 1)  / 2);
		st7789.colstart2 = (uint16_t)((240 - width)      / 2);
		st7789.rowstart  = (uint16_t)((320 - height + 1) / 2);
		st7789.rowstart2 = (uint16_t)((320 - height)     / 2);
	}

	// send setup up lcd
	lcd_send_init();

	// set rotation (and bounds)
  	st7789_set_rotation(rotation);
	st7789_hw_fill_screen(ST7789_BLACK);
	//st7789_hw_fill_rect(16, 16, 16, 16, ST7789_RED); // testing
	st7789_set_addr_window_full();
}

/**************************************************************************/
/*!
  @brief  SPI displays set an address window rectangle for blitting pixels
  @param  x  Top left corner x coordinate
  @param  y  Top left corner x coordinate
  @param  w  Width of window
  @param  h  Height of window
*/
/**************************************************************************/
void st7789_set_addr_window(int16_t x, int16_t y, uint16_t w, uint16_t h) {
	uint16_t nx, ny, nw, nh;
	uint8_t d[4];
	
	nx = x + st7789.xoff;
	ny = y + st7789.yoff;

	nw = w + nx - 1;
	nh = h + ny - 1;

	// column addr set
	write_cmd(ST77XX_CASET);
	
	d[0] = nx >> 8; d[1] = nx & 0xff;
	d[2] = nw >> 8; d[3] = nw & 0xff;
	ST7789_SPI_SS(0);
	ST7789_SPI_Write(d, 4);
	ST7789_SPI_Block();
	ST7789_SPI_SS(1);

	// row addr set
	write_cmd(ST77XX_RASET); 
	d[0] = ny >> 8; d[1] = ny & 0xff;
	d[2] = nh >> 8; d[3] = nh & 0xff;
	ST7789_SPI_SS(0);
	ST7789_SPI_Write(d, 4);
	ST7789_SPI_Block();
	ST7789_SPI_SS(1);

	// resume write to RAM
	write_cmd(ST77XX_RAMWR); 
}

void st7789_set_addr_window_full()
{
	st7789_set_addr_window(0, 0, st7789.w, st7789.h);
}

/**************************************************************************/
/*!
    @brief  Set origin of (0,0) and orientation of TFT display
    @param  m  The index for rotation, from 0-3 inclusive
*/
/**************************************************************************/
void st7789_set_rotation(uint8_t rotation)
{
	uint8_t madctl = 0;

	st7789.rotation = rotation & 3;	// can't be higher than 3

	switch (rotation) {
		case 0:
			madctl = MADCTL_BIT_MX | MADCTL_BIT_MY | MADCTL_BIT_RGB;
			st7789.w = st7789.real_w;
			st7789.h = st7789.real_h;
			st7789.xoff = st7789.colstart;
			st7789.yoff = st7789.rowstart;			
			break;
		case 1:
			madctl = MADCTL_BIT_MY | MADCTL_BIT_MV | MADCTL_BIT_RGB;
			st7789.w = st7789.real_h;
			st7789.h = st7789.real_w;
			st7789.xoff = st7789.rowstart;
			st7789.yoff = st7789.colstart2;
			break;
		case 2:
			madctl = MADCTL_BIT_RGB;
			st7789.w = st7789.real_w;
			st7789.h = st7789.real_h;
			st7789.xoff = st7789.colstart2;
			st7789.yoff = st7789.rowstart2;
			break;
		case 3:
			madctl = MADCTL_BIT_MX | MADCTL_BIT_MV | MADCTL_BIT_RGB;
			st7789.w = st7789.real_h;
			st7789.h = st7789.real_w;
			st7789.xoff = st7789.rowstart2;
			st7789.yoff = st7789.colstart;
			break;
	}

	write_cmd(ST77XX_MADCTL);

	ST7789_SPI_SS(0);
	ST7789_SPI_Write(&madctl, 1);
	ST7789_SPI_Block();
	ST7789_SPI_SS(1);
}

/**************************************************************************/
/*!
    @brief  Invert the colors of the display (if supported by hardware).
            Self-contained, no transaction setup required.
    @param  i  true = inverted display, false = normal display.
*/
/**************************************************************************/
void st7789_invert_display(bool i)
{
    write_cmd(i ? ST77XX_INVON : ST77XX_INVOFF);
}

void st7789_hw_draw_pixel(uint8_t x, uint8_t y, uint16_t color)
{
	uint8_t d[2];

	if((x < st7789.w) && (y < st7789.h)) {
    	st7789_set_addr_window(x, y, 1, 1);
		
		d[0] = color >> 8;
		d[1] = color & 0xff;

		ST7789_SPI_SS(0);
    	ST7789_SPI_Write(d, 2);
		ST7789_SPI_Block();
		ST7789_SPI_SS(1);
	}
}

/**************************************************************************/
/*!
   @brief    Draw a perfectly horizontal line (this is often optimized in a subclass!)
    @param    x   Left-most x coordinate
    @param    y   Left-most y coordinate
    @param    w   Width in pixels
   @param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
/*
void drawHLine(uint8_t x, uint8_t y, uint8_t w, uint16_t color) {
  if( (x < _width) && (y < _height) && w) {   
    uint8_t hi = color >> 8, lo = color;

    if((x + w - 1) >= _width)  
      w = _width  - x;
    #ifdef TFT_CS
      TFT_CS = 0;
    #endif
    setAddrWindow(x, y, w, 1);
    while (w--) {
    ST7789_SPI_Write(hi);
    ST7789_SPI_Write(lo);
    }
    #ifdef TFT_CS
      TFT_CS = 1;
    #endif
  }
}
*/

/**************************************************************************/
/*!
   @brief    Draw a perfectly vertical line (this is often optimized in a subclass!)
    @param    x   Top-most x coordinate
    @param    y   Top-most y coordinate
    @param    h   Height in pixels
   @param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
/*
void drawVLine(uint8_t x, uint8_t y, uint8_t h, uint16_t color) {
  if( (x < _width) && (y < _height) && h) {  
    uint8_t hi = color >> 8, lo = color;
    if((y + h - 1) >= _height) 
      h = _height - y;
    #ifdef TFT_CS
      TFT_CS = 0;
    #endif
    setAddrWindow(x, y, 1, h);
    while (h--) {
      ST7789_SPI_Write(hi);
      ST7789_SPI_Write(lo);
    }
    #ifdef TFT_CS
      TFT_CS = 1;
    #endif
  }
}
*/

/**************************************************************************/
/*!
	@brief    Fill a rectangle completely with one color. Update in subclasses if desired!
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    w   Width in pixels
    @param    h   Height in pixels
	@param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void st7789_hw_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
	if ((x < st7789.w) && (y < st7789.h) && w && h) {
		uint8_t hi = color >> 8, lo = color;
		uint32_t px;

		if ((x + w - 1) >= st7789.w) {
			w = st7789.w - x;
		}
		if ((y + h - 1) >= st7789.h) {
			h = st7789.h - y;
		}
	
		st7789_set_addr_window(x, y, w, h);
	
		px = (uint32_t)w * h;
		
		ST7789_SPI_SS(0);
		while (px--) {
			ST7789_SPI_Write(&hi, 1);
			ST7789_SPI_Write(&lo, 1);
		}
		ST7789_SPI_SS(1);
	}
}

/**************************************************************************/
/*!
	@brief    Fill the screen completely with one color. Update in subclasses if desired!
    @param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void st7789_hw_fill_screen(uint16_t color)
{
    st7789_hw_fill_rect(0, 0, st7789.w, st7789.h, color);
}
