#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include "graphic_lcd.h"
#include "exercise.h"
#include "config.h"

/* Intended for ATmega644P */
/* Clock rate is 8.0 MHz  (8MHz external oscillator , no divide by 8, to give CLKIO of 8MHz */

// USART register setup - (from iomxx4.h):

/**** USART1 LCD Set up in SPI mode *****/

/* UCSR1A: 0x00
 RXC1	0 - no receive flag
 TXC1	0 - set to 1 to clear existing transmits
 UDRE1	0 - clear empty data register flag
 FE1	0 - RESERVED
 DOR1	0 - RESERVED
 UPE1	0 - RESERVED
 U2X1	0 - RESERVED
 MPCM1	0 - RESERVED
*/

/* UCSR1B: 0x08
 RXCIE1	0 - no rx interrupts
 TXCIE1	0 - no tx interrupts
 UDRIE1	0 - no tx data register empty interrupt
 RXEN1	0 - disable receiver
 TXEN1	1 - enable transmitter in spi mode
 UCSZ12	0 - RESERVED
 RXB81	0 - RESERVED
 TXB81	0 - RESERVED
*/

/* UCSR1C: 0xC2
 UMSEL11 1 - SPI mode
 UMSEL10 1 - ^
 UPM11	 0 - RESERVED
 UPM10	 0 - RESERVED
 USBS1	 0 - RESERVED
 UDORD1  0 - MSB first
 UCPHA1  1
 UCPOL1	 0 - data change on falling edge of clock
*/


uint8_t LCD_buffer[LCD_ROWS][LCD_WIDTH];
uint8_t LCD_current_mode = LCD_DATA_MODE; //this tracks the data/command mode of the LCD


void LCD_init() {
	UBRR1 = 0;				//needs to be zero before enabling the transmitter
	UCSR1A = 0x00; 			//see comments in top of file
	UCSR1B = 0x08;			//^
	UCSR1C = 0xC3;			//^
	UBRR1 = 2;				//run at 2Mbit baud rate (BAUD = fosc/(2*(UBBRn+1)) fosc = 8MHz)
	UCSR1A |= (1<<TXC1);	//clear any existing transmits
	LCD_reset();
	//Send the 'magic' initialisation commands
	LCD_transmit_command(0x21);	//switch to extended commands  
	LCD_transmit_command(0xB0);	//set value of Vop (controls contrast) = 0x80 | 0x60 (arbitrary)  
	LCD_transmit_command(0x04);	//set temperature coefficient  
	LCD_transmit_command(0x14);	//set bias mode to 1:48.  
	LCD_transmit_command(0x20);	//switch back to regular commands
	LCD_transmit_command(0x0C); //enable normal display (dark on light), horizontal addressing
	LCD_transmit_command(0x40);	//set Y address to 0
	LCD_transmit_command(0x80);	//set X address to 0
}

void LCD_reset() {
	PORTD &= ~(1<<LCD_RESET);	//Reset pin is active LOW
	_delay_ms(1);				// Wait 1ms
	PORTD |=  (1<<LCD_RESET);	//Reset finished, set HIGH
	_delay_ms(1);				// Wait 1ms
}

void LCD_set_mode(uint8_t mode) {
	if (mode != LCD_current_mode) {
		/* Wait for empty transmit buffer to allow the data/command signal to be read */
		while ( !( UCSR0A & (1<<UDRE0)) );
		_delay_ms(1);				// Wait 1ms to allow the previous mode to be read
	}
	if (mode == LCD_DATA_MODE) {
		PORTD |=  (1<<LCD_DC);	//DATA input is HIGH
	} else if (mode == LCD_COMMAND_MODE) {
		PORTD &= ~(1<<LCD_DC);	//Command/Address input set LOW
	} else {
	}
	LCD_current_mode = mode;
}

void LCD_transmit( uint8_t data ) {
	
	/* Wait for empty transmit buffer */
	while ( !( UCSR1A & (1<<UDRE1)) );

	/* Put data into buffer, sends the data */
	UDR1 = data;
}

void LCD_transmit_data( uint8_t data ) {
	LCD_set_mode(LCD_DATA_MODE);
	LCD_transmit(data);
}

void LCD_transmit_command( uint8_t data ) {
	LCD_set_mode(LCD_COMMAND_MODE);
	LCD_transmit(data);
}

void LCD_clear_buff() {
	uint8_t x = 0;
	uint8_t y = 0;
	for(y = 0; y < LCD_ROWS; y ++) {
		for(x = 0; x < LCD_WIDTH; x ++) LCD_buffer[y][x] = 0;
	}
}

void LCD_clear() {
	LCD_clear_buff();
	LCD_update();
}

void LCD_update() {
	LCD_transmit_command(0x40);	//set Y address to 0
	LCD_transmit_command(0x80);	//set X address to 0
	uint8_t x = 0;
	uint8_t y = 0;
	for(y = 0; y < LCD_ROWS; y ++) {
		for(x = 0; x < LCD_WIDTH; x ++) {
			LCD_transmit_data( LCD_buffer[y][x] );
		}
	}
}

void LCD_set_pixel(uintCoord p) { //measures from top left
	//check the range of the pixels
	if ((p.x < LCD_WIDTH) && (p.y < LCD_HEIGHT)) {
		div_t y_row = div(p.y,LCD_BYTE_HEIGHT);
		LCD_buffer[y_row.quot][p.x] |= (1<<y_row.rem);
	}
}

void LCD_set_pixel_tr(uintCoord p, uint8_t transpose) {
	uintCoord pix;
	if (transpose > 0) {
		pix.x = p.y;
		pix.y = p.x;
	} else {
		pix = p;
	}
	LCD_set_pixel(pix);
}


