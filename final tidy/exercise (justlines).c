/* these two lines take care of the IO definitions for the device, and the interrupts available for them */
#define F_CPU 8E6	// 8MHz
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay_basic.h>
#include <util/delay.h>
#include <stdlib.h>
#include "exercise.h"
#include "config.h"
#include <math.h>

/* put comments in here for future reference, showing what the device is intended to do, which
device is to be used, and the fuse settings (and why) */


/* This is a template created by cutting bits from a stepper motor drive
   Some port direction commands are included as examples */

/* ported from Workbook 2 exercise 2 */

/* Intended for ATmega644P */
/* Clock rate is 8.0 MHz  (8MHz external oscillator , no divide by 8, to give CLKIO of 8MHz */

// Fuse high byte is default (0x99)			See data sheet page 297

/* Fuse low byte is 0xA7:  ((NB fuses active low)   See data sheet page 297
 * CLKDIV8 	1 (div8 deactive)
 * CLKOUT 	0 (clk out active)
 * SUT1   	1 (fast risinfg power)
 * SUT0 	0
 * CKSEL3	0  8MHz external full swing osc
 * CKSEL2	1
 * CKSEL1	1
 * CKSEL0	1
 */

// USART register setup - (from iomxx4.h):

/* UCSR0A: 0x00
 RXC0	x/0 - non writable
 TXC0	0 - set to 1 to clear existing transmits
 UDRE0	x/0
 FE0	x/0
 DOR0	x/0
 UPE0	x/0
 U2X0	0 - turn off float speed
 MPCM0	0 - disable multi-processor communication mode
*/

/* UCSR0B: 0x98
 RXCIE0	1 - enable rx interrupts
 TXCIE0	0 - no tx interrupts
 UDRIE0	0 - no tx data register empty interrupt
 RXEN0	1 - enable receiver
 TXEN0	1 - enable transmitter
 UCSZ02	0 - 8 data bits (in combination with UCSZ01/0)
 RXB80	0/x
 TXB80	0 - not usinfg 9th bit of communication
*/

/* UCSR0C: 0x06
 UMSEL01 0 - asynchronous mode
 UMSEL00 0 - ^
 UPM01	 0 - no parity
 UPM00	 0 - ^read from UDR0 which clears any pending receive-complete interrupts.
Set the global interrupt enable bit in the Status Register SREG, see section 6.3.1 of the
datasheet.
 USBS0	 0 - 1 stop bit
 UCSZ01	 1 - 8 data bits (in combination with UCSZ02)
 UCSZ00	 1 - ^
 UCPOL0	 0 - always set to 0 for async mode
*/

/**** USART0 Compass Set up *****/

/* UCSR0A: 0x00
 RXC0	x/0 - non writable
 TXC0	0 - set to 1 to clear existing transmits
 UDRE0	x/0
 FE0	x/0
 DOR0	x/0
 UPE0	x/0
 U2X0	0 - turn off float speed
 MPCM0	0 - disable multi-processor communication mode
*/

/* UCSR0B: 0x98
 RXCIE0	1 - enable rx interrupts
 TXCIE0	0 - no tx interrupts
 UDRIE0	0 - no tx data register empty interrupt
 RXEN0	1 - enable receiver
 TXEN0	1 - enable transmitter
 UCSZ02	0 - 8 data bits (in combination with UCSZ01/0)
 RXB80	0/x
 TXB80	0 - not usinfg 9th bit of communication
*/

/* UCSR0C: 0x0E
 UMSEL01 0 - asynchronous mode
 UMSEL00 0 - ^
 UPM01	 0 - no parity
 UPM00	 0 - ^read from UDR0 which clears any pending receive-complete interrupts.
Set the global interrupt enable bit in the Status Register SREG, see section 6.3.1 of the
datasheet.
 USBS0	 1 - 2 stop bit
 UCSZ01	 1 - 8 data bits (in combination with UCSZ02)
 UCSZ00	 1 - ^
 UCPOL0	 0 - always set to 0 for async mode
*/

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

/* Interrupt vectors */
// If we were usinfg interrupts the interrupt vectors would be here
// there is a useful list at    http://www.nongnu.org/avr-libc/user-manual/group__avr__interrupts.html


/* Port direction setting */

uint8_t LCD_buffer[LCD_ROWS][LCD_WIDTH];
uint8_t LCD_current_mode = LCD_DATA_MODE; //this tracks the data/command mode of the LCD
volatile float angle = 0;
volatile int updated = 0;
doubleCoord centre;
	
inline void port_direction_init() {

	/* DDR is Data Direction Register
	   PORT is used when referring to outputs
	   PIN  is used when referring to inputs 
	   See ATMEGA644P datasheet section 13  */

	// Assign all ports as inputs. 
	DDRA = 0 ;
	DDRB = 0 ;
	DDRC = 0 ;
	DDRD = 0 ;
	
	// define  outputs
	DDRB |= (1<<PB0); 	// TODO. Put a 1 in each position which needs to be an output
    DDRB |= (1<<PB2); 	
    
    //LCD outputs
    DDRD |= (1<< LCD_DC);
    DDRD |= (1<< LCD_RESET);
    DDRD |= (1<<XCK1);		//Setting the XCK1 port pin as output, enables master mode
	
	// Set high outputs

	// set low outputs
	PORTB = 0; 
	PORTD = 0;
}

void USART0_init() {
	UBRR0 = 25; 		//set baud to 9600
	UCSR0A = 0x00; 		//see comments in top of file
	UCSR0B = 0x98;		//^
	UCSR0C = 0x06;		//^
	UCSR0A |= (1<<TXC0);	//clear any existing transmits
	char tmp = UDR0;	//read from UDR0 which clears any pending receive-complete interrupts.
}

void compass_init() {
	UBRR0 = 51; 		//set baud to 19200
	UCSR0A = 0x00; 		//see comments in top of file
	UCSR0B = 0x98;		//^
	UCSR0C = 0x0E;		//^
	UCSR0A |= (1<<TXC0);	//clear any existing transmits
	char tmp = UDR0;	//read from UDR0 which clears any pending receive-complete interrupts.
}

void LCD_init() {
	UBRR1 = 0;				//needs to be zero before enabling the transmitter
	UCSR1A = 0x00; 			//see comments in top of file
	UCSR1B = 0x18;			//^
	UCSR1C = 0xC3;			//^
	UBRR1 = 1;				//run at 2Mbit baud rate (BAUD = fosc/(2*(UBBRn+1)) fosc = 8MHz)
	UCSR1A |= (1<<TXC1);	//clear any existing transmits
	LCD_reset();
	//Send the 'magic' initialisation commands
	LCD_transmit_command(0x21);	//switch to extended commands  
	LCD_transmit_command(0xB0);	//set value of Vop (controls contrast) = 0x80 | 0x60 (arbitrary)  
	LCD_transmit_command(0x04);	//set temperature coefficient  
	LCD_transmit_command(0x14);	//set bias mode to 1:48.  
	LCD_transmit_command(0x20);	//switch back to regular commands
	LCD_transmit_command(0x0C); //enable normal display (dark on light), horizontal addressinfg
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

void LCD_set_pixel_polar(polarCoord c) { //set from the centre
	float x = (LCD_WIDTH/2.0) - 0.5;	//the X origin offset
	float y = (LCD_HEIGHT/2.0)  - 0.5;	//the Y origin offset
	x = x + c.r*cosf(c.theta);	//the sum of the distance to the origin and the x coord
	y = y - c.r*sinf(c.theta);	//the difference between the distance to the origin and the y coord
	uintCoord p = {.x = round(x), .y = round(y)};
	LCD_set_pixel(p);
}

void compass_transmit( unsigned char data ) {
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );

	/* Put data into buffer, sends the data */
	UDR0 = data;
}

ISR(USART0_RX_vect) {
	// process the byte received in UDR0 from the compass
	uint8_t received = UDR0;
	//USART_transmit_uint8( received );
	//angle1 = received/128.0;
	angle = (float)PI * ((float)received/(float)128);
	PORTB ^= (1<<PB2);
	updated = 1;
}

//polygon fill
//scan each line in turn


doubleCoord rotateDoubleCoord(doubleCoord p, float theta) {
//function that implicitly rotates about the centre of the LCD by an angle theta and returns an x,y, floating point coordinate
	p.x = p.x - centre.x;	//the difference between the distance to the origin and the x coord
	p.y = centre.y - p.y;	//the difference between the distance to the origin and the y coord
	float costheta = cosf(theta);
	float sintheta = sinf(theta);
	doubleCoord pr;
	pr.x = centre.x + (1.15*((costheta*p.x) - (sintheta*p.y))); //here we apply a correction factor for the pixel aspec ratio
	pr.y = centre.y - ((sintheta*p.x) + (costheta*p.y)); 
	return pr;
}

//function that implicitly rotates about the centre of the LCD by an angle theta and returns an x,y, floating point coordinate
doubleCoord rotateUintCoord(uintCoord input, float theta) {
	doubleCoord i = {.x = (float)input.x, .y = (float)input.y};
	return rotateDoubleCoord(i, theta);
}

void draw_line(fixedPointCoord start, fixedPointCoord end) {
	fixedPoint dx = end.x - start.x;
	fixedPoint dy = end.y - start.y;
	//pick the mode that will increment along the correct dimension
	if (absint(dx) >= absint(dy)) {
		//we are now incrementing along the x direction
		line_algorithm(start, end, 0);
	} else {
		fixedPointCoord start_t;
		fixedPointCoord end_t;
		
		/*fixedPoint tmp;
		tmp = start.y;
		start.y = start.x;
		start.x = tmp;
		tmp = end.y;
		end.y = end.x;
		end.x = tmp;*/
		start_t.x = start.y;
		start_t.y = start.x;
		end_t.x = end.y;
		end_t.y = end.x;
		//we use the transpose indicator to flip the algorithm to increment along the y direction
		line_algorithm(start_t, end_t, 1);
	}
}

void line_algorithm(fixedPointCoord start, fixedPointCoord end, int8_t transpose) {
//transpose make x and y be flipped when written (for incrementing in the y direction
	//initialise and draw the first pointcurrent
	fixedPoint dx = end.x - start.x;
	fixedPoint dy = end.y - start.y;
	fixedPoint d = fip(dy)/dx; //be careful of divides and multiplies
	uintCoord current; //records the current pixel we are writing to
	if (end.x < start.x) {
		fixedPointCoord tmp = start;
		start = end;
		end = tmp;
		d=(-1)*d;
	}
	
	current.x = roundfip(start.x);
	fixedPoint yf = start.y + roundfip(d * (fip(current.x) - start.x)); //because of multiplying, we have to take a layer of fipping off - roundfip does this
	current.y = roundfip(yf);
	yf = yf - fip(current.y);
	LCD_set_pixel_tr(current, transpose);
	
	//make sure the iteration goes the right way
	int y_dir = 1;
	if (end.y < start.y) {
		y_dir = -1;
		yf = -1 * yf; //we will be only be incrementing Yf as it it simply a reference to proportion through the pixel we are. If we are moving negatively through y, we should move yf to the other end of its range
	}
	while (fip(current.x) < (end.x - fip(0.5))) {
		current.x += 1;
		yf += absint(d);
		if (yf > fip(0.5)) {
			current.y += y_dir;
			yf -= fip(1);
		}
		LCD_set_pixel_tr(current, transpose);
	}
}

int main(void) {
_delay_ms(100);
	// IO port initialisation
	port_direction_init();
	
	centre.x = ((float)LCD_WIDTH-1.0)/(float)2.0;	//the X origin offset
	centre.y = ((float)LCD_HEIGHT-1.0)/(float)2.0;	//the Y origin offset
	
	//set up the compass serial port
	compass_init();
	LCD_init();
	SREG |= 0x80;	//Set the global interrupt enable bit in the Status Register SREG, see section 6.3.1 of the datasheet.
	LCD_clear();

	while (1) {
	//TODO: use a timed interrupt to send the LCD update
	//TODO: stop the LCD update during the compass update (MAYBE??)
	//TODO: name public and private headers
	//TODO: separate out LCD functions
		updated = 0;
		compass_transmit((char) 0x12);
		while (!updated) {
		}
		LCD_clear_buff();
		//north
		doubleCoord blank_line_start = {.x = centre.x, .y = 0};
		doubleCoord blank_line_end = {.x = 37, .y = centre.y};
		doubleCoord line_start = rotateDoubleCoord(blank_line_start, angle);
		doubleCoord line_end = rotateDoubleCoord(blank_line_end, angle);
		
		//doubleCoord line_start = {.x = 0, .y = 0};
		//doubleCoord line_end = {.x = 40, .y = 48};
		
		fixedPointCoord line_start_fip = {.x = fip(line_start.x), .y = fip(line_start.y)};
		fixedPointCoord line_end_fip = {.x = fip(line_end.x), .y = fip(line_end.y)};
		
		draw_line(line_start_fip, line_end_fip);
		
		blank_line_start.x = centre.x;
		blank_line_start.y = 0;
		blank_line_end.x = 46;
		blank_line_end.y = centre.y;
		line_start = rotateDoubleCoord(blank_line_start, angle);
		line_end = rotateDoubleCoord(blank_line_end, angle);
		line_start_fip.x = fip(line_start.x);
		line_start_fip.y = fip(line_start.y);
		line_end_fip.x = fip(line_end.x);
		line_end_fip.y = fip(line_end.y);
		draw_line(line_start_fip, line_end_fip);
		//north small
		/*
		blank_line_start.x = 41.5;
		blank_line_start.y = 15;
		blank_line_end.x = 46;
		blank_line_end.y = 23.5;
		line_start = rotateDoubleCoord(blank_line_start, angle);
		line_end = rotateDoubleCoord(blank_line_end, angle);
		line_start_fip.x = fip(line_start.x);
		line_start_fip.y = fip(line_start.y);
		line_end_fip.x = fip(line_end.x);
		line_end_fip.y = fip(line_end.y);
		draw_line(line_start_fip, line_end_fip);
		//north small
		blank_line_start.x = 41.5;
		blank_line_start.y = 15;
		blank_line_end.x = 37;
		blank_line_end.y = 23.5;
		line_start = rotateDoubleCoord(blank_line_start, angle);
		line_end = rotateDoubleCoord(blank_line_end, angle);
		line_start_fip.x = fip(line_start.x);
		line_start_fip.y = fip(line_start.y);
		line_end_fip.x = fip(line_end.x);
		line_end_fip.y = fip(line_end.y);
		draw_line(line_start_fip, line_end_fip);*/
		//horizontal line
		blank_line_start.x = 30;//37;
		blank_line_start.y = centre.y;
		blank_line_end.x = 53;//46;
		blank_line_end.y = centre.y;
		line_start = rotateDoubleCoord(blank_line_start, angle);
		line_end = rotateDoubleCoord(blank_line_end, angle);
		line_start_fip.x = fip(line_start.x);
		line_start_fip.y = fip(line_start.y);
		line_end_fip.x = fip(line_end.x);
		line_end_fip.y = fip(line_end.y);
		draw_line(line_start_fip, line_end_fip);
		//south
		blank_line_start.x = centre.x;
		blank_line_start.y = 47;
		blank_line_end.x = 46;
		blank_line_end.y = centre.y;
		line_start = rotateDoubleCoord(blank_line_start, angle);
		line_end = rotateDoubleCoord(blank_line_end, angle);
		line_start_fip.x = fip(line_start.x);
		line_start_fip.y = fip(line_start.y);
		line_end_fip.x = fip(line_end.x);
		line_end_fip.y = fip(line_end.y);
		draw_line(line_start_fip, line_end_fip);
		blank_line_start.x = centre.x;
		blank_line_start.y = 47;
		blank_line_end.x = 37;
		blank_line_end.y = centre.y;
		line_start = rotateDoubleCoord(blank_line_start, angle);
		line_end = rotateDoubleCoord(blank_line_end, angle);
		line_start_fip.x = fip(line_start.x);
		line_start_fip.y = fip(line_start.y);
		line_end_fip.x = fip(line_end.x);
		line_end_fip.y = fip(line_end.y);
		draw_line(line_start_fip, line_end_fip);
		
		/*blank_line_start.x = 40;
		blank_line_start.y = 20;
		blank_line_end.x = 40;
		blank_line_end.y = 0;
		line_start = rotateUintCoord(blank_line_start, angle);
		line_end = rotateUintCoord(blank_line_end, angle);
		line_start_fip.x = fip(line_start.x);
		line_start_fip.y = fip(line_start.y);
		line_end_fip.x = fip(line_end.x);
		line_end_fip.y = fip(line_end.y);
		draw_line(line_start_fip, line_end_fip);
		blank_line_start.x = 39;
		blank_line_start.y = 20;
		blank_line_end.x = 39;
		blank_line_end.y = 0;
		line_start = rotateUintCoord(blank_line_start, angle);
		line_end = rotateUintCoord(blank_line_end, angle);
		line_start_fip.x = fip(line_start.x);
		line_start_fip.y = fip(line_start.y);
		line_end_fip.x = fip(line_end.x);
		line_end_fip.y = fip(line_end.y);
		draw_line(line_start_fip, line_end_fip);
		blank_line_start.x = 43;
		blank_line_start.y = 20;
		blank_line_end.x = 43;
		blank_line_end.y = 0;
		line_start = rotateUintCoord(blank_line_start, angle);
		line_end = rotateUintCoord(blank_line_end, angle);
		line_start_fip.x = fip(line_start.x);
		line_start_fip.y = fip(line_start.y);
		line_end_fip.x = fip(line_end.x);
		line_end_fip.y = fip(line_end.y);
		draw_line(line_start_fip, line_end_fip);
		blank_line_start.x = 44;
		blank_line_start.y = 20;
		blank_line_end.x = 44;
		blank_line_end.y = 0;
		line_start = rotateUintCoord(blank_line_start, angle);
		line_end = rotateUintCoord(blank_line_end, angle);
		line_start_fip.x = fip(line_start.x);
		line_start_fip.y = fip(line_start.y);
		line_end_fip.x = fip(line_end.x);
		line_end_fip.y = fip(line_end.y);
		draw_line(line_start_fip, line_end_fip);
		blank_line_start.x = 38;
		blank_line_start.y = 20;
		blank_line_end.x = 38;
		blank_line_end.y = 0;
		line_start = rotateUintCoord(blank_line_start, angle);
		line_end = rotateUintCoord(blank_line_end, angle);
		line_start_fip.x = fip(line_start.x);
		line_start_fip.y = fip(line_start.y);
		line_end_fip.x = fip(line_end.x);
		line_end_fip.y = fip(line_end.y);
		draw_line(line_start_fip, line_end_fip);
		blank_line_start.x = 37;
		blank_line_start.y = 20;
		blank_line_end.x = 37;
		blank_line_end.y = 0;
		line_start = rotateUintCoord(blank_line_start, angle);
		line_end = rotateUintCoord(blank_line_end, angle);
		line_start_fip.x = fip(line_start.x);
		line_start_fip.y = fip(line_start.y);
		line_end_fip.x = fip(line_end.x);
		line_end_fip.y = fip(line_end.y);
		draw_line(line_start_fip, line_end_fip);
		blank_line_start.x = 36;
		blank_line_start.y = 20;
		blank_line_end.x = 36;
		blank_line_end.y = 10;
		line_start = rotateUintCoord(blank_line_start, angle);
		line_end = rotateUintCoord(blank_line_end, angle);
		line_start_fip.x = fip(line_start.x);
		line_start_fip.y = fip(line_start.y);
		line_end_fip.x = fip(line_end.x);
		line_end_fip.y = fip(line_end.y);
		draw_line(line_start_fip, line_end_fip);*/
		
		
		
		//draw_arrow(angle);
		LCD_update();
		
		//compass_transmit((char) 0x12);
		//transmit_string(message, sizeof(message));
 	}
}

