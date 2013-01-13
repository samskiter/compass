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
 * SUT1   	1 (fast rising power)
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
 U2X0	0 - turn off double speed
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
 TXB80	0 - not using 9th bit of communication
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
 U2X0	0 - turn off double speed
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
 TXB80	0 - not using 9th bit of communication
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
// If we were using interrupts the interrupt vectors would be here
// there is a useful list at    http://www.nongnu.org/avr-libc/user-manual/group__avr__interrupts.html


/* Port direction setting */

uint8_t LCD_buffer[LCD_ROWS][LCD_WIDTH];
uint8_t LCD_current_mode = LCD_DATA_MODE; //this tracks the data/command mode of the LCD
uint8_t arrow[ARROW_ROWS][ARROW_WIDTH] =
	{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0xF0, 0xFF, 0xFF, 0xF0, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
volatile float angle = 0;
volatile int updated = 0;
	
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
	//TODO: check the range of the pixels
	div_t y_row = div(p.y,LCD_BYTE_HEIGHT);
	LCD_buffer[y_row.quot][p.x] |= (1<<y_row.rem);
}

void LCD_set_pixel_polar(polarCoord c) { //set from the centre
	double x = (LCD_WIDTH/2.0) - 0.5;	//the X origin offset
	double y = (LCD_HEIGHT/2.0)  - 0.5;	//the Y origin offset
	x = x + c.r*cos(c.theta);	//the sum of the distance to the origin and the x coord
	y = y - c.r*sin(c.theta);	//the difference between the distance to the origin and the y coord
	uintCoord p = {.x = round(x), .y = round(y)};
	LCD_set_pixel(p);
}

void USART_transmit( unsigned char data ) {
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );

	/* Put data into buffer, sends the data */
	UDR0 = data;	
}

void USART_transmit_uint8(uint8_t val) {
	unsigned char buf[3];
	int8_t ptr;
	for(ptr=0;ptr<3;++ptr) {
		buf[ptr] = (val % 10) + '0';
		val /= 10;
	}
	for(ptr=2;ptr>0;--ptr) {
		if (buf[ptr] != '0') break;
	}
	for(;ptr>=0;--ptr) {
		USART_transmit(buf[ptr]);
	}
}

void transmit_chars( unsigned char data[], unsigned int length ) {
	int i;
	for (i = 0; i < length; i++) {
		USART_transmit(data[i]);
	}
}

void transmit_string( unsigned char data[], unsigned int length ) {
	transmit_chars( data, length );
	transmit_chars( "\r\n", 2 );
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
	angle = PI * received/128;
	PORTB ^= (1<<PB2);
	updated = 1;
}


void draw_arrow(float angle) { //angle in radians
	uint8_t x;
	uint8_t y_row;
	uint8_t y_bit;
	//we loop (inside to out) firstly along the bits of each byte
	// then along the bytes of each row
	// then along the rows
	for(y_row = 0; y_row < ARROW_ROWS; y_row++) {
		for(x = 0; x < ARROW_WIDTH; x++) {
			for(y_bit = 0; y_bit < 8; y_bit++) {
				if (arrow[y_row][x] & (1<<(y_bit))) {
					//this bit is set so we should set it in the buffer
					polarCoord c = get_polar_image_coord(
						x,
						(y_row*8)+y_bit,
						ARROW_HEIGHT,
						ARROW_WIDTH
					);
					c.theta += angle;
					LCD_set_pixel_polar(c);
				}
			}
		}
	}
	
	//rotate all the image vertices into new coordinates
	//run poygon fill/line drawing on the new structure, each polygon at a time
}

//polygon fill
//scan each line in turn


polarCoord get_polar_image_coord(uint8_t x, uint8_t y, uint8_t height, uint8_t width) { //x and y measured in pels from top left, x horizontal, y vertical, from 0 to ARROW_HEIGHT-1
	double x_pos = (width/2.0)  - 0.5;	//the X origin offset
	double y_pos = (height/2.0) - 0.5;	//the Y origin offset
	x_pos = x - x_pos;	//the difference between the distance to the origin and the x coord
	y_pos = y_pos - y;	//the difference between the distance to the origin and the y coord
	polarCoord c;
	c.r = sqrt((x_pos*x_pos) + (y_pos*y_pos));
	c.theta = atan2(y_pos, x_pos);
	return c;
}

doubleCoord rotateDoubleCoord(doubleCoord p, double theta) {
//function that implicitly rotates about the centre of the LCD by an angle theta and returns an x,y, floating point coordinate
	doubleCoord centre;
	centre.x = (LCD_WIDTH/2.0)  - 0.5;	//the X origin offset
	centre.y = (LCD_HEIGHT/2.0) - 0.5;	//the Y origin offset
	p.x = p.x - centre.x;	//the difference between the distance to the origin and the x coord
	p.y = centre.y - p.y;	//the difference between the distance to the origin and the y coord
	polarCoord c;
	c.r = sqrt((p.x*p.x) + (p.y*p.y));
	c.theta = atan2(p.y, p.y);
	c.theta += angle;
	p.x = centre.x + c.r*cos(c.theta);	//the sum of the distance to the origin and the x coord
	p.y = centre.y - c.r*sin(c.theta);	//the difference between the distance to the origin and the y coord
	return p;
}

//function that implicitly rotates about the centre of the LCD by an angle theta and returns an x,y, floating point coordinate
doubleCoord rotateUintCoord(uintCoord input, double theta) {
	doubleCoord i = {.x = input.x, .y = input.y};
	return rotateDoubleCoord(i, theta);
}


void draw_line(doubleCoord start, doubleCoord end) {
	double dx = end.x - start.x;
	double dy = end.y - start.y;
	uintCoord current; //records the current pixel we are writing to
	//pick the mode that will increment along the correct dimension
	if (dx >= dy) {
		//we are now incrementing along the x direction
		//initialise and draw the first point
		double d = dy/dx;
		current.x = round(start.x);
		double yf = start.y + d * (current.x - start.x);
		current.y = round(yf);
		LCD_set_pixel(current);
		
		//make sure the iteration does the right way!
		int x_dir = 1;
		if (dx < 0) x_dir = -1;
		while (current.x < (end.x - 0.5)) {
			current.x += x_dir;
			yf += d;
			if (yf > 0.5) {
				current.y += 1;
				yf -= 1;
			}
			LCD_set_pixel(current);
		}
	} else {
		//we are now incrementing along the x direction
		//initialise and draw the first point
		double d = dx/dy;
		current.y = round(start.y);
		double xf = start.x + d * (current.y - start.y);
		current.x = round(xf);
		LCD_set_pixel(current);
		
		//make sure the iteration does the right way!
		int y_dir = 1;
		if (dy < 0) y_dir = -1;
		while (current.y < (end.y - 0.5)) {
			current.y += y_dir;
			xf += d;
			if (xf > 0.5) {
				current.x += 1;
				xf -= 1;
			}
			LCD_set_pixel(current);
		}
	}
}

int main(void) {
_delay_ms(100);
	// IO port initialisation
	port_direction_init();
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
		uintCoord blank_line_start = {.x = 41, .y = 20};
		uintCoord blank_line_end = {.x = 41, .y = 0};
		doubleCoord line_start = rotateUintCoord(blank_line_start, angle);
		doubleCoord line_end = rotateUintCoord(blank_line_end, angle);
		
		draw_line(line_start, line_end);
		
		//draw_arrow(angle);
		LCD_update();
		
		//compass_transmit((char) 0x12);
		//transmit_string(message, sizeof(message));
 	}
}

