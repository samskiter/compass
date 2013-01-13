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

fixedPointCoord fipCoord(doubleCoord c) {
	fixedPointCoord tmpCoord;
	tmpCoord.x = fip(c.x);
	tmpCoord.y = fip(c.y);
	return tmpCoord;
}

doubleCoord unFipCoord(fixedPointCoord c) {
	doubleCoord tmpCoord;
	tmpCoord.x = unfip(c.x);
	tmpCoord.y = unfip(c.y);
	return tmpCoord;
}

//TODO: rename to rotatefloatcoord
fixedPointCoord rotateDoubleCoord(fixedPointCoord pf, float theta) {
//function that implicitly rotates about the centre of the LCD by an angle theta and returns an x,y, fixed point coordinate
	doubleCoord p = unFipCoord(pf);
	p.x = p.x - centre.x;	//the difference between the distance to the origin and the x coord
	p.y = centre.y - p.y;	//the difference between the distance to the origin and the y coord
	float costheta = cosf(theta);
	float sintheta = sinf(theta);
	doubleCoord pr;
	pr.x = centre.x + (1.15*((costheta*p.x) - (sintheta*p.y))); //here we apply a correction factor for the pixel aspec ratio
	pr.y = centre.y - ((sintheta*p.x) + (costheta*p.y)); 
	pf = fipCoord(pr);
	return pf;
}

fixedPointLine rotateLine(fixedPointLine line, float theta) {
	line.start = rotateDoubleCoord(line.start, theta);
	line.end = rotateDoubleCoord(line.end, theta);
	return line;
}

bool rotatePolygon(fixedPointPolygon* poly, float theta, fixedPointPolygon* fppoly) {
//use a pointer to a pointer to a FPPolygon as we will create it and worry about valid Malloc here (and return bool) - therefore we need to be able to assign the FPPolygon back to whatever is calling
	int vertex;
	fixedPointCoord* fpvertices = calloc(poly->num_vertices, sizeof(fixedPointCoord));
	if (NULL == fpvertices) {
		return false; //error
	}
	fppoly->vertices = fpvertices; //we have now successfully assigned enough memory to do this
	fppoly->num_vertices = poly->num_vertices;
	for (vertex = 0; vertex < poly->num_vertices; vertex++) {
		//index both ptrs to respective arrays
		fppoly->vertices[vertex] = rotateDoubleCoord(poly->vertices[vertex], theta);
	}
	return true;
}

//this function initialises all the edges and orders them into the edge table sorted by increasing ymin and xmin
bool initialise_global_edge_table(fixedPointPolygon* poly, fixedPointEdgeTable* edge_table) {
	//malloc
	fixedPointEdge* fpedges = calloc(poly->num_vertices, sizeof(fixedPointEdge)); //we must assign enough memory for every edge to be in the edge table, even though some may be discarded
	if (NULL == fpedges) {
		return false; //error, not enough space to make this
	}
	edge_table->edges = fpedges; //we have now successfully assigned enough memory to do this
	edge_table->num_edges = 0; //we will increment this as we add each edge to the edge table
	fixedPointCoord start_vertex;
	fixedPointCoord end_vertex;
	fixedPointLine last_line;
	uint8_t vertex;
	uint8_t curr_edge_type = 0;
	for (vertex = 0; vertex < poly->num_vertices; vertex++) { //for every vertex, make an edge using it and the previous vertex
		start_vertex = poly->vertices[vertex];
		if (vertex == 0) {
			end_vertex = poly->vertices[poly->num_vertices - 1]; //pull out the last vertex
		} else {
			end_vertex = poly->vertices[vertex - 1];
		}
		
		//make sure 'start' has the lowest y
		if (start_vertex.y > end_vertex.y) {
			fixedPointCoord tmp = start_vertex;
			start_vertex = end_vertex;
			end_vertex = tmp;
		}
		fixedPoint dy = end_vertex.y - start_vertex.y;
		if (roundfip(dy) == 0) {
			if (last_line.end.x != NULL) {
				//this section moves the end of the previous line onto the end of this horizontal line
				if (start_vertex.x == last_line.end.x) {
					//if the start vertex matches the top of the previous line
					last_line.end.x = end_vertex.x;
					last_line.end.y = end_vertex.y;
				} else if (start_vertex.x == last_line.start.x) {
					//if the start vertex happens to match the bottom of the previous line
					last_line.start.x = end_vertex.x;
					last_line.start.y = end_vertex.y;
				} else if (end_vertex.x == last_line.end.x) {
					//if the start vertex happens to match the top of the previous line
					last_line.end.x = start_vertex.x;
					last_line.end.y = start_vertex.y;
				}  else if (end_vertex.x == last_line.start.x) {
					//if the start vertex happens to match the bottom of the previous line
					last_line.start.x = start_vertex.x;
					last_line.start.y = start_vertex.y;
				}
			}
			continue; //we dont need to draw horizontal lines (or close-to horizontal lines!), other edges will take care of this
		}
		
		fixedPoint dx = end_vertex.x - start_vertex.x;
		fixedPointEdge new_edge;
		new_edge.slope = fip(dx)/dy; //be careful of divides and multiplies
		new_edge.ymin = start_vertex.y;
		new_edge.x = start_vertex.x; //start vertex
		new_edge.ymax = end_vertex.y;
		new_edge.xlim = roundfip(end_vertex.x);
		new_edge.status = INACTIVE;
		if (((start_vertex.x == last_line.start.x) && (start_vertex.y == last_line.start.y))
			|| ((end_vertex.x == last_line.end.x) && (end_vertex.y == last_line.end.y))) {
			//changing direction
			curr_edge_type = 1 - curr_edge_type; //invert
		}
		last_line.start.x = start_vertex.x;
		last_line.start.y = start_vertex.y;
		last_line.end.x = end_vertex.x;
		last_line.end.y = end_vertex.y;
		new_edge.type = curr_edge_type;
		//now find out where the new edge goes in the edge table (order by ymin then xmin, then slope)
		uint8_t current_edge;
		uint8_t new_position = edge_table->num_edges; //default position is to put it on the end
		uintCoord current;
		current.y = roundfip(start_vertex.y);
		current.x = roundfip(start_vertex.x) + (vertex*10) + 3;
		LCD_set_pixel(current);
		current.y = roundfip(end_vertex.y);
		current.x = roundfip(end_vertex.x) + (vertex*10) + 3;
		LCD_set_pixel(current);
		for (current_edge = 0; current_edge < edge_table->num_edges; current_edge++) {
			if (edge_table->edges[current_edge].ymin > new_edge.ymin) { //if the current edge starts above the new edge
				new_position = current_edge;
				break;
			} else if (edge_table->edges[current_edge].ymin == new_edge.ymin) {
				if (edge_table->edges[current_edge].x > new_edge.x) { // if the current edge starts to the right of the new edge
					//add the edge where current_edge is
					new_position = current_edge;
					break;
				} else if (edge_table->edges[current_edge].x == new_edge.x) { //if these two edges start at the same point (and diverge)
					//sort by slope
					
					if (edge_table->edges[current_edge].slope > new_edge.slope) { //if the current edge slopes rightward of the new edge
						new_position = current_edge;
						break;
					}
				}
			}
		}
		//insert the new edge in the new_position in the edge matrix (involves bumping the others up)
		for (current_edge = edge_table->num_edges; current_edge > new_position; current_edge--) {
			edge_table->edges[current_edge] = edge_table->edges[current_edge-1];
		}
		edge_table->edges[new_position] = new_edge;
		edge_table->num_edges++;
	}
	return true;
}

void draw_polygon(fixedPointEdgeTable* global_edge_table) {
	//initialise parity
	uint8_t parity = 0; //0 represents 'not inside polygon'
	uint8_t last_type = 2;
	//initialise the scan-line - set to lowest y value
	int8_t scan_line = roundfip(global_edge_table->edges[0].ymin);
	
	//initialise active edge table (size should be number of edges in the global edge table)
	/*fixedPointEdgeTable active_edge_table;
	fixedPointEdge* fpedges = calloc(global_edge_table->num_edges, sizeof(fixedPointEdge));
	if (NULL == fpedges) {
		return false; //error, not enough space to make this
	}
	active_edge_table.edges = fpedges; //we have now successfully assigned enough memory to do this
	*/
	uint8_t active_edges = 0;
	//loop the global edge table and update the edges for this scan line
	uint8_t current_edge;
	fixedPointEdge start_edge;
	uintCoord current;
	
	while(1) {
		for (current_edge = 0; current_edge < global_edge_table->num_edges; current_edge++) {
			if (global_edge_table->edges[current_edge].status == ACTIVE) {
				//evaluate if this edge should be deactivated
				if (global_edge_table->edges[current_edge].ymax < (fip(scan_line) - fip(0.5))) { //we subtract 0.5 because we want the scan line to be beyond the end of the edge before disabling it
					//deactivate it
					global_edge_table->edges[current_edge].status = COMPLETE;
					active_edges--;
				} else {
					//increment the pixel's x value
					global_edge_table->edges[current_edge].x += global_edge_table->edges[current_edge].slope;
					//put a cap on x values - this fixes issues with very high gradients creating lines off the polygon
					if (((global_edge_table->edges[current_edge].slope > 0) && (roundfip(global_edge_table->edges[current_edge].x) > global_edge_table->edges[current_edge].xlim))
						|| ((global_edge_table->edges[current_edge].slope < 0) && (roundfip(global_edge_table->edges[current_edge].x) < global_edge_table->edges[current_edge].xlim))) {
						global_edge_table->edges[current_edge].x = fip(global_edge_table->edges[current_edge].xlim);
					}
				}
			} else if (global_edge_table->edges[current_edge].status == INACTIVE) {
				//evaluate if this edge should be activated
				if (global_edge_table->edges[current_edge].ymin < (fip(scan_line) + fip(0.5))) { //we add 0.5 because if the scan line were at 0, we would want to pick up ymins from -0.5 to 0.5
					global_edge_table->edges[current_edge].status = ACTIVE;
					active_edges++;
					//we need to make sure this edge is in the right place
					int8_t past_edge;
					for (past_edge = (current_edge-1); past_edge >= 0; past_edge--) { //iterate back down through the older (active/complete) edges and find any active ones with greater x
						if (global_edge_table->edges[current_edge].status == ACTIVE) {
							if ((global_edge_table->edges[current_edge].x < global_edge_table->edges[past_edge].x) 
								&& (global_edge_table->edges[past_edge].ymax != global_edge_table->edges[current_edge].ymin)) {
								//the above if statement stops a new edge that is continuing upward from an older edge from swapping with it while they are both active
								//this is relevant as they are allowed to both be active simultaneously (as the +-0.5 has to account for inflection points)
								current.y = scan_line;
								current.x = past_edge;
								LCD_set_pixel(current);
								fixedPointEdge tmp_edge = global_edge_table->edges[current_edge]; //this assignment copies the data across (there are no pointers in a fixepoint edge)
								global_edge_table->edges[current_edge] = global_edge_table->edges[past_edge];
								global_edge_table->edges[past_edge] = tmp_edge;
							}
						}
					}
				}
			}
		}
		
		if (active_edges == 0) break;
		parity = 1;
		last_type = 2; //2 is a special value meaning uninitialised
		current.y = 0;
		current.x = 0;
		LCD_set_pixel(current);
		current.y = scan_line;
		for (current_edge = 0; current_edge < global_edge_table->num_edges; current_edge++) {
			if (global_edge_table->edges[current_edge].status == ACTIVE) {
				if(last_type == 2) last_type = 1 - global_edge_table->edges[current_edge].type; //initialise last_type
				if (global_edge_table->edges[current_edge].type != last_type) {
					parity = 1 - parity;
					if (parity == 0) {
						start_edge = global_edge_table->edges[current_edge];
					}/* else if (start_edge.ymax != global_edge_table->edges[current_edge].ymin) { //this accounts for the case when 2 connected edges which continue upward are active at the same time
						for (current.x = roundfip(start_edge.x); fip(current.x) <= (global_edge_table->edges[current_edge].x + fip(0.5)); current.x++) {
							LCD_set_pixel(current);
						}
					}*/ else {
						for (current.x = roundfip(start_edge.x); fip(current.x) <= (global_edge_table->edges[current_edge].x + fip(0.5)); current.x++) {
							LCD_set_pixel(current);
						}
					}
				}
				last_type = global_edge_table->edges[current_edge].type;
			}
		}
		scan_line++;
	}
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
		start_t.x = start.y;
		start_t.y = start.x;
		end_t.x = end.y;
		end_t.y = end.x;
		//we use the transpose indicator to flip the algorithm to increment along the y direction
		line_algorithm(start_t, end_t, 1);
	}
}

void draw_line_l(fixedPointLine line) {
	draw_line(line.start, line.end);
}

//subpixel line drawing is pretty: http://www.antigrain.com/doc/introduction/introduction.agdoc.html (Bresenham's)
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
	
	//north
	fixedPointLine northline1;
	northline1.start.x = fip(centre.x);
	northline1.start.y = fip(0);
	northline1.end.x = fip(37);
	northline1.end.y = fip(centre.y);
	fixedPointLine northline2;
	northline2.start.x = fip(centre.x);
	northline2.start.y = fip(0);
	northline2.end.x = fip(46);
	northline2.end.y = fip(centre.y);
	//north small
	fixedPointLine northsmallline1;
	northsmallline1.start.x = fip(41.5);
	northsmallline1.start.y = fip(15);
	northsmallline1.end.x = fip(46);
	northsmallline1.end.y = fip(23.5);
	fixedPointLine northsmallline2;
	northsmallline2.start.x = fip(41.5);
	northsmallline2.start.y = fip(15);
	northsmallline2.end.x = fip(37);
	northsmallline2.end.y = fip(23.5);
	//horizontal line
	fixedPointLine horizontalline;
	horizontalline.start.x = fip(30);//37;
	horizontalline.start.y = fip(centre.y);
	horizontalline.end.x = fip(53);//46;
	horizontalline.end.y = fip(centre.y);
	//south
	fixedPointLine southline1;
	southline1.start.x = fip(centre.x);
	southline1.start.y = fip(47);
	southline1.end.x = fip(46);
	southline1.end.y = fip(centre.y);
	fixedPointLine southline2;
	southline2.start.x = fip(centre.x);
	southline2.start.y = fip(47);
	southline2.end.x = fip(37);
	southline2.end.y = fip(centre.y);
	
	fixedPointPolygon northTriangle;
	northTriangle.num_vertices = 4;
	fixedPointCoord vertices[northTriangle.num_vertices];
	vertices[0].x = fip(centre.x); //point
	vertices[0].y = fip(0);
	vertices[1].x = fip(46); //bottom right
	vertices[1].y = fip(centre.y);
	vertices[2].x = fip(centre.x); //middle arrow
	vertices[2].y = fip(19);
	vertices[3].x = fip(37); //bottom left
	vertices[3].y = fip(centre.y);
	northTriangle.vertices = vertices; //vertices points to the start of the array
	
	fixedPointPolygon rotatedPoly;
	fixedPointEdgeTable globalEdgeTable;
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
		//draw_line_l(rotateLine(northline1, angle));
		//draw_line_l(rotateLine(northline2, angle));
		//north small
		/*draw_line_l(rotateLine(northsmallline1, angle));
		draw_line_l(rotateLine(northsmallline2, angle));*/
		//horizontal line
		//draw_line_l(rotateLine(horizontalline, angle));
		//south
		draw_line_l(rotateLine(southline1, angle));
		
		if (rotatePolygon(&northTriangle, angle, &rotatedPoly)) {
			if (initialise_global_edge_table(&rotatedPoly, &globalEdgeTable)) {
			//remember a logical check that after initialisation, there are at least 2 edges!
				draw_polygon(&globalEdgeTable);
				free(globalEdgeTable.edges);
				globalEdgeTable.edges = NULL;
				draw_line_l(rotateLine(southline2, angle));
			}
			free(rotatedPoly.vertices);
			rotatedPoly.vertices = NULL;
		}
		
		//steps:
		//rotate polygon - check
		//http://www.cs.rit.edu/~icss571/filling/how_to.html
		//initializing should return a bool for successful or not (cos of mallocing)
		//initialize edges
		//initializing global edge table (sorted) (size should be total number of edges)
		//initializing parity
		//initializing the scan-line
		//initializing active edge table (size should be number of edges)
		//fill polygon
		//aim for a draw_scene function (pass an array of polygons and an angle)
		//  if N is 2 for a polygon, cast its array to a line

		LCD_update();

 	}
}

