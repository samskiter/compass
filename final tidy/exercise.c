/* these two lines take care of the IO definitions for the device, and the interrupts available for them */
#define F_CPU 8E6	// 8MHz
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay_basic.h>
#include <util/delay.h>
#include <stdlib.h>
#include "compass.h"
#include "exercise.h"
#include "config.h"
#include "sincoslookup.h"
#include "graphic_lcd.h"
#include "graphing_library.h"
#include <math.h>

/* put comments in here for future reference, showing what the device is intended to do, which
device is to be used, and the fuse settings (and why) */

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


/* Port direction setting */
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
	
	// define  outputs for indicator LEDs
	DDRB |= (1<<PB0);
    DDRB |= (1<<PB2); 	
    
    //LCD outputs
    //TODO: this could be part of the LCD init function
    DDRD |= (1<< LCD_DC);
    DDRD |= (1<< LCD_RESET);
    DDRD |= (1<< XCK1);		//Setting the XCK1 port pin as a clock output, ie operating in master mode
    
    //Note we do not need to set the data direction for the USARTS for the compass and LCD, the data direction register values are overridden
	
	// Set high outputs

	// set low outputs
	PORTB = 0;
	PORTD = 0;
}

int main(void) {
	//there is often a reset shortly after start-up, so do nothing for a short while
	_delay_ms(100);
	
	// IO port initialisation
	port_direction_init();
	
	centre.x = (fip(LCD_WIDTH-1))/2;	//the X origin offset
	centre.y = (fip(LCD_HEIGHT-1))/2;	//the Y origin offset
	
	//set up the compass serial port
	compass_init();
	LCD_init();
	SREG |= 0x80;	//Set the global interrupt enable bit in the Status Register SREG, see section 6.3.1 of the datasheet.
	LCD_clear();
	
	uint8_t LCD_reset_counter = 0; //we use this to regularly re-init the LCD
	
	//north
	fixedPointLine northline1;
	northline1.start.x = centre.x;
	northline1.start.y = fip(0);
	northline1.end.x = fip(37);
	northline1.end.y = (centre.y);
	fixedPointLine northline2;
	northline2.start.x = (centre.x);
	northline2.start.y = fip(0);
	northline2.end.x = fip(46);
	northline2.end.y = (centre.y);
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
	horizontalline.start.y = (centre.y);
	horizontalline.end.x = fip(53);//46;
	horizontalline.end.y = (centre.y);
	//south
	fixedPointLine southline1;
	southline1.start.x = (centre.x);
	southline1.start.y = fip(47);
	southline1.end.x = fip(46);
	southline1.end.y = (centre.y);
	fixedPointLine southline2;
	southline2.start.x = (centre.x);
	southline2.start.y = fip(47);
	southline2.end.x = fip(37);
	southline2.end.y = (centre.y);
	
	fixedPointPolygon northTriangle;
	northTriangle.num_vertices = 4;
	fixedPointCoord vertices[northTriangle.num_vertices];
	vertices[0].x = (centre.x); //point
	vertices[0].y = fip(0);
	vertices[1].x = fip(46); //bottom right
	vertices[1].y = (centre.y);
	vertices[2].x = (centre.x); //middle arrow
	vertices[2].y = fip(30);
	vertices[3].x = fip(37); //bottom left
	vertices[3].y = (centre.y);
	northTriangle.vertices = vertices; //vertices points to the start of the array
	
	fixedPointPolygon halfnorthTriangle;
	halfnorthTriangle.num_vertices = 3;
	fixedPointCoord hnvertices[halfnorthTriangle.num_vertices];
	hnvertices[0].x = (centre.x); //point
	hnvertices[0].y = fip(0);
	hnvertices[1].x = fip(37); //bottom right
	hnvertices[1].y = (centre.y);
	hnvertices[2].x = (centre.x); //middle arrow
	hnvertices[2].y = (centre.y);
	halfnorthTriangle.vertices = hnvertices; //vertices points to the start of the array
	northline1.start.x = centre.x;
	northline1.start.y = fip(0);
	northline1.end.x = fip(46);
	northline1.end.y = (centre.y);
	
	fixedPointPolygon star;
	star.num_vertices = 12;
	fixedPointCoord svertices[star.num_vertices];
	svertices[0].x = (centre.x); //top point
	svertices[0].y = fip(8);
	svertices[1].x = (centre.x); //bottom right
	svertices[1].y = (centre.y-fip(5));
	svertices[1] = rotateCoord(svertices[1], 300);
	svertices[2] = rotateCoord(svertices[0], 600);
	svertices[3] = rotateCoord(svertices[1], 600);
	svertices[4] = rotateCoord(svertices[0], 1200);
	svertices[5] = rotateCoord(svertices[1], 1200);
	svertices[6] = rotateCoord(svertices[0], 1800);
	svertices[7] = rotateCoord(svertices[1], 1800);
	svertices[8] = rotateCoord(svertices[0], 2400);
	svertices[9] = rotateCoord(svertices[1], 2400);
	svertices[10] = rotateCoord(svertices[0], 3000);
	svertices[11] = rotateCoord(svertices[1], 3000);
	svertices[0].y = fip(0);
	svertices[1].y += fip(8);
	svertices[2].y += fip(8);
	svertices[3].y += fip(8);
	svertices[4].y += fip(8);
	svertices[5].y += fip(8);
	svertices[6].y += fip(8);
	svertices[7].y += fip(8);
	svertices[8].y += fip(8);
	svertices[9].y += fip(8);
	svertices[10].y += fip(8);
	svertices[11].y += fip(8);
	star.vertices = svertices;
	fixedPointLine starline1;
	starline1.start.x = centre.x;
	starline1.start.y = fip(0);
	starline1.end.x = centre.x - fip(3);
	starline1.end.y = fip(10);
	fixedPointLine starline2;
	starline2.start.x = (centre.x);
	starline2.start.y = fip(0);
	starline2.end.x = centre.x + fip(3);
	starline2.end.y = fip(10);
	
	fixedPointPolygon rotatedPoly;
	fixedPointEdgeTable globalEdgeTable;
	while (1) {
	//TODO: use a timed interrupt to send the LCD update
	//TODO: stop the LCD update during the compass update (MAYBE??)
	//TODO: name public and private headers
		update_angle();
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
		
		/*
		//draw_line_l(rotateLine(starline1, angle));
		//draw_line_l(rotateLine(starline2, angle));
		if (rotatePolygon(&star, angle, &rotatedPoly)) {
			if (initialise_global_edge_table(&rotatedPoly, &globalEdgeTable)) {
			//remember a logical check that after initialisation, there are at least 2 edges!
				draw_polygon(&globalEdgeTable);
				free(globalEdgeTable.edges);
				globalEdgeTable.edges = NULL;
			}
			free(rotatedPoly.vertices);
			rotatedPoly.vertices = NULL;
		}*/
		
		/*draw_line_l(rotateLine(southline1, angle));
		draw_line_l(rotateLine(southline2, angle));
		draw_line_l(rotateLine(northline1, angle));
		if (rotatePolygon(&halfnorthTriangle, angle, &rotatedPoly)) {
			if (initialise_global_edge_table(&rotatedPoly, &globalEdgeTable)) {
			//remember a logical check that after initialisation, there are at least 2 edges!
				draw_polygon(&globalEdgeTable);
				free(globalEdgeTable.edges);
				globalEdgeTable.edges = NULL;
			}
			free(rotatedPoly.vertices);
			rotatedPoly.vertices = NULL;
		}*/
		
		
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
		
		if ((LCD_reset_counter++) == 50) {
			LCD_init();
			LCD_reset_counter = 0;
		}
		LCD_update();

 	}
}

