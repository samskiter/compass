#ifndef GLIBRARY_H
#define GLIBRARY_H

#include "exercise.h"

fixedPointCoord centre;

typedef struct {
	fixedPointCoord start;
	fixedPointCoord end;
} fixedPointLine;

typedef struct {
	fixedPoint ymin;
	fixedPoint ymax;
	fixedPoint x; // initially x value associated with the minimum y, then becomes the current x value of the edge as it is drawn
	int8_t xlim; // x value limit associated with the maximum y
	uint8_t type; //this is to determine whether the edge is to the right or left of the filled area (its arbitrary however)
	fixedPoint slope; //holds the slope of the edge
	edgeStatus status; //whether the edge is inactive, active, or complete (preferred over mallocing new memory for an active_edge table)
} fixedPointEdge;

typedef struct {
	uint8_t num_vertices;
	fixedPointCoord* vertices;
} fixedPointPolygon;

//this struct is probably not necessary - its just an array?
typedef struct {
	uint8_t num_edges;
	fixedPointEdge* edges;
} fixedPointEdgeTable;

void applyPixelStretchCorrection(fixedPointCoord* p);
fixedPointCoord rotateCoord(fixedPointCoord pf, uint16_t theta);
fixedPointLine rotateLine(fixedPointLine line, uint16_t theta);
bool rotatePolygon(fixedPointPolygon* poly, uint16_t theta, fixedPointPolygon* fppoly);

bool initialise_global_edge_table(fixedPointPolygon* poly, fixedPointEdgeTable* edge_table);
void draw_polygon(fixedPointEdgeTable* global_edge_table);

void draw_line(fixedPointCoord start, fixedPointCoord end);
void draw_line_l(fixedPointLine line);
void line_algorithm(fixedPointCoord start, fixedPointCoord end, int8_t transpose);


#endif
