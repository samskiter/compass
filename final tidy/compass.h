#ifndef COMPASS_H
#define COMPASS_H

volatile uint16_t angle;

void compass_init();
void update_angle();
void compass_transmit( unsigned char data );
uint8_t compass_receive();

#endif
