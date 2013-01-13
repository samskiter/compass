#ifndef EXERCISE_H
#define EXERCISE_H

#define FIXED_POINT 100
#define fixedPoint int32_t

typedef enum { false, true } bool;

typedef enum { INACTIVE, ACTIVE, COMPLETE } edgeStatus;

typedef struct {
    fixedPoint x;
    fixedPoint y;
} fixedPointCoord;

typedef struct {
//this struct is designed for coordinates from the corner of an image (ie never negative)
    uint8_t x;
    uint8_t y;
} uintCoord;

#define round(x) (((x)>=0)?(uint8_t)((x)+0.5):(uint8_t)((x)-0.5))
#define fip(x) ((fixedPoint)(((float)(x))*FIXED_POINT))
#define absint(x) (((x)<(fixedPoint)0)?(((fixedPoint)-1)*(x)):(x))
#define unfip(x) (float)((float)(x)/FIXED_POINT)
#define roundfip(x) round((float)(((float)x)/FIXED_POINT))
#define multfip(x) ((x)/FIXED_POINT) //to be used whenever two FIPs are multiplied together

inline void port_direction_init();

int main(void);

#endif
