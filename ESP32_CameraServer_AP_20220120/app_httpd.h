#ifndef APP_H

#define APP_H


typedef struct
{
    uint16_t x_max, y_max, x_min, y_min;
    uint16_t x_cent;
    uint16_t y_cent;
    long sizePX;
    char colour;
    uint32_t sum_x, sum_y;
}blob_t;


typedef struct {float H; float S; float V;} hsv_t;


#endif