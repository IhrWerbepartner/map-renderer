#ifndef __interface_h
#define __interface_h

#include "base.h"
#include <raylib.h>
#define TRUE 1
#define FALSE 0

extern int triangulate_polygon(int, int *, Vector2 (*), Triangle (*)); // TODO: change everything to work with F32 instead.
extern int is_point_inside_polygon(float *);

#endif /* __interface_h */
