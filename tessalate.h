#pragma once
#include "base.h"

// TODO: implement tessalation


// computes the triangulation of the coordinates which are partitioned by countour_sizes
// so coords[0..cs[0]] polygons are the first ones, coords[cs[0]..cs[1]] the next etc.
// returns the triangles as indices into the coords array.
TriangleArray *tessalate_polygon(Arena *arena, Coord2Array coords, IntSlice contour_sizes);
