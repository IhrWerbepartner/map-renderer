#pragma once
#include "fixed-array.c"
#include <raylib.h>

#include "arena.c"
#include <stdint.h>
#include <stdio.h>

#define DEBUG_MSG(...) fprintf(stderr, __VA_ARGS__);

#define ERROR_MSG(...)                                                         \
  fprintf(stderr, __VA_ARGS__);                                                \
  exit(EXIT_FAILURE);

#define ASSERT(expr, ...)                                                      \
  if (!(expr)) {                                                                 \
    fprintf(stderr, __VA_ARGS__);                                              \
    exit(EXIT_FAILURE);                                                        \
  }

typedef uint8_t U8;
typedef uint16_t U16;
typedef uint32_t U32;
typedef uint64_t U64;
typedef int8_t S8;
typedef int16_t S16;
typedef int32_t S32;
typedef int64_t S64;
typedef S8 B8;
typedef S16 B16;
typedef S32 B32;
typedef S64 B64;
typedef float F32;
typedef double F64;


typedef struct coord2 {
  F64 x, y;
} Coord2;

#define Vector2FromCoord2(coord) (Vector2) {.x = (F32) (coord).x, .y = (F32) (coord).y}

typedef struct triangle {
  S32 a, b, c;
} Triangle;

typedef struct slice {
  S32 start;
  S32 length;
} Slice; // index into the point2DArray

DeclFixedArray(Coord2Array, Coord2)
DeclFixedArray(Vector2Array, Vector2)

    typedef struct geo_properties {
  string8 key;
  void *val;
} GeoProperties;

typedef struct Point {
  Coord2 coordinates;
} Point;


typedef struct multi_point {
  Slice coordinates;
} MultiPoint;

typedef struct line_string {
  Slice coordinates;
} LineString;

DeclFixedArray(LineStringArray, LineString)

    typedef struct multi_line_string {
  Slice coordinates;
} MultiLineString;

typedef struct contour {
  Slice coords;
} Contour;

DeclFixedArray(TriangleArray, Triangle)

    typedef struct polygon {
  U32 outside_coordinates;
  Slice inside_coordinates;
} Polygon;

DeclFixedArray(PolygonArray, Polygon)

    typedef struct multi_polygon {
  Slice polygons;
} MultiPolygon;

DeclFixedArray(PointArray, Point) DeclFixedArray(MultiPointArray, MultiPoint)
    DeclFixedArray(MultiLineStringArray, MultiLineString)
        DeclFixedArray(GeoPropertiesArray, GeoProperties)
            DeclFixedArray(MultiPolygonArray, MultiPolygon)

                typedef struct geo_json {
  enum geo_json_type {
    FEATURE_COLLECTION,
  } type;

  // SoA layout
  PointArray interest_points; // all interest points
                              //
  Coord2Array multi_point_coords;
  MultiPointArray multi_points; // has a slice into coords

  Coord2Array line_string_coords;
  LineStringArray line_strings; // has a slice into coords

  Coord2Array multi_line_string_coords;
  LineStringArray multi_line_string_array; // has a slice into coords
  MultiLineStringArray
      multi_line_strings; // has an array of lineStrings

  Coord2Array polygon_coords;
  TriangleArray polygon_triangles; // triangle indices point into polygon_coords

  Coord2Array multi_polygon_coords;
  TriangleArray multi_polygon_triangles; // has a slice into coords
  MultiPolygonArray
      multi_polygons; // has an array of slices into array of triangles

  GeoPropertiesArray properties; // TODO: add a handle in the types for as an
                                 // index in to the properties array
} Geo_Json;
