#pragma once
#include "fixed-array.c"
#include <raylib.h>
#include "string8.c"

#include "arena.c"
#include <stdio.h>
#include <stdint.h>
#define DEBUG_MSG(...) fprintf(stderr, __VA_ARGS__);

#define ERROR_MSG(...)                                                         \
  fprintf(stderr, __VA_ARGS__);                                                \
  exit(EXIT_FAILURE);

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

typedef struct {
  S32 a, b, c;
} Triangle;

typedef struct slice {
  U32 start;
  U32 length;
} Slice; // index into the point2DArray

DeclFixedArray(Vector2Array, Vector2)

    typedef struct geo_properties {
  string8 key;
  void *val;
} GeoProperties;

typedef struct Point {
  Vector2 coordinates;
} Point;

typedef struct multi_point {
  Slice coordinates;
} MultiPoint;

typedef struct line_string {
  Slice coordinates;
} LineString;

DeclFixedArray(LineStringArray, LineString)

    typedef struct multi_line_string {
  LineStringArray coordinates;
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
  Vector2Array multi_point_coords;
  MultiPointArray multi_points; // has a slice into coords

  Vector2Array line_string_coords;
  LineStringArray line_strings; // has a slice into coords

  Vector2Array multi_line_string_coords;
  LineStringArray multi_line_string_array; // has a slice into coords
  MultiLineStringArray
      multi_line_strings; // has an array of lineStrings
                          // TODO: split polygons into triagnles as we need
                          // concave ones(?)
  Vector2Array polygon_coords;
  TriangleArray triangulated_polygons; // triangle indices point into polygon_coords

  Vector2Array multi_polygon_coords;
  PolygonArray multi_polygon_array; // has a slice into coords
  MultiPolygonArray
      multi_polygons; // has an array of lices into array of polygons

  GeoPropertiesArray properties; // TODO: add a handle in the types for as an
                                 // index in to the properties array
} Geo_Json;
