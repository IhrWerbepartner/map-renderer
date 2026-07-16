#pragma once
#include "fixed-array.c"
#include <raylib.h>

#include <stdint.h>

#define DEBUG_MSG(...) fprintf(stderr, __VA_ARGS__);

#define ERROR_MSG(...)                                                         \
  fprintf(stderr, __VA_ARGS__);                                                \
  exit(EXIT_FAILURE);

#define ASSERT(expr, ...)                                                      \
  if (!(expr)) {                                                               \
    fprintf(stderr, __VA_ARGS__);                                              \
    exit(EXIT_FAILURE);                                                        \
  }

#define Min(A, B) (((A) < (B)) ? (A) : (B))
#define Max(A, B) (((A) > (B)) ? (A) : (B))

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

#define Vector2FromCoord2(coord)                                               \
  (Vector2) { .x = (F32)(coord).x, .y = -(F32)(coord).y }

typedef struct triangle {
  S32 a, b, c;
} Triangle;

typedef struct slice {
  S32 start;
  S32 length;
} Slice; // index into the point2DArray

DeclFixedArray(Coord2Array, Coord2);
DeclFixedArray(Vector2Array, Vector2);
DeclFixedArray(TriangleArray, Triangle);
DeclFixedArray(U32Array, U32);
DeclFixedArray(S32Array, S32);

static Arena arena1 = {0};
static Arena arena2 = {0};
static Arena *arenas[2] = {&arena1, &arena2};
