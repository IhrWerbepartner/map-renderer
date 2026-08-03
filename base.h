#pragma once
#include "fixed-array.c"
#include <raylib.h>

#include <stdint.h>

#define DEBUG_MSG(...) fprintf(stderr, __VA_ARGS__);

#define ERROR_MSG(...)                                                         \
  fprintf(stderr, __VA_ARGS__);                                                \
  exit(EXIT_FAILURE);

#define ASSERT(expr, ...)                                                      \
  do {                                                                         \
    if (!(expr)) {                                                             \
      fprintf(stderr, __VA_ARGS__);                                            \
      exit(EXIT_FAILURE);                                                      \
    }                                                                          \
  } while (0);

#define CROSS(v0, v1, v2)                                                      \
  (((v1).x - (v0).x) * ((v2).y - (v0).y) -                                     \
   ((v1).y - (v0).y) * ((v2).x - (v0).x))

#define DOT(v0, v1) ((v0).x * (v1).x + (v0).y * (v1).y)

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

#define KB(n) (((U64)(n)) << 10)
#define MB(n) (((U64)(n)) << 20)
#define GB(n) (((U64)(n)) << 30)
#define TB(n) (((U64)(n)) << 40)
#define Thousand(n) ((n) * 1000)
#define Million(n) ((n) * 1000000)
#define Billion(n) ((n) * 1000000000)

#define Min(A, B) (((A) < (B)) ? (A) : (B))
#define Max(A, B) (((A) > (B)) ? (A) : (B))
#define ClampTop(A, X) Min(A, X)
#define ClampBot(X, B) Max(X, B)
//#define Clamp(A, X, B) (((X) < (A)) ? (A) : ((X) > (B)) ? (B) : (X))

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
