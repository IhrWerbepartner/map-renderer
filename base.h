#pragma once
#include "fixed-array.c"
#include <assert.h>
#include <float.h>
#include <raylib.h>

#include <stdint.h>

#define internal static
#define global static

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

// global U64 max_U64 = 0xffffffffffffffffull;
global const U32 max_U32 = 0xffffffff;
// global U16 max_U16 = 0xffff;
// global U8 max_U8 = 0xff;
//
// global S64 max_S64 = (S64)0x7fffffffffffffffll;
global const S32 max_S32 = (S32)0x7fffffff;
// global S16 max_S16 = (S16)0x7fff;
// global S8 max_S8 = (S8)0x7f;
//
// global S64 min_S64 = (S64)0x8000000000000000ll;
global S32 min_S32 = (S32)0x80000000;
// global S16 min_S16 = (S16)0x8000;
// global S8 min_S8 = (S8)0x80;
//
global const F64 max_F64 = (F64)DBL_MAX;
global const F64 min_F64 = -(F64)DBL_MAX;

static U32 safe_cast_u32(U64 x) {
  assert(x <= max_U32);
  U32 result = (U32)x;
  return result;
}

static S32 safe_cast_s32(S64 x) {
  assert(x <= max_S32);
  S32 result = (S32)x;
  return result;
}

static S32 safe_cast_s32_from_u64(U64 x) {
  assert(x <= (U32)max_S32);
  S32 result = (S32)x;
  return result;
}

static U64 safe_cast_u64_from_s32(S32 x) {
  assert(x >= 0);
  U64 result = (U64)x;
  return result;
}

static S32 safe_cast_s32_from_u32(U32 x) {
  assert(x <= (U32)max_S32);
  S32 result = (S32)x;
  return result;
}

global const U32 bitmask1 = 0x00000001;
global const U32 bitmask2 = 0x00000003;
global const U32 bitmask3 = 0x00000007;
global const U32 bitmask4 = 0x0000000f;
global const U32 bitmask5 = 0x0000001f;
global const U32 bitmask6 = 0x0000003f;
global const U32 bitmask7 = 0x0000007f;
global const U32 bitmask8 = 0x000000ff;
global const U32 bitmask9 = 0x000001ff;
global const U32 bitmask10 = 0x000003ff;
global const U32 bitmask11 = 0x000007ff;
global const U32 bitmask12 = 0x00000fff;
global const U32 bitmask13 = 0x00001fff;
global const U32 bitmask14 = 0x00003fff;
global const U32 bitmask15 = 0x00007fff;
global const U32 bitmask16 = 0x0000ffff;
global const U32 bitmask17 = 0x0001ffff;
global const U32 bitmask18 = 0x0003ffff;
global const U32 bitmask19 = 0x0007ffff;
global const U32 bitmask20 = 0x000fffff;
global const U32 bitmask21 = 0x001fffff;
global const U32 bitmask22 = 0x003fffff;
global const U32 bitmask23 = 0x007fffff;
global const U32 bitmask24 = 0x00ffffff;
global const U32 bitmask25 = 0x01ffffff;
global const U32 bitmask26 = 0x03ffffff;
global const U32 bitmask27 = 0x07ffffff;
global const U32 bitmask28 = 0x0fffffff;
global const U32 bitmask29 = 0x1fffffff;
global const U32 bitmask30 = 0x3fffffff;
global const U32 bitmask31 = 0x7fffffff;
global const U32 bitmask32 = 0xffffffff;

global const U64 bitmask33 = 0x00000001ffffffffull;
global const U64 bitmask34 = 0x00000003ffffffffull;
global const U64 bitmask35 = 0x00000007ffffffffull;
global const U64 bitmask36 = 0x0000000fffffffffull;
global const U64 bitmask37 = 0x0000001fffffffffull;
global const U64 bitmask38 = 0x0000003fffffffffull;
global const U64 bitmask39 = 0x0000007fffffffffull;
global const U64 bitmask40 = 0x000000ffffffffffull;
global const U64 bitmask41 = 0x000001ffffffffffull;
global const U64 bitmask42 = 0x000003ffffffffffull;
global const U64 bitmask43 = 0x000007ffffffffffull;
global const U64 bitmask44 = 0x00000fffffffffffull;
global const U64 bitmask45 = 0x00001fffffffffffull;
global const U64 bitmask46 = 0x00003fffffffffffull;
global const U64 bitmask47 = 0x00007fffffffffffull;
global const U64 bitmask48 = 0x0000ffffffffffffull;
global const U64 bitmask49 = 0x0001ffffffffffffull;
global const U64 bitmask50 = 0x0003ffffffffffffull;
global const U64 bitmask51 = 0x0007ffffffffffffull;
global const U64 bitmask52 = 0x000fffffffffffffull;
global const U64 bitmask53 = 0x001fffffffffffffull;
global const U64 bitmask54 = 0x003fffffffffffffull;
global const U64 bitmask55 = 0x007fffffffffffffull;
global const U64 bitmask56 = 0x00ffffffffffffffull;
global const U64 bitmask57 = 0x01ffffffffffffffull;
global const U64 bitmask58 = 0x03ffffffffffffffull;
global const U64 bitmask59 = 0x07ffffffffffffffull;
global const U64 bitmask60 = 0x0fffffffffffffffull;
global const U64 bitmask61 = 0x1fffffffffffffffull;
global const U64 bitmask62 = 0x3fffffffffffffffull;
global const U64 bitmask63 = 0x7fffffffffffffffull;
global const U64 bitmask64 = 0xffffffffffffffffull;

global const U32 bit1  = (1<<0);
global const U32 bit2  = (1<<1);
global const U32 bit3  = (1<<2);
global const U32 bit4  = (1<<3);
global const U32 bit5  = (1<<4);
global const U32 bit6  = (1<<5);
global const U32 bit7  = (1<<6);
global const U32 bit8  = (1<<7);
global const U32 bit9  = (1<<8);
global const U32 bit10 = (1<<9);
global const U32 bit11 = (1<<10);
global const U32 bit12 = (1<<11);
global const U32 bit13 = (1<<12);
global const U32 bit14 = (1<<13);
global const U32 bit15 = (1<<14);
global const U32 bit16 = (1<<15);
global const U32 bit17 = (1<<16);
global const U32 bit18 = (1<<17);
global const U32 bit19 = (1<<18);
global const U32 bit20 = (1<<19);
global const U32 bit21 = (1<<20);
global const U32 bit22 = (1<<21);
global const U32 bit23 = (1<<22);
global const U32 bit24 = (1<<23);
global const U32 bit25 = (1<<24);
global const U32 bit26 = (1<<25);
global const U32 bit27 = (1<<26);
global const U32 bit28 = (1<<27);
global const U32 bit29 = (1<<28);
global const U32 bit30 = (1<<29);
global const U32 bit31 = (1<<30);
global const U32 bit32 = (1u<<31u);

global const U64 bit33 = (1ull<<32);
global const U64 bit34 = (1ull<<33);
global const U64 bit35 = (1ull<<34);
global const U64 bit36 = (1ull<<35);
global const U64 bit37 = (1ull<<36);
global const U64 bit38 = (1ull<<37);
global const U64 bit39 = (1ull<<38);
global const U64 bit40 = (1ull<<39);
global const U64 bit41 = (1ull<<40);
global const U64 bit42 = (1ull<<41);
global const U64 bit43 = (1ull<<42);
global const U64 bit44 = (1ull<<43);
global const U64 bit45 = (1ull<<44);
global const U64 bit46 = (1ull<<45);
global const U64 bit47 = (1ull<<46);
global const U64 bit48 = (1ull<<47);
global const U64 bit49 = (1ull<<48);
global const U64 bit50 = (1ull<<49);
global const U64 bit51 = (1ull<<50);
global const U64 bit52 = (1ull<<51);
global const U64 bit53 = (1ull<<52);
global const U64 bit54 = (1ull<<53);
global const U64 bit55 = (1ull<<54);
global const U64 bit56 = (1ull<<55);
global const U64 bit57 = (1ull<<56);
global const U64 bit58 = (1ull<<57);
global const U64 bit59 = (1ull<<58);
global const U64 bit60 = (1ull<<59);
global const U64 bit61 = (1ull<<60);
global const U64 bit62 = (1ull<<61);
global const U64 bit63 = (1ull<<62);
global const U64 bit64 = (1ull<<63);

#define radians_from_turns_f64(v) ((v)*(2*3.1415926535897))
#define turns_from_radians_f64(v) ((v)/(2*3.1415926535897))
#define degrees_from_turns_f64(v) ((v)*360.0)
#define turns_from_degrees_f64(v) ((v)/360.0)
#define degrees_from_radians_f64(v) (degrees_from_turns_f64(turns_from_radians_f64(v)))
#define radians_from_degrees_f64(v) (radians_from_turns_f64(turns_from_degrees_f64(v)))
#define tan_f64(v)    tan(radians_from_turns_f64(v))

#define C_EPS 1.11e-16
#define FP_EQUAL(s, t) (fabs((s) - (t)) <= C_EPS)

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
#define ClampTB(A, X, B) (((X) < (A)) ? (A) : ((X) > (B)) ? (B) : (X))

#define Swap(T, a, b)                                                          \
  do {                                                                         \
    T t__ = (a);                                                               \
    (a) = (b);                                                                 \
    (b) = t__;                                                                 \
  } while (0)

typedef struct coord2 {
  F64 x, y;
} Coord2;

#define Vector2FromCoord2(coord)                                               \
  (Vector2) { .x = (F32)(coord).x, .y = (F32)(coord).y }

typedef struct Triangle Triangle;
struct Triangle {
  S32 a, b, c;
};

typedef struct Slice Slice;
struct Slice {
  S32 start;
  S32 length;
};

// a range has a base and a count. Is a view into an array.
typedef struct Range Range;
struct Range {
  S32 min;
  S32 count;
};

DeclFixedArray(Coord2Array, Coord2);
DeclFixedArray(Vector2Array, Vector2);
DeclFixedArray(TriangleArray, Triangle);
DeclFixedArray(U32Array, U32);
DeclFixedArray(S32Array, S32);
DeclFixedArray(F64Array, F64);
DeclFixedArray(RangeArray, Range);
typedef struct Screen Screen;
struct Screen {
  S32 width, height;
};

static Arena arena1 = {0};
static Arena arena2 = {0};
static Arena *arenas[2] = {&arena1, &arena2};
