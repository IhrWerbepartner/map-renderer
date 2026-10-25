#pragma once

#include <stdint.h>
#define internal static
#define global static

#if defined(_MSC_VER)
#define INTERNAL_FORCEINLINE __forceinline inline
#elif defined(__clang__) || defined(__GNUC__)
#define INTERNAL_FORCEINLINE __attribute__((always_inline)) inline
#else
#error need force inline for this compiler
#endif

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
typedef struct U128 U128;
struct U128 {
    U64 u64[2];
};
