#pragma once

#define internal static
#define global static

#if defined(_MSC_VER)
#define INTERNAL_FORCEINLINE __forceinline inline
#elif defined(__clang__) || defined(__GNUC__)
#define INTERNAL_FORCEINLINE __attribute__((always_inline)) inline
#else
#error need force inline for this compiler
#endif
