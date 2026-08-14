#pragma once

#include "arena.c"

#define DeclFixedArray(typename, type)                                         \
  typedef struct {                                                             \
    S32 count, capacity;                                                       \
    type *d;                                                                   \
  } typename;                                                                  \
  typedef struct type##Slice type##Slice;                                      \
  struct type##Slice {                                                         \
    type *v;                                                                   \
    S32 count;                                                                 \
  };                                                                           \
                                                                               \
  typename typename##New(Arena *arena, S32 capacity) {                         \
    type *data = (type *)arena_alloc_align(                                    \
        arena, (size_t)capacity * sizeof(type), _Alignof(type));               \
    return (typename){.count = 0, .capacity = capacity, .d = data};            \
  }                                                                            \
                                                                               \
  __attribute__((always_inline)) inline void typename##Reset(typename *stck) { \
    stck->count = 0;                                                           \
  }                                                                            \
                                                                               \
  __attribute__((always_inline)) inline S32 typename##Length(                  \
      typename *stack) {                                                       \
    return stack->count;                                                       \
  }                                                                            \
                                                                               \
  __attribute__((always_inline)) inline S32 typename##Push(typename *stack,    \
                                                           type value) {       \
    if (stack->count >= stack->capacity) {                                     \
      fprintf(stderr, "Array overflow, capacity: %d reached here: %s:%d\n",    \
              stack->capacity, __FILE__, __LINE__);                            \
      exit(EXIT_FAILURE);                                                      \
    }                                                                          \
    stack->d[stack->count++] = value;                                          \
    return stack->count - 1;                                                   \
  }                                                                            \
                                                                               \
  __attribute__((always_inline)) inline type typename##Pop(typename *stck) {   \
    if (stck->count == 0) {                                                    \
      exit(EXIT_FAILURE);                                                      \
    }                                                                          \
                                                                               \
    type value = stck->d[stck->count - 1];                                     \
    stck->count--;                                                             \
    return value;                                                              \
  }                                                                            \
                                                                               \
  __attribute__((always_inline)) inline type typename##Peek(typename *stck) {  \
    if (stck->count == 0) {                                                    \
      exit(EXIT_FAILURE);                                                      \
    }                                                                          \
    return stck->d[stck->count - 1];                                           \
  }                                                                            \
                                                                               \
  __attribute__((always_inline)) inline type##Slice type##SliceFromArray(      \
      typename *array) {                                                       \
    return (type##Slice){.v = array->d, .count = array->count};                \
  }                                                                            \
                                                                               \
  __attribute__((always_inline)) inline type##Slice type##SliceFromArrayExt(   \
      typename *array, S32 start, S32 length) {                                \
    if (!array || start + length > array->count) {                             \
      fprintf(stderr,                                                          \
              "Slice too big, requested %d + %d array has count: %d\n", start, \
              length, array->capacity);                                        \
      exit(EXIT_FAILURE);                                                      \
    }                                                                          \
    return (type##Slice){.v = array->d + start, .count = length};              \
  }                                                                            \
                                                                               \
  __attribute__((always_inline)) inline type##Slice type##SliceFromArrayStart( \
      typename *array, S32 start) {                                            \
    if (!array || start >= array->count) {                                     \
      fprintf(stderr, "Slice too big, requested [%d..] array has count: %d\n", \
              start, array->count);                                            \
      exit(EXIT_FAILURE);                                                      \
    }                                                                          \
    return (type##Slice){.v = array->d + start,                                \
                         .count = array->count - start};                       \
  }
