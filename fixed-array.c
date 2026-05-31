#pragma once

#define DeclFixedArray(typename, type)                                         \
  typedef struct {                                                             \
    S32 len, capacity;                                                         \
    type *data;                                                                \
  } typename;                                                                  \
                                                                               \
  typename typename##New(Arena *arena, S32 capacity) {                         \
    type *data = (type *)arena_alloc_align(                                    \
        arena, (size_t)capacity * sizeof(type), _Alignof(type));               \
    return (typename){.len = 0, .capacity = capacity, .data = data};           \
  }                                                                            \
                                                                               \
  void typename##Reset(typename *stck) {                                       \
    if (stck) {                                                                \
      stck->len = 0;                                                           \
    }                                                                          \
  }                                                                            \
                                                                               \
  S32 typename##Length(typename *stack) { return stack ? stack->len : 0; }     \
                                                                               \
  void typename##Push(typename *stack, type value) {                           \
    if (!stack || stack->len >= stack->capacity) {                          \
      exit(EXIT_FAILURE);                                                      \
    }                                                                          \
    stack->data[stack->len++] = value;                                         \
  }                                                                            \
                                                                               \
  type typename##Pop(typename *stck) {                                         \
    if (!stck || stck->len == 0) {                                             \
      exit(EXIT_FAILURE);                                                      \
    }                                                                          \
                                                                               \
    type value = stck->data[stck->len - 1];                                    \
    stck->len--;                                                               \
    return value;                                                              \
  }
