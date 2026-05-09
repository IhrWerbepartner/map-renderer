#pragma once

#define DeclFixedArray(typename, type)                                         \
  typedef struct {                                                             \
    size_t len, capacity;                                                      \
    type *data;                                                                \
  } typename;                                                                  \
                                                                               \
  typename typename##_new(Arena *arena, size_t capacity) {                     \
    type *data = (type *)arena_alloc_align(arena, capacity * sizeof(type),     \
                                           _Alignof(type));                    \
    return (typename){.len = 0, .capacity = capacity, .data = data};           \
  }                                                                            \
                                                                               \
  void typename##_reset(typename *stck) {                                      \
    if (stck) {                                                                \
      stck->len = 0;                                                           \
    }                                                                          \
  }                                                                            \
                                                                               \
  size_t typename##_length(typename *stack) { return stack ? stack->len : 0; } \
                                                                               \
  void typename##_push(typename *stack, type value) {                          \
    if (!stack || stack->len + 1 > stack->capacity) {                          \
      exit(EXIT_FAILURE);                                                      \
    }                                                                          \
    stack->data[stack->len++] = value;                                         \
  }                                                                            \
                                                                               \
  type typename##_pop(typename *stck) {                                        \
    if (!stck || stck->len == 0) {                                             \
      exit(EXIT_FAILURE);                                                      \
    }                                                                          \
                                                                               \
    type value = stck->data[stck->len - 1];                                    \
    stck->len--;                                                               \
    return value;                                                              \
  }
