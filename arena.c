#pragma once

#include "base_defines.h"
#include <assert.h>
#include <stdalign.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static INTERNAL_FORCEINLINE bool is_power_of_two(intptr_t x) { return (x & (x - 1)) == 0; }

static INTERNAL_FORCEINLINE intptr_t align_forward(intptr_t ptr, S64 align) {
    assert(is_power_of_two(align));

    intptr_t p = ptr;
    const intptr_t a = (intptr_t)align;
    // Same as (p % a) but faster as 'a' is a power of two
    const intptr_t modulo = p & (a - 1);

    if (modulo != 0) {
        // If 'p' address is not aligned, push the address to the
        // next value which is aligned
        p += a - modulo;
    }
    return p;
}

#ifndef DEFAULT_ALIGNMENT
#define DEFAULT_ALIGNMENT (2 * (S64)sizeof(void *))
#endif

typedef struct Arena Arena;
struct Arena {
    unsigned char *buf;
    S64 buf_len;
    S64 prev_offset; // This will be useful for later on
    S64 curr_offset;
};

static INTERNAL_FORCEINLINE void arena_init(Arena *a, void *backing_buffer,
                                            S64 backing_buffer_length) {
    a->buf = (unsigned char *)backing_buffer;
    a->buf_len = backing_buffer_length;
    a->curr_offset = 0;
    a->prev_offset = 0;
}

static INTERNAL_FORCEINLINE void *arena_alloc_align(Arena *a, S64 size, S64 align) {
    assert(size >= 0);
    // Align 'curr_offset' forward to the specified alignment
    intptr_t curr_ptr = (intptr_t)a->buf + (intptr_t)a->curr_offset;
    intptr_t offset = align_forward(curr_ptr, align);
    offset -= (intptr_t)a->buf; // Change to relative offset

    // Check to see if the backing memory has space left
    if (offset + size <= a->buf_len) {
        void *ptr = &a->buf[offset];
        a->prev_offset = offset;
        a->curr_offset = offset + size;

        // Zero new memory by default
        memset(ptr, 0, (size_t)size);
        return ptr;
    }
    // Return NULL if the arena is out of memory (or handle differently)
    assert(0 && "Backing memory is exhausted");
    return NULL;
}

// Because C doesn't have default parameters
static INTERNAL_FORCEINLINE void *arena_alloc(Arena *a, S64 size) {
    return arena_alloc_align(a, size, DEFAULT_ALIGNMENT);
}

#define arena_alloc_array_aligned(a, T, c, align)                                                  \
    (T *)arena_alloc_align((a), (S64)sizeof(T) * (c), (align))
#define arena_alloc_array(a, T, c) arena_alloc_array_aligned(a, T, c, Max(8, (S64)alignof(T)))

static INTERNAL_FORCEINLINE void *arena_resize_align(Arena *a, void *old_memory, S64 old_size,
                                                     S64 new_size, S64 align) {
    assert(new_size >= 0);
    unsigned char *old_mem = (unsigned char *)old_memory;

    assert(is_power_of_two(align));

    if (old_mem == NULL || old_size == 0) {
        return arena_alloc_align(a, new_size, align);
    } else if (a->buf <= old_mem && old_mem < a->buf + a->buf_len) {
        if (a->buf + a->prev_offset == old_mem) {
            a->curr_offset = a->prev_offset + new_size;
            if (new_size > old_size) {
                // Zero the new memory by default
                memset(&a->buf[a->curr_offset], 0, (size_t)(new_size - old_size));
            }
            return old_memory;
        } else {
            void *new_memory = arena_alloc_align(a, new_size, align);
            S64 copy_size = old_size < new_size ? old_size : new_size;
            assert(copy_size >= 0);
            // Copy across old memory to the new memory
            memmove(new_memory, old_memory, (size_t)copy_size);
            return new_memory;
        }

    } else {
        assert(0 && "Memory is out of bounds of the buffer in this arena");
        return NULL;
    }
}

// Because C doesn't have default parameters
static INTERNAL_FORCEINLINE void *arena_resize(Arena *a, void *old_memory, S64 old_size,
                                               S64 new_size) {
    return arena_resize_align(a, old_memory, old_size, new_size, DEFAULT_ALIGNMENT);
}

static INTERNAL_FORCEINLINE void arena_free_all(Arena *a) {
    a->curr_offset = 0;
    a->prev_offset = 0;
}

// Extra Features
typedef struct Temp_Arena_Memory Temp_Arena_Memory;
struct Temp_Arena_Memory {
    Arena *arena;
    S64 prev_offset;
    S64 curr_offset;
};

static INTERNAL_FORCEINLINE Temp_Arena_Memory temp_arena_memory_begin(Arena *a) {
    Temp_Arena_Memory temp;
    temp.arena = a;
    temp.prev_offset = a->prev_offset;
    temp.curr_offset = a->curr_offset;
    return temp;
}

static INTERNAL_FORCEINLINE void temp_arena_memory_end(Temp_Arena_Memory temp) {
    temp.arena->prev_offset = temp.prev_offset;
    temp.arena->curr_offset = temp.curr_offset;
}

static INTERNAL_FORCEINLINE Temp_Arena_Memory scratch_get_free(Arena **arenaPool, int arenaPoolSize,
                                                               Arena **conflictingArenas,
                                                               int conflictingArenaSize) {
    for (int i = 0; i < arenaPoolSize; i++) {
        bool conflict = false;
        for (int z = 0; z < conflictingArenaSize; z++) {
            if (arenaPool[i] == conflictingArenas[z]) {
                conflict = true;
            }
        }
        if (!conflict) {
            return temp_arena_memory_begin(arenaPool[i]);
        }
    }
    // unreachable
    exit(EXIT_FAILURE);
}

#define GetScratch() scratch_get_free(arenas, 2, 0, 0)

#define GetScratchConflict(conflictingArenas, num)                                                 \
    scratch_get_free(arenas, 2, conflictingArenas, num)

#define ScratchArena()                                                                             \
    for (                                                                                          \
        struct {                                                                                   \
            Temp_Arena_Memory scratch;                                                             \
            S32 latch;                                                                             \
        } arena_auto_close_latch = {GetScratch(), 0};                                              \
        arena_auto_close_latch.latch < 1;                                                          \
        arena_auto_close_latch.latch = 1, temp_arena_memory_end(arena_auto_close_latch.scratch))
