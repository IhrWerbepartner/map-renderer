#include "arena.c"
#include "base.h"
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

#ifdef _WIN32
#include <memoryapi.h>
#else
#include <sys/mman.h>
#endif


void usage(char *program_name) {
  printf("usage: %s <filepath>\n", program_name);
}

int main(int argc, char **argv) {
  if (argc != 2) {
    usage(argv[0]);
    exit(EXIT_FAILURE);
  }
  const U64 backing_buffer_size = GB(2);
#ifdef _WIN32
  void *backing_buffer = VirtualAlloc(NULL, backing_buffer_size,
                                      MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
#else
  void *backing_buffer = mmap(NULL, backing_buffer_size, PROT_READ | PROT_WRITE,
                              MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
#endif
  arena_init(arenas[0], backing_buffer, backing_buffer_size);

#ifdef _WIN32
  backing_buffer = VirtualAlloc(NULL, backing_buffer_size,
                                MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
#else
  backing_buffer = mmap(NULL, backing_buffer_size, PROT_READ | PROT_WRITE,
                        MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
#endif

  arena_init(arenas[1], backing_buffer, backing_buffer_size);

  //--------------------------------------------------------------------------------------
  // Initialization
  //--------------------------------------------------------------------------------------
  const Screen screen = {.width = 2560, .height = 1440};

  InitWindow(screen.width, screen.height, "Map Renderer");
  SetTargetFPS(60);
  if (String8EndsWith(argv[1], ".geojson")) {
    GeoJsonDisplayFile(argv[1], screen);
  } else if (String8EndsWith(argv[1], ".vtpk")) {
    VtpkDisplayFile(argv[1], screen);
  } else {
    ERROR_MSG("unknown file format: %s\n", argv[1]);
  }
  // De-Initialization
  //--------------------------------------------------------------------------------------
  CloseWindow(); // Close window and OpenGL context
  //--------------------------------------------------------------------------------------
  return 0;
}
