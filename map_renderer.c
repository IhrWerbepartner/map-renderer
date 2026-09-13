#include "arena.c"
#include "base.h"
#include "geojson.h"
#include "string8.h"
#include "vtpk/vtpk.h"
#include <assert.h>
#include <raylib.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

#ifdef _WIN32
#include <memoryapi.h>
#else
#include <sys/mman.h>
#endif

void usage(char *program_name) { printf("usage: %s <filepath>\n", program_name); }

int main(int argc, char **argv) {
    if (argc != 2) {
        usage(argv[0]);
        exit(EXIT_FAILURE);
    }
    const U64 backing_buffer_size = GB(2);
#ifdef _WIN32
    void *backing_buffer =
        VirtualAlloc(NULL, backing_buffer_size, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
#else
    void *backing_buffer = mmap(NULL, backing_buffer_size, PROT_READ | PROT_WRITE,
                                MAP_PRIVATE | MAP_ANONYMOUS | MAP_HUGE_2GB, -1, 0);
    if (backing_buffer == MAP_FAILED) {
        ERROR_MSG("unable to map memory, errno: %d", errno);
    }
#endif
    arena_init(arenas[0], backing_buffer, backing_buffer_size);

#ifdef _WIN32
    backing_buffer =
        VirtualAlloc(NULL, backing_buffer_size, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
#else
    backing_buffer = mmap(NULL, backing_buffer_size, PROT_READ | PROT_WRITE,
                          MAP_PRIVATE | MAP_ANONYMOUS | MAP_HUGE_2GB, -1, 0);
    if (backing_buffer == MAP_FAILED) {
        ERROR_MSG("unable to map memory");
    }
#endif

    arena_init(arenas[1], backing_buffer, backing_buffer_size);
#ifdef INTERNAL_ENABLE_PROFILE
    spall_auto_init((char *)"profile.spall");
    int thread_id = 0;
    spall_auto_thread_init(thread_id, SPALL_DEFAULT_BUFFER_SIZE);
#endif

    //--------------------------------------------------------------------------------------
    // Initialization
    //--------------------------------------------------------------------------------------
    const Screen screen = {.width = 2560, .height = 1440};
    SetTraceLogLevel(LOG_INFO);
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    const String8 map_file = String8FromCString(argv[1]);

    if (String8EndsWith(map_file, String8FromCString(".geojson")) ||
        String8EndsWith(map_file, String8FromCString(".json"))) {
        InitWindow(screen.width, screen.height, "Map Renderer");
        SetTargetFPS(60);
        GeoJsonDisplayFile(argv[1], screen);
    } else if (String8EndsWith(map_file, String8FromCString(".vtpk"))) {
        InitWindow(screen.width, screen.height, "Map Renderer");
        SetTargetFPS(60);
        VtpkDisplayFile(argv[1], screen);
    } else {
        ERROR_MSG("unknown file format: %s\n", argv[1]);
    }
    // De-Initialization
    //--------------------------------------------------------------------------------------
    CloseWindow(); // Close window and OpenGL context
    //--------------------------------------------------------------------------------------
#ifdef INTERNAL_ENABLE_PROFILE
    spall_auto_thread_quit();
    spall_auto_quit();
#endif
    return 0;
}
