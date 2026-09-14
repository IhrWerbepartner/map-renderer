GEO_FILE=./samples/bmapv_vtpk_3857.vtpk
EXE_NAME= map-renderer
EXE_NAME_UNOPTIMIZED= unoptimized-map-renderer
CFLAGS= -Wextra -Wall -Wundef -Wno-unused-function -Wshadow -Wpointer-arith -Wcast-align -Wstrict-prototypes -Wstrict-overflow=5 -Wwrite-strings -Wcast-qual -Wswitch-enum -Werror=switch -Wconversion -DRAYMATH_USE_SIMD_INTRINSICS -march=native

SOURCE_FILES= map_renderer.c arena.c base.h triangulate/earcut.h json_parser.h string8.h vtpk/vtpk.h vtpk/vtpk_reader.h vtpk/mvt.h
WINDOWS_INCLUDES="C:\raylib\w64devkit\include" "C:\raylib\w64devkit\lib\libraylib.a"
WINDOWS_LDFLAGS=-lopengl32 -lgdi32 -lwinmm
WINDOWS_DEFINES=-DWIN32_LEAN_AND_MEAN -DNOMINMAX -DNOGDI -DNOUSER

LDFLAGS= -lraylib -lm

.PHONY: default
default: release

run: $(EXE_NAME)
	./$(EXE_NAME) $(GEO_FILE)

windows: $(SOURCE_FILES)
	zig cc -o render_debug.exe map_renderer.c -I $(WINDOWS_INCLUDES) $(WINDOWS_LDFLAGS) $(WINDOWS_DEFINES) -g -DDEBUG

windows-optimized: $(SOURCE_FILES)
	zig cc -o render_release.exe map_renderer.c -I $(WINDOWS_INCLUDES) $(WINDOWS_LDFLAGS) $(WINDOWS_DEFINES) -g -O2

release: $(SOURCE_FILES)
	gcc $(CFLAGS) $(executable) $(LDFLAGS) -g -finstrument-functions map_renderer.c -o $(EXE_NAME) -O2

unoptimized: $(SOURCE_FILES)
	gcc -DDEBUG $(CFLAGS) $(LDFLAGS) -g -finstrument-functions map_renderer.c -o $(EXE_NAME_UNOPTIMIZED)

release-stripped: $(SOURCE_FILES)
	gcc $(CFLAGS) $(executable) $(LDFLAGS) map_renderer.c -o map-renderer-release-stripped -O2

stat: $(EXE_NAME)
	perf stat -d ./$(EXE_NAME) $(GEO_FILE)

clean:
	rm $(EXE_NAME)
