GEO_FILE=./samples/poly_test_2.json
EXE_NAME= map-renderer
EXE_NAME_UNOPTIMIZED= unoptimized-map-renderer
CFLAGS= -Wextra -Wall -Wundef -Wno-unused-function -Wshadow -Wpointer-arith -Wcast-align -Wstrict-prototypes -Wstrict-overflow=5 -Wwrite-strings -Wcast-qual -Wswitch-enum -Werror=switch -Wconversion -DRAYMATH_USE_SIMD_INTRINSICS -march=native

SOURCE_FILES= map_renderer.c arena.c base.h triangulate/earcut.h json_parser.h string8.h vtpk/vtpk.h vtpk/vtpk_reader.h

LDFLAGS= -lraylib -lm

.PHONY: default
default: map-renderer

run: $(EXE_NAME)
	./$(EXE_NAME) $(GEO_FILE)

windows: $(SOURCE_FILES)
	zig cc -o render.exe map_renderer.c -I"C:\raylib\w64devkit\include" "C:\raylib\w64devkit\lib\libraylib.a" -lopengl32 -lgdi32 -lwinmm -g

map-renderer: $(SOURCE_FILES)
	gcc $(CFLAGS) $(executable) $(LDFLAGS) -g map_renderer.c -o $(EXE_NAME) -O2

unoptimized: $(SOURCE_FILES)
	gcc -DDEBUG $(CFLAGS) $(LDFLAGS) -g map_renderer.c -o $(EXE_NAME_UNOPTIMIZED)

debug: unoptimized
	gf2 $(EXE_NAME_UNOPTIMIZED)


stat: $(EXE_NAME)
	perf stat -d ./$(EXE_NAME) $(GEO_FILE)

perf-record: $(EXE_NAME)
	perf record -g ./$(EXE_NAME) $(GEO_FILE)

clean:
	rm $(EXE_NAME)
