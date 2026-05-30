GEO_FILE=./samples/poly_austria.json
EXE_NAME= map-renderer
EXE_NAME_UNOPTIMIZED= unoptimized-map-renderer
CFLAGS= -Wextra -Wall -Wundef -Wshadow -Wpointer-arith -Wcast-align -Wstrict-prototypes -Wstrict-overflow=5 -Wwrite-strings -Wcast-qual -Wswitch-default -Wswitch-enum -Wconversion 

LDFLAGS= -lraylib -lm

.PHONY: default
default: map-renderer

run: $(EXE_NAME)
	./$(EXE_NAME) $(GEO_FILE)

map-renderer: geojson-parser.c triangulate.h arena.c base.h
	gcc -DDEBUG $(CFLAGS) $(executable) $(LDFLAGS) -g geojson-parser.c -o $(EXE_NAME) -O3

unoptimized: geojson-parser.c triangulate.h arena.c base.h
	gcc -DDEBUG $(CFLAGS) $(LDFLAGS) -g geojson-parser.c -o $(EXE_NAME_UNOPTIMIZED)

debug: unoptimized
	gf2 $(EXE_NAME_UNOPTIMIZED)


stat: $(EXE_NAME)
	perf stat -d ./$(EXE_NAME) $(GEO_FILE)

perf-record: $(EXE_NAME)
	perf record -g ./$(EXE_NAME) $(GEO_FILE)

