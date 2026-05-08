CFLAGS= -Wextra -Wall -Wfloat-equal -Wundef -Wshadow -Wpointer-arith -Wcast-align -Wstrict-prototypes -Wstrict-overflow=5 -Wwrite-strings -Wcast-qual -Wswitch-default -Wswitch-enum -Wconversion

LDFLAGS= -lraylib -lm

.PHONY: default
default: map-renderer

map-renderer: geojson-parser.c
	gcc $(CFLAGS) $(LDFLAGS) -g geojson-parser.c -o map-renderer
