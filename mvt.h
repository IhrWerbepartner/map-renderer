#pragma once

#include "arena.c"
#include "base.h"
#include "string8.h"
#include "vtpk_reader.h"
#include <assert.h>
#include <raylib.h>
#include <stdbool.h>
#include <sys/types.h>

typedef struct MapboxVectorTileProtobufData MapboxVectorTileProtobufData;
struct MapboxVectorTileProtobufData {
  const U8 *v;
  U64 size;
};

// implements the spec found here:
// https://github.com/mapbox/vector-tile-spec/tree/master/2.1
enum GeometryType {
  UNKOWN = 0,
  POINT = 1,
  LINESTRING = 2,
  POLYGON = 3,
};

enum ValueType {
  VALUE_STRING = 1,
  VALUE_FLOAT = 2,
  VALUE_DOUBLE = 3,
  VALUE_INT = 4,
  VALUE_UINT = 5,
  VALUE_SINT = 6,
  VALUE_BOOL = 7,
};

/*
struct Value {
  ValueType type;
  union {
    String8 string;
    F32 _float;
    F64 _double;
    S32 _int;
    S32 sint;
    S32 uint;
    bool _bool;
  } value;
};

struct Feature {
  U64 id;
  U32Slice tags;
  GeometryType type;
  U32Slice geometry;
};

struct Layer {
  String8 name;
  FeatureSlice features;
  U32Slice keys;
  U32Slice values;
};

struct LayerTags {
  String8Slice keys;
  ValueSlice values;
};
*/

typedef struct TileCoords TileCoords;
struct TileCoords {
  Coord2Array coords;        // holds the coordinates for every feature.
  RangeArray polygons;       // holds range for every polygon (NOT FEATURE!).
  RangeArray multi_polygons; // holds range for every (multi)-polygon
                             // FEATURE! Indexes into polygon_coords.
  RangeArray line_string_coords; // a slice into coords for every line-string.
  S32Array point_coords;         // an index into coord for every point.
};

typedef enum PlotterCommand PlotterCommand;
enum PlotterCommand {
  MOVE_TO = 1,    // 2 parameters (dX, dY)
  LINE_TO = 2,    // 2 parameters (dX, dY)
  CLOSE_PATH = 7, // 0 parameters    -
};

typedef struct PlotterInstruction PlotterInstruction;
struct PlotterInstruction {
  U32 count;
  PlotterCommand command;
};

typedef struct MapboxVectorTilePlotter MapboxVectorTilePlotter;
struct MapboxVectorTilePlotter {
  S32 pos_x, pos_y;
  U64 ip;
};

static PlotterInstruction InstructionFromCommandInteger(U32 command_integer) {
  return (PlotterInstruction){.command = command_integer & bitmask3,
                              .count = command_integer >> 3};
}

static S32 ParameterIntegerFromZigzagInteger(U32 zigzag_value) {
  return safe_cast_s32_from_u32((zigzag_value >> 1) ^ (-(zigzag_value & 1)));
}

static U64 DecodeVarInt128(MapboxVectorTileProtobufData data,
                           S32 allowed_bytes_read, U64 *ip) {
  U64 result = 0;
  S32 shift = 0;
  for (S32 i = 0; i < allowed_bytes_read; i += 1, shift += 7) {
    result |= (U64)(data.v[i] & bitmask7) << shift;
    if (!((data.v[i] & bit8) == bit8)) {
      *ip += i;
      return result;
    }
  }
  ERROR_MSG("invalid VarInt128 detected");
}

static U64 U64FromVarInt128(MapboxVectorTileProtobufData data, U64 *ip) {
  return DecodeVarInt128(data, 10, ip);
}

static U32 U32FromVarInt128(MapboxVectorTileProtobufData data, U64 *ip) {
  return safe_cast_u32(DecodeVarInt128(data, 5, ip));
}

typedef struct ProtobufTag ProtobufTag;
struct ProtobufTag {
  U32 field_number;
  enum {
    VARINT = 0,
    I64 = 1,
    LEN = 2,
    SGROUP = 3,
    EGROUP = 4,
    I32 = 5
  } wire_type;
};

static ProtobufTag TagFromProtobufData(MapboxVectorTileProtobufData data,
                                       U64 *ip) {
  U32 raw_bytes = U32FromVarInt128(data, ip);
  return (ProtobufTag){.field_number = raw_bytes >> 3,
                       .wire_type = raw_bytes & bitmask3};
}

static String8 String8FromProtobufData(MapboxVectorTileProtobufData data,
                                       U64 *ip) {
  ProtobufTag tag = TagFromProtobufData(data, ip);
  assert(tag.wire_type == LEN);
  const U32 string_size = U32FromVarInt128(data, ip);
  return (String8){(const char *)data.v, string_size};
}

static F64 F64FromProtobufData(MapboxVectorTileProtobufData data, U64 *ip) {
  F64 val = ((F64 *)data.v)[*ip];
  *ip += 8;
  return val;
}

static F32 F32FromProtobufData(MapboxVectorTileProtobufData data, U64 *ip) {
  F32 val = ((F32 *)data.v)[*ip];
  *ip += 4;
  return val;
}

static S64 S64FromProtobufData(MapboxVectorTileProtobufData data, U64 *ip) {
  S64 val = ((S64 *)data.v)[*ip];
  *ip += 8;
  return val;
}

static S32 S32FromProtobufData(MapboxVectorTileProtobufData data, U64 *ip) {
  S32 val = ((S32 *)data.v)[*ip];
  *ip += 4;
  return val;
}

static S32 LayerCount(const MapboxVectorTileProtobufData data) {
  U64 ip = 0;
  S32 count = 0;
  while (ip < data.size) {
    const ProtobufTag tag = TagFromProtobufData(data, &ip);
    assert(tag.wire_type == LEN);
    assert(tag.field_number == 3);
    const U32 layer_length = U32FromVarInt128(data, &ip);
    ip += layer_length;
    count += 1;
  }
  return count;
}

static void ProtobufParseFeature() {
  makeNewLayer();
  resetPlotter();
  while (plotter.ip < data.size) {
    switch (data.v[plotter.ip]) {
    case UNKOWN:
      break;
    case POINT:
      AddPoint();
      break;
    case LINESTRING:
      ParseLineString();
      break;
    case POLYGON:
      ParsePolygon();
      break;
    }
  }
}

// parse a MVT and store it as a model/mesh on the arena given.
static VectorTileGPU_Data
ParseMapboxVectorTile(Arena *arena, MapboxVectorTileProtobufData data) {
  Temp_Arena_Memory scratch = GetScratchConflict(&arena, 1);
  MapboxVectorTilePlotter plotter = {0};
  TileCoords tile_coords = {0}; // TODO initialzie with correct size
  S32 layer_count = LayerCount(data);
  MeshArray meshes = MeshArrayNew(arena, layer_count);
  RenderTexture2DArray textures = RenderTexture2DArrayNew(arena, layer_count);
  String8Array names = String8ArrayNew(arena, layer_count);
  while (plotter.ip < data.size) {
    const ProtobufTag layer_tag = TagFromProtobufData(data, &plotter.ip);
    assert(layer_tag.field_number == 3);
    assert(layer_tag.wire_type == LEN);
    const U32 layer_length = U32FromVarInt128(data, &plotter.ip);

    const ProtobufTag layer_field = TagFromProtobufData(data, &plotter.ip);
    switch (layer_field.field_number) {
    case 1: {
      const String8 layer_name = String8FromProtobufData(data, &plotter.ip);
    } break;
    case 2: {
      ProtobufParseFeature(data, &plotter.ip);
    } break;
    case 3: {
      const String8 key_name = String8FromProtobufData(data, &plotter.ip);
    } break;
    case 4: {
      ProtobufParseValue(data, &plotter.ip);
    } break;
    case 5: {
      const U32 extent = U32FromVarInt128(data, &plotter.ip);
      assert(extent == 4096);
    } break;
    case 15: {
      const U32 version = U32FromVarInt128(data, &plotter.ip);
      assert(version == 2);
    } break;
    default:
      ERROR_MSG("invalid field number: %d\n", tag.field_number);
    }
  }
  temp_arena_memory_end(scratch);
}
