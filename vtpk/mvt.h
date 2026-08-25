#pragma once

#include "../arena.c"
#include "../base.h"
#include "../fixed-array.c"
#include "../string8.h"
#include "../triangulate/earcut.h"
#include <assert.h>
#include <raylib.h>
#include <stdbool.h>

typedef struct MapboxVectorTileProtobufData MapboxVectorTileProtobufData;
struct MapboxVectorTileProtobufData {
    U8 *v;
    U64 size;
};

DeclFixedArray(RenderTexture2DArray, RenderTexture2D);

// implements the spec found here:
// https://github.com/mapbox/vector-tile-spec/tree/master/2.1
typedef enum GeometryType GeometryType;
enum GeometryType {
    UNKOWN = 0,
    POINT = 1,
    LINESTRING = 2,
    POLYGON = 3,
};

typedef enum WindingOrder WindingOrder;
enum WindingOrder {
    CLOCKWISE,
    COUNTER_CLOCKWISE,
};

typedef enum ProtobufValueType ProtobufValueType;
enum ProtobufValueType {
    VALUE_STRING = 1,
    VALUE_FLOAT = 2,
    VALUE_DOUBLE = 3,
    VALUE_INT = 4,
    VALUE_UINT = 5,
    VALUE_SINT = 6,
    VALUE_BOOL = 7,
};

typedef struct ProtobufValue ProtobufValue;
// TODO expand this list to all supported types
struct ProtobufValue {
    ProtobufValueType type;
    union {
        double _double;
        float _float;
        S32 int32;
        S64 int64;
        U32 uint32;
        U64 uint64;
        bool _bool;
        String8 string;
    } value;
};

typedef struct LayerCoords LayerCoords;
struct LayerCoords {
    Coord2Array mesh_coords;       // holds the coordinates for every feature that gets
                                   // transformed into a GPU mesh.
    Coord2Array texture_coords;    // holds the coordinates for every feature that
                                   // gets transformed into a GPU texture.
    RangeArray polygons;           // holds range for every polygon (NOT FEATURE!).
    RangeArray multi_polygons;     // holds range for every (multi)-polygon
                                   // FEATURE! Indexes into polygon_coords.
    RangeArray line_strings;       // a slice into coords for every line-string.
    RangeArray multi_line_strings; // a slice into coords for every MULTI line-string.
                                   // if this only has one entry for a line string it is
                                   // a single line string. otherwise a multi line string
    RangeArray multi_points;       // an index into coord for every (multi)-point.
};

DeclFixedArray(MeshArray, Mesh);
//
// SOA layout for vector tile data.
// Name can determine if this layer should be drawn and how
typedef struct VectorTileGPU_Data VectorTileGPU_Data;
struct VectorTileGPU_Data {
    MeshSlice meshes;
    RenderTexture2DSlice textures;
    String8Slice layer_names;
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
};

static PlotterInstruction InstructionFromCommandInteger(U32 command_integer) {
    return (PlotterInstruction){.command = command_integer & bitmask3,
                                .count = command_integer >> 3};
}

static S32 ParameterIntegerFromZigzagInteger(U32 zigzag_value) {
    return (S32)((zigzag_value >> 1) ^ (-(zigzag_value & 1)));
}

static U64 DecodeVarInt128(const MapboxVectorTileProtobufData data, U32 allowed_bytes_read,
                           U64 *ip) {
    U64 result = 0;
    S32 shift = 0;
    for (U32 i = 0; i < allowed_bytes_read; i += 1, shift += 7) {
        result |= (U64)(data.v[i + *ip] & bitmask7) << shift;
        if ((data.v[i + *ip] & bit8) != bit8) {
            *ip += i + 1;
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

static PlotterInstruction PlotterInstructionFromProtobufData(MapboxVectorTileProtobufData data, U64 *ip) {
    const U32 command_integer = U32FromVarInt128(data, ip);
    return InstructionFromCommandInteger(command_integer);
}

typedef struct ProtobufTag ProtobufTag;
struct ProtobufTag {
    U32 field_number;
    enum { VARINT = 0, I64 = 1, LEN = 2, SGROUP = 3, EGROUP = 4, I32 = 5 } wire_type;
};

static ProtobufTag TagFromProtobufData(MapboxVectorTileProtobufData data, U64 *ip) {
    const U32 raw_bytes = U32FromVarInt128(data, ip);
    return (ProtobufTag){.field_number = raw_bytes >> 3,
                         .wire_type = raw_bytes & bitmask3};
}

static String8 String8FromProtobufData(MapboxVectorTileProtobufData data, U64 *ip) {
    const U32 string_size = U32FromVarInt128(data, ip);
    const char *string_start = (const char *)data.v + *ip;
    *ip += string_size;
    return (String8){string_start, string_size};
}

static F64 F64FromProtobufData(MapboxVectorTileProtobufData data, U64 *ip) {
    F64 val;
    const U8 *src = data.v + *ip;
    memcpy(&val, src, sizeof(F64));
    *ip += sizeof(F64);
    return val;
}

static F32 F32FromProtobufData(MapboxVectorTileProtobufData data, U64 *ip) {
    F32 val;
    const U8 *src = data.v + *ip;
    memcpy(&val, src, sizeof(F32));
    *ip += sizeof(F32);
    return val;
}

static S64 S64FromProtobufData(MapboxVectorTileProtobufData data, U64 *ip) {
    S64 val;
    const U8 *src = data.v + *ip;
    memcpy(&val, src, sizeof(S64));
    *ip += sizeof(S64);
    return val;
}

static S32 S32FromProtobufData(MapboxVectorTileProtobufData data, U64 *ip) {
    S32 val;
    const U8 *src = data.v + *ip;
    memcpy(&val, src, sizeof(S32));
    *ip += sizeof(S32);
    return val;
}

static U32 U32FromProtobufData(MapboxVectorTileProtobufData data, U64 *ip) {
    U32 val;
    const U8 *src = data.v + *ip;
    memcpy(&val, src, sizeof(U32));
    *ip += sizeof(U32);
    return val;
}

static U64 U64FromProtobufData(MapboxVectorTileProtobufData data, U64 *ip) {
    U64 val;
    const U8 *src = data.v + *ip;
    memcpy(&val, src, sizeof(U64));
    *ip += sizeof(U64);
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
        assert(layer_length > 0);
        count += 1;
    }
    return count;
}

static ProtobufValue ProtobufParseValue(MapboxVectorTileProtobufData data, U32 value_size,
                                        U64 *ip) {
    (void)data;
    *ip += value_size;
    // TODO: implement
    return (ProtobufValue){0};
}

static S32 ParameterFromProtobufData(const MapboxVectorTileProtobufData data, U64 *ip) {
    return ParameterIntegerFromZigzagInteger(U32FromVarInt128(data, ip));
}

static WindingOrder WindingOrderFromCoords(Coord2Slice coords) {
    Coord2 coord_min = (Coord2){.x = min_F64, .y = max_F64};
    S32 index_min = 0;
    for (S32 i = 0; i < coords.count; i += 1) {
        Coord2 c = coords.v[i];
        if (c.y < coord_min.y || (c.y == coord_min.y && c.x > coord_min.x)) {
            index_min = i;
            coord_min = c;
        }
    }
    const Coord2 a = coords.v[index_min];
    const Coord2 b = coords.v[(index_min + coords.count - 1) % coords.count];
    const Coord2 c = coords.v[(index_min + 1) % coords.count];
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x) > 0.f
               ? CLOCKWISE
               : COUNTER_CLOCKWISE;
}

static void ProtobufParsePolygon(const MapboxVectorTileProtobufData data,
                                 LayerCoords *coords, const Range geometry) {
    MapboxVectorTilePlotter plotter = {0};
    S32 polygon_ring_start = coords->polygons.count;
    S32 polygon_vertex_start = coords->mesh_coords.count;
    for (U64 ip = (U64)geometry.min; ip < (U64)(geometry.min + geometry.count);) {
        const PlotterInstruction instruction = PlotterInstructionFromProtobufData(data, &ip);
        switch (instruction.command) {
        case MOVE_TO: {
            assert(instruction.count == 1);
            plotter.pos_x += ParameterFromProtobufData(data, &ip);
            plotter.pos_y += ParameterFromProtobufData(data, &ip);
            Coord2ArrayPush(&coords->mesh_coords,
                            (Coord2){.x = plotter.pos_x, .y = plotter.pos_y});
        } break;
        case LINE_TO: {
            assert(instruction.count > 1);
            for (U32 i = 0; i < instruction.count; i += 1) {
                plotter.pos_x += ParameterFromProtobufData(data, &ip);
                plotter.pos_y += ParameterFromProtobufData(data, &ip);
                Coord2ArrayPush(&coords->mesh_coords,
                                (Coord2){.x = plotter.pos_x, .y = plotter.pos_y});
            }
        } break;
        case CLOSE_PATH: {
            // if we encounter a COUNTER_CLOCKWISE polygon and have encountered a polygon already
            // append the previous to the multipolygon list.
            assert(instruction.count == 1);
            const WindingOrder winding = WindingOrderFromCoords(
                Coord2SliceFromArrayStart(&coords->mesh_coords, polygon_vertex_start));
            switch (winding) {
            case COUNTER_CLOCKWISE: {
                if (coords->polygons.count > polygon_ring_start) {
                    RangeArrayPush(&coords->multi_polygons,
                                   (Range){polygon_ring_start,
                                           coords->polygons.count - polygon_ring_start});
                    polygon_ring_start = coords->polygons.count;
                }
            } break;
            case CLOCKWISE:
                break;
            }
            RangeArrayPush(
                &coords->polygons,
                (Range){polygon_vertex_start, coords->mesh_coords.count - polygon_vertex_start});
            polygon_vertex_start = coords->mesh_coords.count;
        } break;
        }
    }
    RangeArrayPush(
        &coords->multi_polygons,
        (Range){polygon_ring_start, coords->polygons.count - polygon_ring_start});
}

static void ProtobufParseLineString(const MapboxVectorTileProtobufData data,
                                    LayerCoords *coords, const Range geometry) {
    assert(geometry.min >= 0);
    assert(geometry.count >= 1);
    MapboxVectorTilePlotter plotter = {0};
    S32 line_string_start = coords->texture_coords.count;
    const S32 multi_line_string_start = coords->line_strings.count;
    for (U64 ip = (U64)geometry.min; ip < (U64)(geometry.min + geometry.count);) {
        const PlotterInstruction instruction = PlotterInstructionFromProtobufData(data, &ip);
        switch (instruction.command) {
        case MOVE_TO: {
            assert(instruction.count == 1);
            plotter.pos_x += ParameterFromProtobufData(data, &ip);
            plotter.pos_y += ParameterFromProtobufData(data, &ip);
            Coord2ArrayPush(&coords->texture_coords,
                            (Coord2){.x = plotter.pos_x, .y = plotter.pos_y});
            if (coords->line_strings.count > multi_line_string_start) {
                RangeArrayPush(&coords->line_strings,
                               (Range){line_string_start,
                                       coords->texture_coords.count - line_string_start});
                line_string_start = coords->texture_coords.count;
            }
        } break;
        case LINE_TO: {
            assert(instruction.count > 0);
            for (U32 i = 0; i < instruction.count; i += 1) {
                plotter.pos_x += ParameterFromProtobufData(data, &ip);
                plotter.pos_y += ParameterFromProtobufData(data, &ip);
                Coord2ArrayPush(&coords->texture_coords,
                                (Coord2){.x = plotter.pos_x, .y = plotter.pos_y});
            }
        } break;
        case CLOSE_PATH: {
            ERROR_MSG("invalid command CLOSE_PATH for LINESTRING");
        } break;
        }
    }
    RangeArrayPush(&coords->multi_line_strings,
                   (Range){multi_line_string_start,
                           coords->line_strings.count - multi_line_string_start});
}

static void ProtobufParsePoint(const MapboxVectorTileProtobufData data,
                               LayerCoords *coords, const Range geometry) {
    assert(geometry.min >= 0);
    assert(geometry.count >= 1);
    MapboxVectorTilePlotter plotter = {0};
    const S32 point_start = coords->texture_coords.count;
    for (U64 ip = (U64)geometry.min; ip < (U64)(geometry.min + geometry.count);) {
        const PlotterInstruction instruction = PlotterInstructionFromProtobufData(data, &ip);
        switch (instruction.command) {
        case MOVE_TO: {
            assert(instruction.count > 0);
            for (U32 i = 0; i < instruction.count; i += 1) {
                plotter.pos_x += ParameterFromProtobufData(data, &ip);
                plotter.pos_y += ParameterFromProtobufData(data, &ip);
                Coord2ArrayPush(&coords->texture_coords,
                                (Coord2){.x = plotter.pos_x, .y = plotter.pos_y});
            }
        } break;
        case LINE_TO: {
            ERROR_MSG("invalid command LINE_TO for POINT");
        } break;
        case CLOSE_PATH: {
            ERROR_MSG("invalid command CLOSE_PATH for POINT");
        } break;
        }
    }
    RangeArrayPush(&coords->multi_points,
                   (Range){point_start, coords->texture_coords.count - point_start});
}
// parses a Feature and writes the coords into the provided layercoords for
// triangulation -> mesh/texture generation
static void ProtobufParseFeature(MapboxVectorTileProtobufData data, LayerCoords *coords,
                                 U32 feature_size, U64 *ip) {
    U64 feature_end = *ip + feature_size;
    GeometryType geometry_type = UNKOWN;
    Range geometry_range = {0};
    while (*ip < feature_end) {
        const U64 saved_ip = *ip;
        const ProtobufTag feature_field = TagFromProtobufData(data, ip);
        switch (feature_field.field_number) {
        case 1: {
            // ID: skip for now (is optional anyway
            assert(feature_field.wire_type == VARINT);
            const U64 id = U64FromVarInt128(data, ip);
           } break;
        case 2: {
            // TAGS: TODO skip for now
            assert(feature_field.wire_type == LEN);
            const U32 tags_size = U32FromVarInt128(data, ip);
            *ip += tags_size;
            }break;
        case 3:  {
            assert(feature_field.wire_type == VARINT);
            geometry_type = U32FromVarInt128(data, ip);
            }break;
        case 4:{
            assert(feature_field.wire_type == LEN);
            const U32 geometry_size = U32FromVarInt128(data, ip);
            geometry_range = (Range){.min = safe_cast_s32_from_u64(*ip),
                                     .count = safe_cast_s32_from_u32(geometry_size)};
            *ip += geometry_size;
            }break;
        default:
            ERROR_MSG("invalid field number for feature: %d", feature_field.field_number);
        }
        assert(saved_ip < *ip); // ensure we are making progress
    }
    switch (geometry_type) {
    case UNKOWN:
        ERROR_MSG("UNKNOWN geoemtry not supported")
    case POINT:
        ProtobufParsePoint(data, coords, geometry_range);
        break;
    case LINESTRING:
        ProtobufParseLineString(data, coords, geometry_range);
        break;
    case POLYGON:
        ProtobufParsePolygon(data, coords, geometry_range);
        break;
    }
}

// creates a new layer with very conservative size estimates
static LayerCoords *CreateNewLayer(Arena *arena, S32 layer_size) {
    LayerCoords *layer = arena_alloc(arena, sizeof(LayerCoords));
    layer->mesh_coords = Coord2ArrayNew(arena, layer_size);
    layer->texture_coords = Coord2ArrayNew(arena, layer_size);
    layer->line_strings = RangeArrayNew(arena, layer_size);
    layer->multi_line_strings = RangeArrayNew(arena, layer_size);
    layer->polygons = RangeArrayNew(arena, layer_size);
    layer->multi_polygons = RangeArrayNew(arena, layer_size);
    layer->multi_points = RangeArrayNew(arena, layer_size);
    return layer;
}

static void LayerTextureFromCoords(RenderTexture2DArray *textures,
                                   LayerCoords *layer_coords) {
    const S32 tile_extent = 512; // TODO: parse this from tile layer??
    RenderTexture2D texture = LoadRenderTexture(tile_extent, tile_extent);
    BeginTextureMode(texture);
    {
        for (S32 i = 0; i < layer_coords->multi_points.count; i += 1) {
            Range multi_point = layer_coords->multi_points.d[i];
            for (S32 j = multi_point.min; j < multi_point.count; j += 1) {
                Coord2 point = layer_coords->texture_coords.d[j];
                point.x /= 8.0;
                point.y /= 8.0;
                DrawCircleV(Vector2FromCoord2(point), 10.f, RED);
            }
        }
        for (S32 i = 0; i < layer_coords->multi_line_strings.count; i += 1) {
            Range multi_line_string = layer_coords->multi_line_strings.d[i];
            for (S32 j = multi_line_string.min; j < multi_line_string.count; j += 1) {
                Range line_string = layer_coords->line_strings.d[i];
                for (S32 k = line_string.min; k < line_string.count - 1; k += 1) {
                    Vector2 a = Vector2FromCoord2(layer_coords->texture_coords.d[k]);
                    a.x /= 8.f;
                    a.y /= 8.f;
                    Vector2 b = Vector2FromCoord2(layer_coords->texture_coords.d[k + 1]);
                    b.x /= 8.f;
                    b.y /= 8.f;
                    DrawLineEx(a, b, 10.f, RED);
                }
            }
        }
    }
    EndTextureMode();
    RenderTexture2DArrayPush(textures, texture);
}
static Mesh MeshFromTriangles(Arena *arena, const TriangleArray *triangles,
                              const Coord2Slice coords) {

    // create a big mesh (triangle soup) to avoid unsigned short limit for
    // indicdes in openGL
    S32 vertex_soup_count = triangles->count * 3;
    Mesh mesh = {
        .vertexCount = vertex_soup_count,
        .vertices = arena_alloc_array(arena, float, (size_t)vertex_soup_count * 3),
        .triangleCount = triangles->count,
    };
    S32 v = 0;
    for (S32 i = 0; i < triangles->count; i++) {
        Triangle t = triangles->d[i];
        mesh.vertices[v++] = (float)coords.v[t.a].x;
        mesh.vertices[v++] = -(float)coords.v[t.a].y;
        mesh.vertices[v++] = 0.f;
        mesh.vertices[v++] = (float)coords.v[t.b].x;
        mesh.vertices[v++] = -(float)coords.v[t.b].y;
        mesh.vertices[v++] = 0.f;
        mesh.vertices[v++] = (float)coords.v[t.c].x;
        mesh.vertices[v++] = -(float)coords.v[t.c].y;
        mesh.vertices[v++] = 0.f;
    }
    UploadMesh(&mesh, false);
    return mesh;
}

// creates a mesh from the relevant coords and polygon
// stores the mesh triangle soup on the arena passed.
static void LayerMeshFromCoords(Arena *arena, MeshArray *meshes,
                                LayerCoords *layer_coords) {
    // const S32 tile_extent = 512; // TODO: parse this from tile layer??
    Temp_Arena_Memory scratch = GetScratch();
    TriangleArray triangles =
        TriangleArrayNew(scratch.arena, layer_coords->mesh_coords.count);
    S32Array contour_sizes = S32ArrayNew(scratch.arena, layer_coords->polygons.count);
    for (S32 i = 0; i < layer_coords->multi_polygons.count; i += 1) {
        const Range polygon_contours = layer_coords->multi_polygons.d[i];
        const S32 start_index = layer_coords->polygons.d[polygon_contours.min].min;
        const S32 triangle_index_first = triangles.count;
        S32 polygon_vertex_count = 0;
        for (S32 j = polygon_contours.min;
             j < polygon_contours.min + polygon_contours.count; j += 1) {
            const S32 contour_vertex_count = layer_coords->polygons.d[j].count;
            S32ArrayPush(&contour_sizes, contour_vertex_count);
            polygon_vertex_count += contour_vertex_count;
        }
        Earcut(&triangles,
               Coord2SliceFromArrayExt(&layer_coords->mesh_coords, start_index,
                                       polygon_vertex_count),
               S32SliceFromArray(&contour_sizes));
        // we need to fix the indices as they are local to a polygon, but they
        // now point into the global coordinate array.
        for (S32 j = triangle_index_first; j < triangles.count; j += 1) {
            Triangle *triangle = &triangles.d[j];
            triangle->a += start_index;
            triangle->b += start_index;
            triangle->c += start_index;
            assert(triangle->a > start_index);
            assert(triangle->a < layer_coords->mesh_coords.count);
            assert(triangle->b > start_index);
            assert(triangle->b < layer_coords->mesh_coords.count);
            assert(triangle->c > start_index);
            assert(triangle->c < layer_coords->mesh_coords.count);
        }
        S32ArrayReset(&contour_sizes);
    }
    MeshArrayPush(meshes,
                  MeshFromTriangles(arena, &triangles,
                                    Coord2SliceFromArray(&layer_coords->mesh_coords)));
    temp_arena_memory_end(scratch);
}

static void ComputeGpuData(Arena *arena, MeshArray *meshes,
                           RenderTexture2DArray *textures, String8Array *names,
                           LayerCoords *layer_coords) {
    LayerTextureFromCoords(textures, layer_coords);
    LayerMeshFromCoords(arena, meshes, layer_coords);
}

// parse a MVT and store it as a mesh/texture on the arena given.
static VectorTileGPU_Data ParseMapboxVectorTile(Arena *arena,
                                                MapboxVectorTileProtobufData data) {
    Temp_Arena_Memory scratch = GetScratchConflict(&arena, 1);
    S32 layer_count = LayerCount(data);
    DEBUG_MSG("layer count: %d\n", layer_count);
    MeshArray meshes = MeshArrayNew(arena, layer_count);
    RenderTexture2DArray textures = RenderTexture2DArrayNew(arena, layer_count);
    String8Array names = String8ArrayNew(arena, layer_count);
    U64 ip = 0;
    while (ip < data.size) {
        U64 saved_ip = ip;
        const ProtobufTag layer_tag = TagFromProtobufData(data, &ip);
        assert(layer_tag.field_number == 3);
        assert(layer_tag.wire_type == LEN);
        const U32 layer_size = U32FromVarInt128(data, &ip);
        LayerCoords *layer_coords =
            CreateNewLayer(scratch.arena, safe_cast_s32_from_u32(layer_size));
        const U64 layer_end = ip + layer_size;
        while (ip < layer_end) {
            saved_ip = ip;
            const ProtobufTag layer_field = TagFromProtobufData(data, &ip);
            switch (layer_field.field_number) {
            case 1: {
                assert(layer_field.wire_type == LEN);
                const String8 layer_name = String8FromProtobufData(data, &ip);
                assert(saved_ip < ip); // ensure we are making progress
            } break;
            case 2: {
                assert(layer_field.wire_type == LEN);
                const U32 feature_size = U32FromVarInt128(data, &ip);
                ProtobufParseFeature(data, layer_coords, feature_size, &ip);
                assert(saved_ip < ip); // ensure we are making progress
            } break;
            case 3: {
                const String8 key_name = String8FromProtobufData(data, &ip);
                assert(saved_ip < ip); // ensure we are making progress
            } break;
            case 4: {
                assert(layer_field.wire_type == LEN);
                const U32 value_size = U32FromVarInt128(data, &ip);
                assert(value_size > 0);
                ProtobufParseValue(data, value_size, &ip);
                assert(saved_ip < ip); // ensure we are making progress
            } break;
            case 5: {
                const U32 extent = U32FromVarInt128(data, &ip);
                assert(extent == 4096);
                assert(saved_ip < ip); // ensure we are making progress
            } break;
            case 15: {
                const U32 version = U32FromVarInt128(data, &ip);
                assert(version == 2);
                assert(saved_ip < ip); // ensure we are making progress
            } break;
            default:
                ERROR_MSG("invalid field number: %d\n", layer_field.field_number);
            }
            assert(saved_ip < ip); // ensure we are making progress
        }
        assert(saved_ip < ip); // ensure we are making progress
        if (layer_coords->mesh_coords.count > 0 ||
            layer_coords->texture_coords.count > 0) {
            assert(layer_coords->mesh_coords.count > 0 ||
                   layer_coords->texture_coords.count > 0);
            ComputeGpuData(arena, &meshes, &textures, &names, layer_coords);
        }
    }
    assert(meshes.count > 0 || textures.count > 0);
    temp_arena_memory_end(scratch);
    return (VectorTileGPU_Data){MeshSliceFromArray(&meshes),
                                RenderTexture2DSliceFromArray(&textures),
                                String8SliceFromArray(&names)};
}
