#pragma once

#include "../arena.c"
#include "../base.h"
#include "../string8.h"
#include "../triangulate/earcut.h"
#include "types.h"
#include "vtpk_reader.h"
#include <assert.h>
#include <raylib.h>
#include <stdbool.h>

// ------------------------  LIMITS  ------------------------
#define MVT_TEXTURE_SIZE (512 * 4)
#define MVT_MESH_SIZE (512 * 8)
#define LAYER_KEYS_MAX (1000)
#define LAYER_VALUES_MAX (1000)
#define LAYER_FEATURES_MAX (10000)

static unsigned char vector_data_buf[MB(500)] = {0};
static Arena vector_data_arena = (Arena){vector_data_buf, sizeof(vector_data_buf), 0, 0};

// TODO: figure out where to store the mesh arrays.
struct VT_VectorDataMap {
    VT_Coordinate key;
    VT_VectorData value;
};

typedef struct MVT_ProtobufData MVT_ProtobufData;
struct MVT_ProtobufData {
    U8 *v;
    U64 size;
};
DeclFixedArray(MVT_ProtobufDataArray, MVT_ProtobufData);

// implements the spec found here:
// https://github.com/mapbox/vector-tile-spec/tree/master/2.1
typedef enum GeometryType GeometryType;
enum GeometryType {
    GEOMETRY_TYPE_UNKNOWN = 0,
    GEOMETRY_TYPE_POINT = 1,
    GEOMETRY_TYPE_LINESTRING = 2,
    GEOMETRY_TYPE_POLYGON = 3,
};

typedef enum MVT_ProtobufLayerField MVT_ProtobufLayerField;
enum MVT_ProtobufLayerField {
    LAYER_FIELD_VERSION = 15,
    LAYER_FIELD_NAME = 1,
    LAYER_FIELD_FEATURES = 2,
    LAYER_FIELD_KEYS = 3,
    LAYER_FIELD_VALUES = 4,
    LAYER_FIELD_EXTENT = 5,
};

typedef enum MVT_ProtobufFeatureField MVT_ProtobufFeatureField;
enum MVT_ProtobufFeatureField {
    FEATURE_FIELD_ID = 1,
    FEATURE_FIELD_TAGS = 2,
    FEATURE_FIELD_TYPE = 3,
    FEATURE_FIELD_GEOMETRY = 4,
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
        String8 string;
        float _float;
        double _double;
        S64 int64;
        S64 sint64;
        U64 uint64;
        bool _bool;
    } value;
};
DeclFixedArray(ProtobufValueArray, ProtobufValue);

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

typedef enum MVT_PlotterCommand MVT_PlotterCommand;
enum MVT_PlotterCommand {
    MOVE_TO = 1,    // 2 parameters (dX, dY)
    LINE_TO = 2,    // 2 parameters (dX, dY)
    CLOSE_PATH = 7, // 0 parameters    -
};

typedef struct MVT_PlotterInstruction MVT_PlotterInstruction;
struct MVT_PlotterInstruction {
    U32 count;
    MVT_PlotterCommand command;
};

typedef struct MVT_Plotter MVT_Plotter;
struct MVT_Plotter {
    S32 pos_x, pos_y;
};

INTERNAL_FORCEINLINE static MVT_PlotterInstruction
InstructionFromCommandInteger(U32 command_integer) {
    return (MVT_PlotterInstruction){.command = command_integer & bitmask3,
                                    .count = command_integer >> 3};
}

INTERNAL_FORCEINLINE static S32 ParameterIntegerFromZigzagInteger(U32 zigzag_value) {
    return (S32)((zigzag_value >> 1) ^ (-(zigzag_value & 1)));
}

static U64 DecodeVarInt128(const MVT_ProtobufData data, U32 allowed_bytes_read, U64 *ip) {
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

static S64 DecodeVarInt128Signed(const MVT_ProtobufData data, U32 allowed_bytes_read, U64 *ip) {
    S64 result = 0;
    S32 shift = 0;
    U32 i = 0;
    while (i < allowed_bytes_read) {
        U8 b = data.v[i];
        result |= (b & bitmask7) << shift;
        shift += 7;
        if ((bit8 & b) == 0) {
            if (shift < 32 && (b & bit7) != 0) {
                result |= ~0 << shift;
            }
            *ip += i + 1;
            return result;
        }
        i += 1;
    }
    ERROR_MSG("invalid SignedVarInt128 detected");
}

INTERNAL_FORCEINLINE static S64 S64FromVarInt128(MVT_ProtobufData data, U64 *ip) {
    return DecodeVarInt128Signed(data, 10, ip);
}

INTERNAL_FORCEINLINE static U64 U64FromVarInt128(MVT_ProtobufData data, U64 *ip) {
    return DecodeVarInt128(data, 10, ip);
}

INTERNAL_FORCEINLINE static U32 U32FromVarInt128(MVT_ProtobufData data, U64 *ip) {
    return safe_cast_u32(DecodeVarInt128(data, 5, ip));
}

INTERNAL_FORCEINLINE static MVT_PlotterInstruction
PlotterInstructionFromProtobufData(MVT_ProtobufData data, U64 *ip) {
    const U32 command_integer = U32FromVarInt128(data, ip);
    return InstructionFromCommandInteger(command_integer);
}

typedef struct ProtobufTag ProtobufTag;
struct ProtobufTag {
    U32 field_number;
    enum { VARINT = 0, I64 = 1, LEN = 2, SGROUP = 3, EGROUP = 4, I32 = 5 } wire_type;
};

INTERNAL_FORCEINLINE static ProtobufTag TagFromProtobufData(MVT_ProtobufData data, U64 *ip) {
    const U32 raw_bytes = U32FromVarInt128(data, ip);
    return (ProtobufTag){.field_number = raw_bytes >> 3, .wire_type = raw_bytes & bitmask3};
}

INTERNAL_FORCEINLINE static String8 String8FromProtobufData(MVT_ProtobufData data, U64 *ip) {
    const U32 string_size = U32FromVarInt128(data, ip);
    const char *string_start = (const char *)data.v + *ip;
    *ip += string_size;
    return (String8){string_start, string_size};
}

INTERNAL_FORCEINLINE static F64 F64FromProtobufData(MVT_ProtobufData data, U64 *ip) {
    F64 val;
    const U8 *src = data.v + *ip;
    memcpy(&val, src, sizeof(F64));
    *ip += sizeof(F64);
    return val;
}

INTERNAL_FORCEINLINE static F32 F32FromProtobufData(MVT_ProtobufData data, U64 *ip) {
    F32 val;
    const U8 *src = data.v + *ip;
    memcpy(&val, src, sizeof(F32));
    *ip += sizeof(F32);
    return val;
}

INTERNAL_FORCEINLINE static S64 S64FromProtobufData(MVT_ProtobufData data, U64 *ip) {
    S64 val;
    const U8 *src = data.v + *ip;
    memcpy(&val, src, sizeof(S64));
    *ip += sizeof(S64);
    return val;
}

INTERNAL_FORCEINLINE static S32 S32FromProtobufData(MVT_ProtobufData data, U64 *ip) {
    S32 val;
    const U8 *src = data.v + *ip;
    memcpy(&val, src, sizeof(S32));
    *ip += sizeof(S32);
    return val;
}

INTERNAL_FORCEINLINE static U32 U32FromProtobufData(MVT_ProtobufData data, U64 *ip) {
    U32 val;
    const U8 *src = data.v + *ip;
    memcpy(&val, src, sizeof(U32));
    *ip += sizeof(U32);
    return val;
}

INTERNAL_FORCEINLINE static U64 U64FromProtobufData(MVT_ProtobufData data, U64 *ip) {
    U64 val;
    const U8 *src = data.v + *ip;
    memcpy(&val, src, sizeof(U64));
    *ip += sizeof(U64);
    return val;
}

static S32 LayerCount(const MVT_ProtobufData data) {
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
    assert(ip == data.size);
    return count;
}

static ProtobufValue ProtobufParseValue(MVT_ProtobufData data, U64 *ip) {
    ProtobufValue val = {0};
    val.type = data.v[*ip];
    switch (val.type) {
    case VALUE_STRING: {
        val.value.string = String8FromProtobufData(data, ip);
    } break;
    case VALUE_FLOAT: {
        val.value._float = F32FromProtobufData(data, ip);
    } break;
    case VALUE_DOUBLE: {
        val.value._double = F64FromProtobufData(data, ip);
    } break;
    case VALUE_INT: {
        val.value.int64 = S64FromVarInt128(data, ip);
    } break;
    case VALUE_UINT: {
        val.value.uint64 = U64FromVarInt128(data, ip);
    } break;
    case VALUE_SINT: {
        val.value.sint64 = S64FromVarInt128(data, ip);
    } break;
    case VALUE_BOOL: {
        val.value._bool = U64FromVarInt128(data, ip);
    } break;
    default:
        ERROR_MSG("invalid Protbuf value");
    }
    return val;
}

INTERNAL_FORCEINLINE static S32 ParameterFromProtobufData(const MVT_ProtobufData data, U64 *ip) {
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
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x) > 0.f ? CLOCKWISE
                                                                       : COUNTER_CLOCKWISE;
}

static void PlotterReadParametersAndCreatePoint(MVT_ProtobufData data, U64 *ip,
                                                MVT_Plotter *plotter, Coord2Array *coords) {
    plotter->pos_x += ParameterFromProtobufData(data, ip);
    plotter->pos_y += ParameterFromProtobufData(data, ip);
    Coord2ArrayPush(coords, (Coord2){.x = plotter->pos_x, .y = plotter->pos_y});
}

static void MultiPolygonFromProtobufData(VT_VectorData *tile_vector_data,
                                         const MVT_ProtobufData data, const Range geometry) {
    Temp_Arena_Memory scratch = GetScratch();
    Coord2Array coords = Coord2ArrayNew(scratch.arena, geometry.count);
    RangeArray polygons = RangeArrayNew(scratch.arena, geometry.count);

    MVT_Plotter plotter = {0};
    S32 polygon_ring_start = polygons.count;
    S32 polygon_vertex_start = coords.count;
    for (U64 ip = (U64)geometry.min; ip < ((U64)geometry.min + (U64)geometry.count);) {
        const MVT_PlotterInstruction instruction = PlotterInstructionFromProtobufData(data, &ip);
        switch (instruction.command) {
        case MOVE_TO: {
            assert(instruction.count == 1);
            PlotterReadParametersAndCreatePoint(data, &ip, &plotter, &coords);
        } break;
        case LINE_TO: {
            assert(instruction.count > 1);
            for (U32 i = 0; i < instruction.count; i += 1) {
                PlotterReadParametersAndCreatePoint(data, &ip, &plotter, &coords);
            }
        } break;
        case CLOSE_PATH: {
            // if we encounter a COUNTER_CLOCKWISE polygon and have encountered a polygon
            // already append the previous to the multipolygon list.
            assert(instruction.count == 1);
            const WindingOrder winding =
                WindingOrderFromCoords(Coord2SliceFromArrayStart(&coords, polygon_vertex_start));
            switch (winding) {
            case COUNTER_CLOCKWISE: {
                if (coords->polygons.count > polygon_ring_start) {
                    RangeArrayPush(&coords->multi_polygons,
                                   (Range){.min = polygon_ring_start,
                                           .count = coords->polygons.count - polygon_ring_start});
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
        (Range){.min = polygon_ring_start, .count = coords->polygons.count - polygon_ring_start});
    temp_arena_memory_end(scratch);
}

static void MultiLineStringFromProtobufData(VT_VectorData *tile_vector_data,
                                            const MVT_ProtobufData data, const Range geometry) {
    assert(geometry.min >= 0);
    assert(geometry.count >= 1);
    MVT_Plotter plotter = {0};
    S32 line_string_start = tile_vector_data->texture_coords.count;
    const S32 multi_line_string_start = tile_vector_data->line_strings.count;
    bool encountered_move_to = false;
    for (U64 ip = (U64)geometry.min; ip < ((U64)geometry.min + (U64)geometry.count);) {
        const MVT_PlotterInstruction instruction = PlotterInstructionFromProtobufData(data, &ip);
        switch (instruction.command) {
        case MOVE_TO: {
            if (encountered_move_to) {
                const S32 line_string_coordinate_count =
                    tile_vector_data->texture_coords.count - line_string_start;
                assert(line_string_coordinate_count > 0);
                RangeArrayPush(
                    &tile_vector_data->line_strings,
                    (Range){.min = line_string_start, .count = line_string_coordinate_count});
                line_string_start = tile_vector_data->texture_coords.count;
            }
            assert(instruction.count == 1);
            PlotterReadParametersAndCreatePoint(data, &ip, &plotter,
                                                &tile_vector_data->texture_coords);
            encountered_move_to = true;
        } break;
        case LINE_TO: {
            assert(instruction.count > 0);
            for (U32 i = 0; i < instruction.count; i += 1) {
                PlotterReadParametersAndCreatePoint(data, &ip, &plotter,
                                                    &tile_vector_data->texture_coords);
            }
        } break;
        case CLOSE_PATH: {
            ERROR_MSG("invalid command CLOSE_PATH for LINESTRING");
        } break;
        }
    }
    const S32 line_string_coordinate_count =
        tile_vector_data->texture_coords.count - line_string_start;
    assert(line_string_coordinate_count > 0);
    RangeArrayPush(&tile_vector_data->line_strings,
                   (Range){.min = line_string_start, .count = line_string_coordinate_count});
    RangeArrayPush(
        &tile_vector_data->multi_line_strings,
        (Range){.min = multi_line_string_start,
                .count = tile_vector_data->line_strings.count - multi_line_string_start});
}

static void PointFromProtobufData(VT_VectorData *tile_vector_data, const MVT_ProtobufData data,
                                  const Range geometry) {
    assert(geometry.min >= 0);
    assert(geometry.count >= 1);
    MVT_Plotter plotter = {0};
    const S32 point_start = tile_vector_data->texture_coords.count;
    for (U64 ip = (U64)geometry.min; ip < ((U64)geometry.min + (U64)geometry.count);) {
        const MVT_PlotterInstruction instruction = PlotterInstructionFromProtobufData(data, &ip);
        switch (instruction.command) {
        case MOVE_TO: {
            assert(instruction.count > 0);
            for (U32 i = 0; i < instruction.count; i += 1) {
                PlotterReadParametersAndCreatePoint(data, &ip, &plotter,
                                                    &tile_vector_data->texture_coords);
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
    RangeArrayPush(&tile_vector_data->multi_points,
                   (Range){point_start, tile_vector_data->texture_coords.count - point_start});
}

// parses Features and writes the coords into the provided layercoords for triangulation ->
// mesh/texture generation
static void FeaturesFromProtobuf(VT_VectorData *tile_vector_data, MVT_ProtobufDataSlice features,
                                 String8 layer_name, String8Slice keys, ProtobufValueSlice values,
                                 VT_Coordinate tile_coordinate, VT_StyleMap *style_map) {
    for (S32 i = 0; i < features.count; i += 1) {
        MVT_ProtobufData feature = features.v[i];

        GeometryType geometry_type = GEOMETRY_TYPE_UNKNOWN;
        S32 feature_style_index = -1;
        Range geometry_range = {0};
        U64 ip = 0;

        while (ip < feature.size) {
            const U64 saved_ip = ip;
            const ProtobufTag feature_field = TagFromProtobufData(feature, &ip);
            switch (feature_field.field_number) {
            case FEATURE_FIELD_ID: {
                // ID: skip (is optional anyway)
                assert(feature_field.wire_type == VARINT);
                const U64 id = U64FromVarInt128(feature, &ip);
                (void)id;
            } break;
            case FEATURE_FIELD_TAGS: {
                // TAGS:
                // The tags are layed out as (key: VarInt128, val: VarInt128)*
                // Look up the style for this feature by iterating over the key-value
                // pairs for this feature and take the last (?) one that fits.
                // This requires writing to the Texture/Mesh here as we have the correct style
                // info for this feature.
                assert(feature_field.wire_type == LEN);
                const U32 tags_size = U32FromVarInt128(feature, &ip);
                const U64 tags_end = tags_size + ip;
                while (ip < tags_end) {
                    const U32 tag_key_index = U32FromVarInt128(feature, &ip);
                    const U32 tag_value_index = U32FromVarInt128(feature, &ip);
                    const String8 tag_key = keys.v[tag_key_index];
                    const ProtobufValue tag_value = values.v[tag_value_index];
                    if (tag_value.type == VALUE_SINT) {
                        S64 style_index = ValueFromLayerKeyFilter(style_map, layer_name, tag_key,
                                                                  tag_value.value.sint64,
                                                                  (U32)tile_coordinate.level);
                        if (style_index != -1) {
                            feature_style_index = style_index;
                        }
                    }
                }
                ip += tags_size;
            } break;
            case FEATURE_FIELD_TYPE: {
                assert(feature_field.wire_type == VARINT);
                geometry_type = U32FromVarInt128(feature, &ip);
            } break;
            case FEATURE_FIELD_GEOMETRY: {
                assert(feature_field.wire_type == LEN);
                const U32 geometry_size = U32FromVarInt128(feature, &ip);
                geometry_range = (Range){.min = safe_cast_s32_from_u64(ip),
                                         .count = safe_cast_s32_from_u32(geometry_size)};
                ip += geometry_size;
            } break;
            default:
                ERROR_MSG("invalid field number for feature: %d", feature_field.field_number);
            }
            assert(saved_ip < ip); // ensure we are making progress
        }
        switch (geometry_type) {
            // TODO: figure out how to convert these to meshes/textures right here.
            // Maybe store the style information in a seperate array alongside the parsed geometry
            // to batch convert it once the layer has been parsed.
            // As overlapping triangles are not a problem consider drawing everything in one mesh
            // per layer.
        case GEOMETRY_TYPE_UNKNOWN: {
            ERROR_MSG("UNKNOWN geoemtry not supported")
        } break;
        case GEOMETRY_TYPE_POINT: {
            PointFromProtobufData(tile_vector_data, feature, geometry_range);
            S64ArrayPush(&tile_vector_data->mutli_point_style_indices, feature_style_index);

        } break;
        case GEOMETRY_TYPE_LINESTRING: {
            MultiLineStringFromProtobufData(tile_vector_data, feature, geometry_range);
            S64ArrayPush(&tile_vector_data->multi_line_style_indices, feature_style_index);
        } break;
        case GEOMETRY_TYPE_POLYGON: {
            MultiPolygonFromProtobufData(tile_vector_data, feature, geometry_range);
        } break;
        }
    }
    if (layer_coords->mesh_coords.count > 0 || layer_coords->texture_coords.count > 0) {
        if (layer_coords->texture_coords.count > 0) {
            LayerTextureFromCoords(layer_coords, extent);
        }
        if (layer_coords->mesh_coords.count > 0) {
            LayerMeshFromCoords(arena, &meshes, layer_coords, extent);
        }
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

static void LayerTextureFromCoords(LayerCoords *layer_coords, U32 tile_extent) {
    const F32 scale_factor = (F32)MVT_TEXTURE_SIZE / (F32)tile_extent;
    for (S32 i = 0; i < layer_coords->multi_points.count; i += 1) {
        const Range multi_point = layer_coords->multi_points.d[i];
        for (S32 j = multi_point.min; j < multi_point.count; j += 1) {
            Coord2 point = layer_coords->texture_coords.d[j];
            point.x *= scale_factor;
            point.y *= scale_factor;
            DrawCircleV(Vector2FromCoord2(point), 15.f, GREEN);
        }
    }
    for (S32 multi_line_string_index = 0;
         multi_line_string_index < layer_coords->multi_line_strings.count;
         multi_line_string_index += 1) {
        const Range multi_line_string = layer_coords->multi_line_strings.d[multi_line_string_index];
        for (S32 line_string_index = multi_line_string.min;
             line_string_index < multi_line_string.min + multi_line_string.count;
             line_string_index += 1) {
            const Range line_string = layer_coords->line_strings.d[line_string_index];
            for (S32 k = line_string.min; k < line_string.min + line_string.count - 1; k += 1) {
                Vector2 a = Vector2FromCoord2(layer_coords->texture_coords.d[k]);
                a.x *= scale_factor;
                a.y *= scale_factor;
                Vector2 b = Vector2FromCoord2(layer_coords->texture_coords.d[k + 1]);
                b.x *= scale_factor;
                b.y *= scale_factor;
                DrawLineEx(a, b, 5.f, RED);
            }
        }
    }
}

static Mesh MeshFromTriangles(Arena *arena, const TriangleArray *triangles,
                              const Coord2Slice coords, F32 scale_factor) {

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
        mesh.vertices[v++] = (float)coords.v[t.a].x * scale_factor;
        mesh.vertices[v++] = (float)coords.v[t.a].y * scale_factor + 512;
        mesh.vertices[v++] = 0.f;
        mesh.vertices[v++] = (float)coords.v[t.c].x * scale_factor;
        mesh.vertices[v++] = (float)coords.v[t.c].y * scale_factor + 512;
        mesh.vertices[v++] = 0.f;
        mesh.vertices[v++] = (float)coords.v[t.b].x * scale_factor;
        mesh.vertices[v++] = (float)coords.v[t.b].y * scale_factor + 512;
        mesh.vertices[v++] = 0.f;
    }

    assert(mesh.vertexCount > 2);
    assert(mesh.triangleCount > 0);
    UploadMesh(&mesh, false);
    return mesh;
}

// creates a mesh from the relevant coords and polygon
// stores the mesh triangle soup on the arena passed.
static void LayerMeshFromCoords(Arena *arena, Mesh meshes, LayerCoords *layer_coords,
                                U32 tile_extent) {
    Temp_Arena_Memory scratch = GetScratch();

    // this scaling is necessary as different tiles may have a different extent.
    // This ensuers the parsed coordinates are within MVT_MESH_SIZE x MVT_MESH_SIZE.
    const F32 scale_factor = (F32)MVT_MESH_SIZE / (F32)tile_extent;

    TriangleArray triangles = TriangleArrayNew(
        scratch.arena,
        layer_coords->mesh_coords.count * 2); // account for some triangulation that might
                                              // violate #triangulated triangles = #coords - 2.
    S32Array contour_sizes = S32ArrayNew(scratch.arena, layer_coords->polygons.count);
    for (S32 i = 0; i < layer_coords->multi_polygons.count; i += 1) {
        const Range polygon_contours = layer_coords->multi_polygons.d[i];
        const S32 start_index = layer_coords->polygons.d[polygon_contours.min].min;
        const S32 triangle_index_first = triangles.count;
        S32 polygon_vertex_count = 0;
        for (S32 j = polygon_contours.min; j < polygon_contours.min + polygon_contours.count;
             j += 1) {
            const S32 contour_vertex_count = layer_coords->polygons.d[j].count;
            S32ArrayPush(&contour_sizes, contour_vertex_count);
            polygon_vertex_count += contour_vertex_count;
        }
        Earcut(
            &triangles,
            Coord2SliceFromArrayExt(&layer_coords->mesh_coords, start_index, polygon_vertex_count),
            S32SliceFromArray(&contour_sizes));
        // we need to fix the indices as they are local to a polygon, but they
        // now point into the global coordinate array.
        for (S32 j = triangle_index_first; j < triangles.count; j += 1) {
            Triangle *triangle = &triangles.d[j];
            triangle->a += start_index;
            triangle->b += start_index;
            triangle->c += start_index;
            assert(triangle->a >= start_index);
            assert(triangle->a < layer_coords->mesh_coords.count);
            assert(triangle->b >= start_index);
            assert(triangle->b < layer_coords->mesh_coords.count);
            assert(triangle->c >= start_index);
            assert(triangle->c < layer_coords->mesh_coords.count);
        }
        S32ArrayReset(&contour_sizes);
    }
    MeshArrayPush(meshes, MeshFromTriangles(arena, &triangles,
                                            Coord2SliceFromArray(&layer_coords->mesh_coords),
                                            scale_factor));
    temp_arena_memory_end(scratch);
}

// parse a MVT and store it as a mesh/texture on the arena given.
// It computes the following:
// - 1 Texture
// - int(layer count in MVT) Meshes
static VectorTileGPU_Data ParseMapboxVectorTile(VT_Coordinate tile_coordinate,
                                                MVT_ProtobufData data, VT_StyleMap *style_map) {
    Temp_Arena_Memory scratch = GetScratch();
    S32 layer_count = LayerCount(data);
    DEBUG_MSG("layer count: %d\n", layer_count);

    // tile vector data, seperated in mesh (polygons) and texture, sprites/lines/circle
    Mesh layer_mesh = (Mesh){0};
    const RenderTexture2D texture = LoadRenderTexture(MVT_TEXTURE_SIZE, MVT_TEXTURE_SIZE);
    BeginTextureMode(texture);
    ClearBackground(BLANK);

    U64 ip = 0;
    S32 parsed_layers = 0;
    while (ip < data.size) {
        U64 saved_ip = ip;
        const ProtobufTag layer_tag = TagFromProtobufData(data, &ip);
        assert(layer_tag.field_number == 3);
        assert(layer_tag.wire_type == LEN);
        const U32 layer_size = U32FromVarInt128(data, &ip);
        LayerCoords *layer_coords =
            CreateNewLayer(scratch.arena, safe_cast_s32_from_u32(layer_size));
        const U64 layer_end = ip + layer_size;
        String8Array layer_keys = String8ArrayNew(scratch.arena, LAYER_KEYS_MAX);
        ProtobufValueArray layer_values = ProtobufValueArrayNew(scratch.arena, LAYER_VALUES_MAX);

        U32 extent = 4096; // default extent from spec
        MVT_ProtobufDataArray layer_features =
            MVT_ProtobufDataArrayNew(scratch.arena, LAYER_FEATURES_MAX);
        String8 layer_name = {0};

        // parse one layer
        while (ip < layer_end) {
            saved_ip = ip;
            const ProtobufTag layer_field = TagFromProtobufData(data, &ip);
            switch (layer_field.field_number) {
            case LAYER_FIELD_NAME: {
                assert(layer_field.wire_type == LEN);
                layer_name = String8FromProtobufData(data, &ip);
                assert(saved_ip < ip); // ensure we are making progress
            } break;
            case LAYER_FIELD_FEATURES: {
                assert(layer_field.wire_type == LEN);
                const U32 feature_size = U32FromVarInt128(data, &ip);
                MVT_ProtobufDataArrayPush(&layer_features,
                                          (MVT_ProtobufData){data.v + ip, feature_size});
                assert(ip + feature_size < data.size);
                assert(saved_ip < ip); // ensure we are making progress
            } break;
            case LAYER_FIELD_KEYS: {
                assert(layer_field.wire_type == LEN);
                // TODO: store the keys in a []String8 and the values in a
                // []discriminated union. Although splitting the values depending on type
                // would be easier for the CPU, indexing becomes really cumbersome.
                //
                // TODO: These arrays are stored once per layer and passed to the feature
                // parser to use. This also means parsing features has to be deferred
                // after all key-values are decoded. (E.g. not in this loop).
                const String8 key_name = String8FromProtobufData(data, &ip);
                String8ArrayPush(&layer_keys, key_name);
                assert(saved_ip < ip); // ensure we are making progress
            } break;
            case LAYER_FIELD_VALUES: {
                assert(layer_field.wire_type == LEN);
                const U32 value_size = U32FromVarInt128(data, &ip);
                assert(value_size > 0);
                ProtobufValue value = ProtobufParseValue(data, &ip);
                ProtobufValueArrayPush(&layer_values, value);
                assert(saved_ip < ip); // ensure we are making progress
            } break;
            case LAYER_FIELD_EXTENT: {
                assert(layer_field.wire_type == VARINT);
                extent = U32FromVarInt128(data, &ip);
                assert(saved_ip < ip); // ensure we are making progress
            } break;
            case LAYER_FIELD_VERSION: {
                assert(layer_field.wire_type == VARINT);
                const U32 version = U32FromVarInt128(data, &ip);
                assert(version == 2);
                assert(saved_ip < ip); // ensure we are making progress
            } break;
            default:
                ERROR_MSG("invalid field number: %d\n", layer_field.field_number);
            }
            assert(saved_ip < ip); // ensure we are making progress
        }
        // done parsing 1 layer
        assert(saved_ip < ip); // ensure we are making progress

        // compute the data we need on the GPU
        assert(layer_name.buf); // layer name is required
        FeaturesFromProtobuf(MVT_ProtobufDataSliceFromArray(&layer_features), layer_coords,
                             layer_name, String8SliceFromArray(&layer_keys),
                             ProtobufValueSliceFromArray(&layer_values), tile_coordinate,
                             style_map);
        parsed_layers += 1;
        if (parsed_layers == layer_count) {
            assert(ip == data.size);
        }
    }
    EndTextureMode();
    assert(layer_mesh.vertexCount > 0 || texture.id);
    temp_arena_memory_end(scratch);
    return (VectorTileGPU_Data){layer_mesh, texture};
}
