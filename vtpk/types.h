#pragma once
#include "../base.h"
#include "../string8.h"
#include "../vendor/miniz.c"
typedef enum QuadTreeNodeType QuadTreeNodeType;
enum QuadTreeNodeType {
    EMPTY = 0, // no TileIndexRecord exists
    LEAF = 1,  // no children exist
    INNER = 2, // node contains 4 children
};

// axis-aligned bounding-box
typedef struct AABB AABB;
struct AABB {
    S32 min_x, min_y, max_x, max_y;
};

typedef struct VectorTileCoordinate VectorTileCoordinate;
struct VectorTileCoordinate {
    S32 row, col, level;
};

typedef enum VectorTileHandleStatus VectorTileHandleStatus;
enum VectorTileHandleStatus {
    DATA_INVALID = 0, // model data was never queried and therefore is not valid
    DATA_PRESENT = 1, // model data is present
    DATA_EVICTED = 2, // model data was present but was evicted from the cache
                      // and is now invalid
};

DeclFixedArray(MeshArray, Mesh);
DeclFixedArray(RenderTexture2DArray, RenderTexture2D);
// SOA layout for vector tile data.
// Name can determine if this layer should be drawn and how
typedef struct VectorTileGPU_Data VectorTileGPU_Data;
struct VectorTileGPU_Data {
    MeshSlice meshes;
    RenderTexture2DSlice textures;
    String8Slice layer_names;
};

// handle into cache for triangualted tiles
typedef struct VectorTileHandle VectorTileHandle;
struct VectorTileHandle {
    VectorTileCoordinate coordinate;
    VectorTileGPU_Data gpu_data;
    S32 quad_tree_node;
    VectorTileHandleStatus status;
};
DeclFixedArray(VectorTileHandleArray, VectorTileHandle);

typedef struct QuadTreeNode QuadTreeNode;
struct QuadTreeNode {
    QuadTreeNodeType type;
    S32 child_nw;
    S32 child_ne;
    S32 child_sw;
    S32 child_se;
    VectorTileHandle tile;
};
DeclFixedArray(QuadTreeNodeArray, QuadTreeNode);

// ----------- .bundle file ------------
typedef struct TileIndexRecord TileIndexRecord;
struct TileIndexRecord {
    U64 tile_offset;
    U32 tile_size;
};

TileIndexRecord TileIndexRecordFromIndex(U64 index) {
    const U8 gzip_header_size = 10;
    return (TileIndexRecord){
        .tile_offset = (index & bitmask40) + gzip_header_size,
        .tile_size = safe_cast_u32(index >> 40) - gzip_header_size,
    };
}
typedef struct TileBundleFileHeader TileBundleFileHeader;
struct TileBundleFileHeader {
    S32 version; // has to be 3
    S32 record_count;
    S32 max_tile_size;
    S32 offset_byte_count;
    S64 slack_space;
    S64 file_size;
    S64 user_header_offset;
    S32 user_header_size;
    S32 legacy1;
    S32 legacy2;
    S32 legacy3;
    S32 legacy4;
    S32 index_size;           // has to be 131072 (128 * 128 * 8)
    U64 tile_index[128][128]; // bits 0..39 represent offset, 40..63 the size
                              // (bytes) of a tile
};

typedef struct VtpkFileRootProperties VtpkFileRootProperties;
struct VtpkFileRootProperties {
    F64Array lod_resolutions;
    F64 tile_info_origin_x, tile_info_origin_y;
    U32 lod_min, lod_max;
    U32 tile_info_cols, tile_info_rows;
};

// describes a lookup for a style held in FilterValueNode.paint
// Nodes are ordered hirachically SourceLayerNode -> FilterKeyNode -> FilterValueNode.
// Where Key/Value nodes can have siblings representing the multiple children of the node
// in the hirachy above.

typedef enum VT_SourceLayerType VT_SourceLayerType;
enum VT_SourceLayerType { FILL, LINE, SYMBOL, CIRCLE };
typedef struct VT_SourceLayer VT_SourceLayer;
struct VT_SourceLayer {
    VT_SourceLayerType type;
    String8 source_layer;
    S32 key_first;
    S32 key_last;
};

typedef struct VT_SourceLayerMap VT_SourceLayerMap;
struct VT_SourceLayerMap {
    char *key;
    VT_SourceLayer value;
};

typedef enum VT_LayerStyleNodeType VT_LayerStyleNodeType;
enum VT_LayerStyleNodeType { FILTER_KEY = 1, FILTER_VALUE };

typedef struct VT_LayerStyleNode VT_LayerStyleNode;
struct VT_LayerStyleNode {
    VT_LayerStyleNodeType type;
    union {
        struct FilterKeyNode {
            String8 key;
            S32 next;
            S32 value_first;
            S32 value_last;
        } filter_key;
        struct FilterValueNode {
            S64 value;
            union {
                Color fill_color;
                Color line_color;
                Color circle_color;
                String8 icon_image; // referes to the sprite name
            } paint;
            S32 next;
        } filter_val;
    };
};
DeclFixedArray(VT_LayerStyleNodeArray, VT_LayerStyleNode);

typedef struct VtpkFile VtpkFile;
struct VtpkFile {
    mz_zip_archive *archive;
    VtpkFileRootProperties root_propreties;
    VT_SourceLayerMap *layer_styles;
    VT_LayerStyleNodeArray layer_filters;
    AABB bounding_box;
    QuadTreeNodeArray quad_tree;
    S32 root_node;
};

// TODO: switch the map to be. this is way better
// store a map<keys, values> this stores as many layers as in the styles file, not too
// bad(!).
// if a filter is not present hash("") is computed as the key hash. and 0 is written into
// the value.
//
// keys:
// struct {
//     string_hash_128_bits layer;
//     string_hash_128_bits key;
//     U64 value;
// };
// values:
// struct {
//     union {
//         struct fill_info;
//         struct line_info;
//         struct symbol_info;
//         struct circle_info;
//     };
// };
//
