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

typedef struct VT_Coordinate VT_Coordinate;
struct VT_Coordinate {
    S32 row, col, level;
};

typedef enum VectorTileHandleStatus VectorTileHandleStatus;
enum VectorTileHandleStatus {
    DATA_INVALID = 0, // model data was never queried and therefore is not valid
    DATA_PRESENT = 1, // model data is present
    DATA_EVICTED = 2, // model data was present but was evicted from the cache
                      // and is now invalid
};

typedef struct VectorTileGPU_Data VectorTileGPU_Data;
struct VectorTileGPU_Data {
    Mesh mesh;
    RenderTexture2D texture;
};

// handle into cache for triangualted tiles
typedef struct VectorTileHandle VectorTileHandle;
struct VectorTileHandle {
    VT_Coordinate coordinate;
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

typedef struct VT_StyleMapKey VT_StyleMapKey;
struct VT_StyleMapKey {
    U64 layer_hash;
    U64 filter_key_hash;
    S64 filter_value;
    U32 zoom;
};

typedef enum VT_StylePaintType VT_StylePaintType;
enum VT_StylePaintType { FILL, LINE, SYMBOL, CIRCLE };

typedef struct VT_StyleMapValue VT_StyleMapValue;
struct VT_StyleMapValue {
    VT_StylePaintType type;
    union {
        Color fill_color;
        Color line_color;
        Color circle_color;
        String8 icon_image; // referes to the sprite name
    } paint;
};

// maps VT_StyleMapKeys -> VT_StyleMapValues. Stored once persistent (!) per VTPK Bundle.
typedef struct VT_StyleMap VT_StyleMap;
struct VT_StyleMap {
    VT_StyleMapKey key;
    VT_StyleMapValue value;
};

typedef struct VtpkFile VtpkFile;
struct VtpkFile {
    mz_zip_archive *archive;
    VtpkFileRootProperties root_propreties;
    VT_StyleMap *layer_styles;
    AABB bounding_box;
    QuadTreeNodeArray quad_tree;
    S32 root_node;
};

typedef struct VT_VectorData VT_VectorData;
struct VT_VectorData {
    S32Array vertex_soup;
    S32Array triangle_colors; // RGBA
    S32 vertex_count;
    S32 triangle_count;

    Coord2Array texture_coords;    // holds the coordinates for every feature that
                                   // gets transformed into a GPU texture.
    RangeArray line_strings;       // a slice into coords for every line-string.
    RangeArray multi_line_strings; // a slice into coords for every MULTI line-string. This is
                                   // equivalent to the number of LINESTRINGs in the Protobuf Data.
    S64Array multi_line_style_indices;

    RangeArray multi_points; // an index into coord for every (multi)-point.
    S64Array mutli_point_style_indices;
};
