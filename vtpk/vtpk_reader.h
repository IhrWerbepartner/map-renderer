#pragma once

#include "../arena.c"
#include "../base.h"
#include "../fixed-array.c"
#include "../json_parser.h"
#include "../string8.h"
#include "../vendor/miniz.c"
#include "mvt.h"
#include <assert.h>
#include <raylib.h>
#include <stdbool.h>
#include <stdio.h>

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

typedef struct VtpkFile VtpkFile;
struct VtpkFile {
    mz_zip_archive *archive;
    VtpkFileRootProperties root_propreties;
    AABB bounding_box;
    QuadTreeNodeArray quad_tree;
    S32 root_node;
};

static unsigned char gpu_data_arena_level0to7_buf[MB(100)] = {0};
static unsigned char gpu_data_arena_level8to12_buf[MB(100)] = {0};
static unsigned char gpu_data_arena_level13to15_buf[MB(100)] = {0};
static unsigned char gpu_data_arena_level16_buf[MB(100)] = {0};

static Arena gpu_data_arena_level0to7 =
    (Arena){gpu_data_arena_level0to7_buf, sizeof(gpu_data_arena_level0to7_buf), 0, 0};
static Arena gpu_data_arena_level8to12 =
    (Arena){gpu_data_arena_level8to12_buf, sizeof(gpu_data_arena_level8to12_buf), 0, 0};
static Arena gpu_data_arena_level13to15 =
    (Arena){gpu_data_arena_level13to15_buf, sizeof(gpu_data_arena_level13to15_buf), 0, 0};
static Arena gpu_data_arena_level16 =
    (Arena){gpu_data_arena_level16_buf, sizeof(gpu_data_arena_level16_buf), 0, 0};

static void OpenZipArchive(mz_zip_archive *archive, const char *filepath) {
    assert(archive->m_last_error == MZ_ZIP_NO_ERROR);
    if (!(mz_zip_reader_init_file(archive, filepath, 0))) {
        // ERROR_MSG("can not open zip archive\n");
        ERROR_MSG("can not open zip archive: '%s'\n",
                  mz_zip_get_error_string(archive->m_last_error));
    }
    assert(archive->m_last_error == MZ_ZIP_NO_ERROR);
}

static U64 UncompressedFileSize(mz_zip_archive *archive, U32 file_index) {
    mz_zip_archive_file_stat stat = {0};
    if (!mz_zip_reader_file_stat(archive, file_index, &stat)) {
        ERROR_MSG("can not read stats for file '%d'", file_index);
    }
    return stat.m_uncomp_size;
}

static S32 QuadTreeNodeFromJson(QuadTreeNodeArray *quad_tree, const JsonNode *node, S32 x,
                                S32 y, S32 level) {
    assert(node != NULL);
    switch (node->type) {
    case JSON_ARRAY: {
        ASSERT(node->children.count == 4, "invalid list of children");
        const S32 child_nw = QuadTreeNodeFromJson(quad_tree, node->children.first, x * 2,
                                                  y * 2, level + 1);
        const S32 child_ne = QuadTreeNodeFromJson(quad_tree, node->children.first->next,
                                                  x * 2, (y * 2) + 1, level + 1);
        const S32 child_sw = QuadTreeNodeFromJson(quad_tree, node->children.last->prev,
                                                  (x * 2) + 1, y * 2, level + 1);
        const S32 child_se = QuadTreeNodeFromJson(quad_tree, node->children.last,
                                                  (x * 2) + 1, (y * 2) + 1, level + 1);
        const S32 node_index =
            QuadTreeNodeArrayPush(quad_tree, (QuadTreeNode){.type = INNER,
                                                            .child_nw = child_nw,
                                                            .child_ne = child_ne,
                                                            .child_sw = child_sw,
                                                            .child_se = child_se});
        quad_tree->d[node_index].tile =
            (VectorTileHandle){.quad_tree_node = node_index,
                               .coordinate = (VectorTileCoordinate){x, y, level}};
        return node_index;
    }
    case JSON_INTEGER: {
        U64 val = node->num.u_value;
        ASSERT(val == 0 || val == 1, "invalid node value");
        if (val == 0) {
            return 0;
        }
        const S32 node_index =
            QuadTreeNodeArrayPush(quad_tree, (QuadTreeNode){.type = LEAF});
        quad_tree->d[node_index].tile =
            (VectorTileHandle){.quad_tree_node = node_index,
                               .coordinate = (VectorTileCoordinate){x, y, level}};
        return node_index;
    }
    case JSON_NULL:
    case JSON_BOOL:
    case JSON_OBJECT:
    case JSON_DOUBLE:
    case JSON_STRING:
    default:
        ERROR_MSG("invalid json node type");
    }
}

static U32 FileIndexFromFileName(mz_zip_archive *archive, const char *filename) {
    assert(archive != NULL);
    assert(filename != NULL);
    S32 file_index = mz_zip_reader_locate_file(archive, filename, NULL, 0);
    if (file_index < 0) {
        ERROR_MSG("file '%s' not found in archive", filename);
    }
    assert(file_index >= 0);
    return (U32)file_index;
}

static void QuadTreeFromJson(Arena *arena, VtpkFile *vtpk_file) {
    const char *tilemap = "p12/tilemap/root.json";
    const U32 file_index = FileIndexFromFileName(vtpk_file->archive, tilemap);
    const U64 file_size = UncompressedFileSize(vtpk_file->archive, file_index);

    // zero (sentinel value) first index
    vtpk_file->quad_tree =
        QuadTreeNodeArrayNew(arena, safe_cast_s32_from_u64(file_size) / 2);
    QuadTreeNodeArrayPush(&vtpk_file->quad_tree, (QuadTreeNode){0});

    Temp_Arena_Memory json_scratch = temp_arena_memory_begin(arena);
    char *file_content = arena_alloc(json_scratch.arena, file_size + 1);
    mz_zip_reader_extract_to_mem(vtpk_file->archive, file_index, file_content,
                                 file_size + 1, 0);
    assert(vtpk_file->archive->m_last_error == MZ_ZIP_NO_ERROR);

    JsonNode root = {0};
    JsonParseValue(json_scratch.arena, &root, (String8){0}, file_content);
    assert(root.type == JSON_NULL);
    // get root object (second node in parsed json)
    root = *root.children.first;
    assert(root.type == JSON_OBJECT);
    const JsonNode *tree_root = find_key_in_children(&root, String8FromCString("index"));
    vtpk_file->root_node =
        QuadTreeNodeFromJson(&vtpk_file->quad_tree, tree_root, 0, 0, 0);

    temp_arena_memory_end(json_scratch);
}

static void RootPropertiesFromJson(Arena *arena, VtpkFile *vtpk_file) {
    Temp_Arena_Memory json_scratch = GetScratchConflict(&arena, 1);
    const char *root_properties = "p12/root.json";
    U32 file_index = FileIndexFromFileName(vtpk_file->archive, root_properties);
    U64 file_size = UncompressedFileSize(vtpk_file->archive, file_index);
    char *file_content = arena_alloc(json_scratch.arena, file_size + 1);
    mz_zip_reader_extract_to_mem(vtpk_file->archive, file_index, file_content,
                                 file_size + 1, 0);

    JsonNode root = {0};
    JsonParseValue(arena, &root, (String8){0}, file_content);
    assert(root.type == JSON_NULL);
    // get root object (second node in parsed json)
    root = *root.children.first;
    assert(root.type == JSON_OBJECT);
    const JsonNode *tile_info =
        find_key_in_children(&root, String8FromCString("tileInfo"));
    {
        const JsonNode *tile_info_rows =
            find_key_in_children(tile_info, String8FromCString("rows"));
        const JsonNode *tile_info_cols =
            find_key_in_children(tile_info, String8FromCString("cols"));
        assert(tile_info_rows->type == JSON_INTEGER);
        assert(tile_info_cols->type == JSON_INTEGER);
        vtpk_file->root_propreties.tile_info_rows =
            safe_cast_u32(tile_info_rows->num.u_value);
        vtpk_file->root_propreties.tile_info_cols =
            safe_cast_u32(tile_info_cols->num.u_value);
    }
    {
        const JsonNode *origin =
            find_key_in_children(tile_info, String8FromCString("origin"));
        assert(origin->type == JSON_OBJECT);
        const JsonNode *origin_x = find_key_in_children(origin, String8FromCString("x"));
        const JsonNode *origin_y = find_key_in_children(origin, String8FromCString("y"));
        assert(origin_x->type == JSON_DOUBLE);
        assert(origin_y->type == JSON_DOUBLE);
        vtpk_file->root_propreties.tile_info_origin_x = origin_x->num.dbl_value;
        vtpk_file->root_propreties.tile_info_origin_y = origin_y->num.dbl_value;
    }
    {
        const JsonNode *lods =
            find_key_in_children(tile_info, String8FromCString("lods"));
        vtpk_file->root_propreties.lod_resolutions =
            F64ArrayNew(arena, lods->children.count);
        for (JsonNode *child = lods->children.first; child != lods->children.last;
             child = child->next) {
            assert(child->type == JSON_OBJECT);
            const JsonNode *resolution =
                find_key_in_children(child, String8FromCString("resolution"));
            assert(resolution->type == JSON_DOUBLE);
            F64ArrayPush(&vtpk_file->root_propreties.lod_resolutions,
                         resolution->num.dbl_value);
        }
    }
    {
        const JsonNode *min_LOD =
            find_key_in_children(&root, String8FromCString("minLOD"));
        const JsonNode *max_LOD =
            find_key_in_children(&root, String8FromCString("minLOD"));
        assert(min_LOD->type == JSON_INTEGER);
        assert(max_LOD->type == JSON_INTEGER);
        vtpk_file->root_propreties.lod_min = safe_cast_u32(min_LOD->num.u_value);
        vtpk_file->root_propreties.lod_max = safe_cast_u32(max_LOD->num.u_value);
    }
    temp_arena_memory_end(json_scratch);
    // TODO: figure out wich properties we actually need.
}

VtpkFile *VtpkParseFile(Arena *arena, const char *filepath) {
    VtpkFile *vtpk_file = arena_alloc(arena, sizeof(VtpkFile));
    vtpk_file->archive = arena_alloc(arena, sizeof(mz_zip_archive));
    mz_zip_zero_struct(vtpk_file->archive);
    OpenZipArchive(vtpk_file->archive, filepath);
    assert(vtpk_file->archive != NULL);
    {
        // TILEMAP
        QuadTreeFromJson(arena, vtpk_file);
    }
    {
        // ROOT PROPERTIES
        RootPropertiesFromJson(arena, vtpk_file);
    }
    return vtpk_file;
}

// returns true if the bounding box contains or touches the coordinate
static bool AABBContains(AABB bbox, VectorTileCoordinate coord) {
    return bbox.min_x <= coord.col && bbox.max_x >= coord.col &&
           bbox.min_y <= coord.row && bbox.max_y >= coord.row;
}

// writes all indices of nodes that intersect/touch the AABB with desired zoom
// level into 'matching_nodes'.
static void QuadTreeFind(const QuadTreeNodeSlice quad_tree, S32 root, AABB bounding_box,
                         S32 zoom_level, S32Array *matching_nodes) {
    const QuadTreeNode n = quad_tree.v[root];
    switch (n.type) {
    case EMPTY:
        return;
    case LEAF: {
        if (n.tile.coordinate.level == zoom_level &&
            AABBContains(bounding_box, n.tile.coordinate)) {
            S32ArrayPush(matching_nodes, root);
        }
    } break;
    case INNER: {
        if (n.tile.coordinate.level < zoom_level) {
            QuadTreeFind(quad_tree, n.child_nw, bounding_box, zoom_level, matching_nodes);
            QuadTreeFind(quad_tree, n.child_ne, bounding_box, zoom_level, matching_nodes);
            QuadTreeFind(quad_tree, n.child_sw, bounding_box, zoom_level, matching_nodes);
            QuadTreeFind(quad_tree, n.child_se, bounding_box, zoom_level, matching_nodes);
        } else if (n.tile.coordinate.level == zoom_level) {
            if (AABBContains(bounding_box, n.tile.coordinate)) {
                S32ArrayPush(matching_nodes, root);
            }
        }

    } break;
    }
}

// returns the corresponding
static Arena *GpuDataArenaFromLevel(S32 level) {
    if (level < 8) {
        return &gpu_data_arena_level0to7;
    }
    if (level < 13) {
        return &gpu_data_arena_level8to12;
    }
    if (level < 16) {
        return &gpu_data_arena_level13to15;
    }
    return &gpu_data_arena_level16;
}

// updates all vector tile handles that are referenced with the corresponding
// indices into the quad tree
static void VectorTileHandlesFromFile(VtpkFile *file, const S32Slice tile_indices) {
    Temp_Arena_Memory scratch = GetScratch();
    for (S32 i = 0; i < tile_indices.count; i += 1) {
        VectorTileHandle *tile = &file->quad_tree.d[tile_indices.v[i]].tile;
        if (tile->status == DATA_PRESENT) {
            continue;
        }

        const S32 tile_file_row = (tile->coordinate.row / 128) * 128;
        const S32 tile_file_col = (tile->coordinate.col / 128) * 128;
        char bundle_filename[40] = {0};
        const S32 filename_size = snprintf(
            bundle_filename, sizeof(bundle_filename), "p12/tile/L%02d/R%04xC%04x.bundle",
            tile->coordinate.level, tile_file_row, tile_file_col);
        assert(filename_size == 30);
        const U32 file_index = FileIndexFromFileName(file->archive, bundle_filename);
        const U64 file_size = UncompressedFileSize(file->archive, file_index);
        unsigned char *file_content = arena_alloc(scratch.arena, file_size);
        if (!mz_zip_reader_extract_to_mem(file->archive, file_index, file_content,
                                          file_size, 0)) {
            ERROR_MSG("can not open zip archive: '%s'\n",
                      mz_zip_get_error_string(file->archive->m_last_error));
        }
        const TileBundleFileHeader *header = (TileBundleFileHeader *)file_content;
        assert(header->version == 3);
        // assert(header->record_count == 16384);TODO: figure out why this is wrong
        // assert(header->max_tile_size == 0);
        assert(header->offset_byte_count == 5);
        assert(header->slack_space == 0);
        // assert(header->file_size == 0);
        assert(header->user_header_offset == 40);
        assert(header->user_header_size == 20 + 131072);
        assert(header->legacy1 == 3);
        // assert(header->legacy2 == 16);
        assert(header->legacy3 == 16384);
        assert(header->legacy4 == 5);
        assert(header->index_size == 131072);
        const TileIndexRecord compressed_mvt = TileIndexRecordFromIndex(
            header->tile_index[tile->coordinate.row % 128][tile->coordinate.col % 128]);

        const U8 *uncompressed_tile_size_location =
            file_content + compressed_mvt.tile_offset + compressed_mvt.tile_size - 4;
        U32 uncompressed_tile_size = *(const U32 *)(uncompressed_tile_size_location);
        const MVT_ProtobufData mvt_protobuf = {
            .v = arena_alloc(scratch.arena, uncompressed_tile_size),
            .size = uncompressed_tile_size};

        mz_stream stream = {0};
        stream.next_in = file_content + compressed_mvt.tile_offset;
        stream.avail_in = compressed_mvt.tile_size;
        stream.next_out = mvt_protobuf.v;
        stream.avail_out = uncompressed_tile_size;

        int err = mz_inflateInit2(&stream, -15);
        if (err == MZ_OK) {
            err = mz_inflate(&stream, MZ_FINISH);
            mz_inflateEnd(&stream);
            // mz_inflate with MZ_FINISH returns MZ_STREAM_END on successful completion
            if (err == MZ_STREAM_END) {
                err = MZ_OK;
            }
        }
        if (!(err == MZ_OK)) {
            ERROR_MSG("%s\n", mz_error(err));
        }
        assert(err == MZ_OK);
        Arena *gpu_data_arena = GpuDataArenaFromLevel(tile->coordinate.level);
        tile->gpu_data = ParseMapboxVectorTile(gpu_data_arena, mvt_protobuf);
        tile->status = DATA_PRESENT;
    }
    temp_arena_memory_end(scratch);
}
