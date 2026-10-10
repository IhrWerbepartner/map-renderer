#pragma once

#include "../arena.c"
#include "../base.h"
#include "../json_parser.h"
#include "../string8.h"
#include "mvt.h"
#include "types.h"
#include <assert.h>
#include <math.h>
#include <raylib.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>

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

static S32 QuadTreeNodeFromJson(QuadTreeNodeArray *quad_tree, const JsonNode *node, S32 x, S32 y,
                                S32 level) {
    assert(node != &json_node_null);
    switch (node->type) {
    case JSON_ARRAY: {
        ASSERT(node->children.count == 4, "invalid list of children");
        const S32 child_nw =
            QuadTreeNodeFromJson(quad_tree, node->children.first, x * 2, y * 2, level + 1);
        const S32 child_ne = QuadTreeNodeFromJson(quad_tree, node->children.first->next, x * 2,
                                                  (y * 2) + 1, level + 1);
        const S32 child_sw = QuadTreeNodeFromJson(quad_tree, node->children.last->prev, (x * 2) + 1,
                                                  y * 2, level + 1);
        const S32 child_se = QuadTreeNodeFromJson(quad_tree, node->children.last, (x * 2) + 1,
                                                  (y * 2) + 1, level + 1);
        const S32 node_index =
            QuadTreeNodeArrayPush(quad_tree, (QuadTreeNode){.type = INNER,
                                                            .child_nw = child_nw,
                                                            .child_ne = child_ne,
                                                            .child_sw = child_sw,
                                                            .child_se = child_se});
        quad_tree->d[node_index].tile = (VectorTileHandle){
            .quad_tree_node = node_index, .coordinate = (VT_Coordinate){x, y, level}};
        return node_index;
    }
    case JSON_INTEGER: {
        U64 val = node->num.u_value;
        ASSERT(val == 0 || val == 1, "invalid node value");
        if (val == 0) {
            return 0;
        }
        const S32 node_index = QuadTreeNodeArrayPush(quad_tree, (QuadTreeNode){.type = LEAF});
        quad_tree->d[node_index].tile = (VectorTileHandle){
            .quad_tree_node = node_index, .coordinate = (VT_Coordinate){x, y, level}};
        return node_index;
    }
    case JSON_INVALID:
    case JSON_NULL:
    case JSON_BOOL:
    case JSON_OBJECT:
    case JSON_DOUBLE:
    case JSON_STRING:
    default:
        ERROR_MSG("invalid json node type");
    }
}

// return the file index of a file inside the archive
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

// set up the quad tree stored inside the tilemap
static void QuadTreeFromJson(Arena *arena, VtpkFile *vtpk_file) {
    const char *tilemap = "p12/tilemap/root.json";
    const U32 file_index = FileIndexFromFileName(vtpk_file->archive, tilemap);
    const U64 file_size = UncompressedFileSize(vtpk_file->archive, file_index);

    // zero (sentinel value) first index
    vtpk_file->quad_tree = QuadTreeNodeArrayNew(arena, safe_cast_s32_from_u64(file_size) / 2);
    QuadTreeNodeArrayPush(&vtpk_file->quad_tree, (QuadTreeNode){0});

    Temp_Arena_Memory json_scratch = temp_arena_memory_begin(arena);
    char *file_content = arena_alloc(json_scratch.arena, file_size + 1);
    mz_zip_reader_extract_to_mem(vtpk_file->archive, file_index, file_content, file_size + 1, 0);
    assert(vtpk_file->archive->m_last_error == MZ_ZIP_NO_ERROR);

    JsonNode *root = JsonNodeFromString(arena, file_content);
    assert(root->type == JSON_OBJECT);
    const JsonNode *tree_root = JsonFindKey(root, String8FromCString("index"));
    vtpk_file->root_node = QuadTreeNodeFromJson(&vtpk_file->quad_tree, tree_root, 0, 0, 0);

    temp_arena_memory_end(json_scratch);
}

// ----------------------------------------------------------------
// ----------------------- ROOT PROPERTIES ------------------------
// ----------------------------------------------------------------
static void RootPropertiesFromJson(Arena *arena, VtpkFile *vtpk_file) {
    Temp_Arena_Memory json_scratch = GetScratchConflict(&arena, 1);
    const char *root_properties = "p12/root.json";
    U32 file_index = FileIndexFromFileName(vtpk_file->archive, root_properties);
    U64 file_size = UncompressedFileSize(vtpk_file->archive, file_index);
    char *file_content = arena_alloc(json_scratch.arena, file_size + 1);
    mz_zip_reader_extract_to_mem(vtpk_file->archive, file_index, file_content, file_size + 1, 0);

    JsonNode *root = JsonNodeFromString(arena, file_content);
    assert(root->type == JSON_OBJECT);
    const JsonNode *tile_info = JsonFindKey(root, String8FromCString("tileInfo"));
    {
        const JsonNode *tile_info_rows = JsonFindKey(tile_info, String8FromCString("rows"));
        const JsonNode *tile_info_cols = JsonFindKey(tile_info, String8FromCString("cols"));
        assert(tile_info_rows->type == JSON_INTEGER);
        assert(tile_info_cols->type == JSON_INTEGER);
        vtpk_file->root_propreties.tile_info_rows = safe_cast_u32(tile_info_rows->num.u_value);
        vtpk_file->root_propreties.tile_info_cols = safe_cast_u32(tile_info_cols->num.u_value);
    }
    {
        const JsonNode *origin = JsonFindKey(tile_info, String8FromCString("origin"));
        assert(origin->type == JSON_OBJECT);
        const JsonNode *origin_x = JsonFindKey(origin, String8FromCString("x"));
        const JsonNode *origin_y = JsonFindKey(origin, String8FromCString("y"));
        assert(origin_x->type == JSON_DOUBLE);
        assert(origin_y->type == JSON_DOUBLE);
        vtpk_file->root_propreties.tile_info_origin_x = origin_x->num.dbl_value;
        vtpk_file->root_propreties.tile_info_origin_y = origin_y->num.dbl_value;
    }
    {
        const JsonNode *lods = JsonFindKey(tile_info, String8FromCString("lods"));
        vtpk_file->root_propreties.lod_resolutions = F64ArrayNew(arena, lods->children.count);
        for (JsonNode *child = lods->children.first; child != lods->children.last;
             child = child->next) {
            assert(child->type == JSON_OBJECT);
            const JsonNode *resolution = JsonFindKey(child, String8FromCString("resolution"));
            assert(resolution->type == JSON_DOUBLE);
            F64ArrayPush(&vtpk_file->root_propreties.lod_resolutions, resolution->num.dbl_value);
        }
    }
    {
        const JsonNode *min_LOD = JsonFindKey(root, String8FromCString("minLOD"));
        const JsonNode *max_LOD = JsonFindKey(root, String8FromCString("minLOD"));
        assert(min_LOD->type == JSON_INTEGER);
        assert(max_LOD->type == JSON_INTEGER);
        vtpk_file->root_propreties.lod_min = safe_cast_u32(min_LOD->num.u_value);
        vtpk_file->root_propreties.lod_max = safe_cast_u32(max_LOD->num.u_value);
    }
    temp_arena_memory_end(json_scratch);
    // TODO: figure out wich properties we actually need.
}

// ----------------------------------------------------------------
// -----------------------STYLE INFORMATION -----------------------
// ----------------------------------------------------------------
static VT_StylePaintType PaintTypeFromString(String8 string) {
    if (String8Equals(string, String8FromCString("fill"))) {
        return FILL;
    }
    if (String8Equals(string, String8FromCString("line"))) {
        return LINE;
    }
    if (String8Equals(string, String8FromCString("symbol"))) {
        return SYMBOL;
    }
    if (String8Equals(string, String8FromCString("circle"))) {
        return CIRCLE;
    }
    ERROR_MSG("unreachable");
}

// returns the Color from a rgba(r,g,b,a) string.
static Color ColorFromRGBAString(String8 color) {
    String8 prefix = String8FromCString("rgba(");
    if (!String8StartsWith(color, prefix)) {
        return BLANK;
    }
    const char *cursor = color.buf + prefix.len;
    char *endptr;
    U8 red = safe_cast_u8((U64)strtol(cursor, &endptr, 10));
    cursor = endptr + 1; // skip ','
    assert(*(cursor - 1) == ',');
    U8 green = safe_cast_u8((U64)strtol(cursor, &endptr, 10));
    cursor = endptr + 1; // skip ','
    assert(*(cursor - 1) == ',');
    U8 blue = safe_cast_u8((U64)strtol(cursor, &endptr, 10));
    cursor = endptr + 1; // skip ','
    assert(*(cursor - 1) == ',');
    U8 alpha = (U8)(strtof(cursor, &endptr) * 255.f);
    assert(endptr == color.buf + color.len - 1);
    return (Color){red, green, blue, alpha};
}

// returns the style information of a layer given its type.
static VT_StyleMapValue StyleValueFromPaint(const JsonNode *paint, VT_StylePaintType paint_type) {
    switch (paint_type) {
    case FILL: {
        const JsonNode *fill_color = JsonFindKey(paint, String8FromCString("fill-color"));
        assert(fill_color->type == JSON_STRING);
        Color color = ColorFromRGBAString(fill_color->text_value);
        return (VT_StyleMapValue){.type = paint_type, .paint = {.fill_color = color}};
    } break;
    case LINE: {
        const JsonNode *line_color = JsonFindKey(paint, String8FromCString("line-color"));
        assert(line_color->type == JSON_STRING);
        Color color = ColorFromRGBAString(line_color->text_value);
        return (VT_StyleMapValue){.type = paint_type, .paint = {.line_color = color}};
    } break;
    case SYMBOL: {
        // TODO: icon color represents what exactly? tint? the icon has a color by
        // itself no?
        return (VT_StyleMapValue){.type = paint_type, .paint = {.icon_image = (String8){0}}};
    } break;
    case CIRCLE: {
        const JsonNode *circle_color = JsonFindKey(paint, String8FromCString("circle-color"));
        assert(circle_color->type == JSON_STRING);
        Color color = ColorFromRGBAString(circle_color->text_value);
        return (VT_StyleMapValue){.type = paint_type, .paint = {.circle_color = color}};
    } break;
    default:
        ERROR_MSG("unreachable");
    }
}

// creats the lookup map to identify the style by (layer, key, value) triple
static void StyleFromJson(Arena *arena, VtpkFile *vtpk_file) {
    Temp_Arena_Memory json_scratch = GetScratchConflict(&arena, 1);
    const char *styles = "p12/resources/styles/root.json";
    U32 file_index = FileIndexFromFileName(vtpk_file->archive, styles);
    U64 file_size = UncompressedFileSize(vtpk_file->archive, file_index);
    char *file_content = arena_alloc(json_scratch.arena, file_size + 1);
    mz_zip_reader_extract_to_mem(vtpk_file->archive, file_index, file_content, file_size + 1, 0);

    JsonNode *root = JsonNodeFromString(json_scratch.arena, file_content);
    assert(root->type == JSON_OBJECT);
    const JsonNode *layers = JsonFindKey(root, String8FromCString("layers"));
    assert(layers->type == JSON_ARRAY);
    vtpk_file->layer_styles = NULL;

    for (JsonNode *layer_style = layers->children.first; layer_style != &json_node_null;
         layer_style = layer_style->next) {
        const JsonNode *source_layer = JsonFindKey(layer_style, String8FromCString("source-layer"));
        assert(source_layer->type == JSON_STRING);

        // key
        VT_StyleMapKey key = (VT_StyleMapKey){0};
        key.layer_hash = HashFromString8(0, source_layer->text_value);

        const JsonNode *filter = JsonFindKey(layer_style, String8FromCString("filter"));
        if (filter != &json_node_null) {
            assert(filter->type == JSON_ARRAY);
            assert(filter->children.count == 3);
            assert(filter->children.first->type == JSON_STRING);
            if (String8Equals(filter->children.first->text_value, String8FromCString("all"))) {
                // account for the case that the filter specifies a quantifier ("all").
                // Just take the first criterion and treat it as the sole filter.
                // The second one should be irrelevant as the layer already only contains
                // the relevant geometry type.
                filter = filter->children.first->next;
            }
            const JsonNode *filter_key = filter->children.first->next;
            assert(filter_key->type == JSON_STRING);
            const JsonNode *filter_val = filter->children.last;
            assert(filter_val->type == JSON_INTEGER);
            key.filter_key_hash = HashFromString8(0, filter_key->text_value);
            key.filter_value = filter_val->num.s_value;
        }
        U32 zoom_min = vtpk_file->root_propreties.lod_min;
        U32 zoom_max = vtpk_file->root_propreties.lod_max;
        const JsonNode *zoom_node_min = JsonFindKey(layer_style, String8FromCString("minzoom"));
        if (zoom_node_min != &json_node_null) {
            if (zoom_node_min->type == JSON_INTEGER) {
                zoom_min = (U32)zoom_node_min->num.u_value;
            } else if (zoom_node_min->type == JSON_DOUBLE) {
                zoom_min = (U32)zoom_node_min->num.dbl_value;
            } else {
                ERROR_MSG("invalid minzoom");
            }
        }
        const JsonNode *zoom_node_max = JsonFindKey(layer_style, String8FromCString("maxzoom"));
        if (zoom_node_max != &json_node_null) {
            if (zoom_node_max->type == JSON_INTEGER) {
                zoom_max = (U32)zoom_node_max->num.u_value;
            } else if (zoom_node_max->type == JSON_DOUBLE) {
                zoom_max = (U32)ceil(zoom_node_max->num.dbl_value);
            } else {
                ERROR_MSG("invalid maxzoom");
            }
        }

        // value
        const JsonNode *layer_type = JsonFindKey(layer_style, String8FromCString("type"));
        assert(layer_type->type == JSON_STRING);
        const VT_StylePaintType paint_type = PaintTypeFromString(layer_type->text_value);
        const JsonNode *paint = JsonFindKey(layer_style, String8FromCString("paint"));
        VT_StyleMapValue value = StyleValueFromPaint(paint, paint_type);

        for (U32 zoom = zoom_min; zoom < zoom_max; zoom += 1) {
            key.zoom = zoom;
            hmput(vtpk_file->layer_styles, key, value);
        }
        temp_arena_memory_end(json_scratch);
    }
}

// Parse the whole VTPK Zip Bundle and set up internal datastructures
static VtpkFile *VtpkParseFile(Arena *arena, const char *filepath) {
    VtpkFile *vtpk_file = arena_alloc(arena, sizeof(VtpkFile));
    vtpk_file->archive = arena_alloc(arena, sizeof(mz_zip_archive));
    mz_zip_zero_struct(vtpk_file->archive);
    OpenZipArchive(vtpk_file->archive, filepath);
    assert(vtpk_file->archive != NULL);
    QuadTreeFromJson(arena, vtpk_file);
    RootPropertiesFromJson(arena, vtpk_file);
    StyleFromJson(arena, vtpk_file);
    return vtpk_file;
}

// returns true if the bounding box contains or touches the coordinate
static bool AABBContains(AABB bbox, VT_Coordinate coord) {
    return bbox.min_x <= coord.col && bbox.max_x >= coord.col && bbox.min_y <= coord.row &&
           bbox.max_y >= coord.row;
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

// updates all vector tile handles that are referenced with the corresponding
// indices into the quad tree
static void VectorTileHandlesFromFile(VtpkFile *file, const S32Slice tile_indices) {
    Temp_Arena_Memory scratch = GetScratch();
    struct {
        char *key;
        U8 *value;
    } *bundle_files_map;
    sh_new_arena(bundle_files_map);
    for (S32 i = 0; i < tile_indices.count; i += 1) {
        VectorTileHandle *tile = &file->quad_tree.d[tile_indices.v[i]].tile;
        if (tile->status == DATA_PRESENT) {
            continue;
        }
        // READ AND DECOMPRESS PROTOBUF TILE DATA
        clock_t decompress_start, decompress_end;
        double decompress_cpu_time_seconds;

        decompress_start = clock();

        const S32 tile_file_row = (tile->coordinate.row / 128) * 128;
        const S32 tile_file_col = (tile->coordinate.col / 128) * 128;
        char bundle_filename[40] = {0};
        const S32 filename_size =
            snprintf(bundle_filename, sizeof(bundle_filename), "p12/tile/L%02d/R%04xC%04x.bundle",
                     tile->coordinate.level, tile_file_row, tile_file_col);
        assert(filename_size == 30);
        const U32 file_index = FileIndexFromFileName(file->archive, bundle_filename);
        const U64 file_size = UncompressedFileSize(file->archive, file_index);
        U8 *file_content = shget(bundle_files_map, &bundle_filename);
        if (file_content == NULL) {
            clock_t tile_parse_start, tile_parse_end;
            double tile_parse_cpu_time_seconds;

            tile_parse_start = clock();
            file_content = arena_alloc(scratch.arena, file_size);
            if (!mz_zip_reader_extract_to_mem(file->archive, file_index, file_content, file_size,
                                              0)) {
                ERROR_MSG("can not open zip archive: '%s'\n",
                          mz_zip_get_error_string(file->archive->m_last_error));
            }
            tile_parse_end = clock();
            tile_parse_cpu_time_seconds =
                ((double)(tile_parse_end - tile_parse_start)) / CLOCKS_PER_SEC;
            fprintf(stdout, "EXTRACTING FILE FOR TILE: c: %d, r: %d, l: %d: %f MS\n",
                    tile->coordinate.col, tile->coordinate.row, tile->coordinate.level,
                    Thousand(tile_parse_cpu_time_seconds));
            shput(bundle_files_map, &bundle_filename, file_content);
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

        // decompress (unsually gzip compressed) individual tile
        const TileIndexRecord compressed_mvt = TileIndexRecordFromIndex(
            header->tile_index[tile->coordinate.row % 128][tile->coordinate.col % 128]);

        const U8 *uncompressed_tile_size_location =
            file_content + compressed_mvt.tile_offset + compressed_mvt.tile_size - 4;
        // read 4 bytes (maybe unaligned)
        U32 uncompressed_tile_size;
        memcpy(&uncompressed_tile_size, uncompressed_tile_size_location, sizeof(U32));
        const MVT_ProtobufData mvt_protobuf =
            (MVT_ProtobufData){.v = arena_alloc(scratch.arena, uncompressed_tile_size),
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
        decompress_end = clock();
        decompress_cpu_time_seconds =
            ((double)(decompress_end - decompress_start)) / CLOCKS_PER_SEC;
        fprintf(stdout, "DECOMPRESSING TILE: c: %d, r: %d, l: %d: %f MS\n", tile->coordinate.col,
                tile->coordinate.row, tile->coordinate.level,
                Thousand(decompress_cpu_time_seconds));

        {
            // PARSE TILE INTO GPU DATA
            clock_t tile_parse_start, tile_parse_end;
            double tile_parse_cpu_time_seconds;

            tile_parse_start = clock();
            // work done here
            // ------------------------------------------------------------------
            tile->gpu_data =
                ParseMapboxVectorTile(tile->coordinate, mvt_protobuf, file->layer_styles);
            // ------------------------------------------------------------------
            tile_parse_end = clock();
            tile_parse_cpu_time_seconds =
                ((double)(tile_parse_end - tile_parse_start)) / CLOCKS_PER_SEC;
            fprintf(stdout, "PARSING TILE: c: %d, r: %d, l: %d: %f MS\n", tile->coordinate.col,
                    tile->coordinate.row, tile->coordinate.level,
                    Thousand(tile_parse_cpu_time_seconds));
        }
        assert(IsRenderTextureValid(tile->gpu_data.texture));
        UploadMesh(&tile->gpu_data.mesh, false);

        tile->status = DATA_PRESENT;
    }
    shfree(bundle_files_map);
    temp_arena_memory_end(scratch);
}
