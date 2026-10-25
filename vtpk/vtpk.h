#include "../arena.c"
#include "../base.h"
#include "../vendor/raymath.h"
#include "mvt.h"
#include "types.h"
#include "vtpk_reader.h"
#include <assert.h>
#include <math.h>
#include <raylib.h>
#include <rlgl.h>
#include <stdbool.h>
#ifdef DEBUG
#include <stdio.h>
#endif

// NOTE: fine tune
#define DRAW_CACHE_SIZE (40)
#define DRAW_TILE_PIXEL_COUNT (512 * 2)

typedef struct DrawCache DrawCache;
struct DrawCache {
    VectorTileHandleArray front_buffer;
    VectorTileHandleArray back_buffer;
};

typedef struct VTPK_RenderOptions VTPK_RenderOptions;
struct VTPK_RenderOptions {
    bool show_grid;
    bool show_bounding_box;
    bool show_meshes;
    bool show_textures;
};

// #ifdef DEBUG
static void PrintMissingCoordinates(const VT_CoordinateSlice s) {
    fprintf(stderr, "[");
    for (S32 i = 0; i < s.count; i += 1) {
        if (i > 0) {
            fprintf(stderr, ", ");
        }
        const VT_Coordinate coord = s.v[i];
        fprintf(stderr, "(r: %d, c: %d, l: %d)", coord.row, coord.col, coord.level);
    }
    fprintf(stderr, "]\n");
}
static void PrintMissingTiles(const S32Slice s, QuadTreeNodeArray quad_tree) {
    fprintf(stderr, "[");
    for (S32 i = 0; i < s.count; i += 1) {
        if (i > 0) {
            fprintf(stderr, ", ");
        }
        const VT_Coordinate coords = quad_tree.d[s.v[i]].tile.coordinate;
        fprintf(stderr, "(r: %d, c: %d, l: %d)", coords.row, coords.col, coords.level);
    }
    fprintf(stderr, "]\n");
}
// #endif

// initializes the gpu_buffers inside the draw cache
static void InitDrawCache(DrawCache *draw_cache) {
    for (S32 i = 0; i < draw_cache->front_buffer.capacity; i += 1) {
        draw_cache->front_buffer.d[i].gpu_data.texture =
            LoadRenderTexture(MVT_TEXTURE_SIZE, MVT_TEXTURE_SIZE);
    }
}

// returns a []S32 with all indices of the quad_tree that are visible but not currently in
// the cache
// TODO: make this not a trivial O(n^2) operation but something smarter like using a
// hashset
static VT_CoordinateSlice NonCachedTileIndices(Arena *arena,
                                               const VT_CoordinateArray *visible_coordinates,
                                               VectorTileHandleSlice cached_tiles) {
    VT_CoordinateArray missing_coordinates =
        VT_CoordinateArrayNew(arena, visible_coordinates->count);
    for (S32 i = 0; i < visible_coordinates->count; i += 1) {
        bool found = false;
        for (S32 j = 0; j < cached_tiles.count; j += 1) {
            if (VT_CoordinateEq(visible_coordinates->d[i], cached_tiles.v[j].coordinate)) {
                found = true;
                break;
            }
        }
        if (!found) {
            VT_CoordinateArrayPush(&missing_coordinates, visible_coordinates->d[i]);
        }
    }
    return VT_CoordinateSliceFromArray(&missing_coordinates);
}

// copy tiles within AABB to back buffer and request missing ones fromt the
// tilecache. finally swap back and front buffer.
// furthemore take care that all GPU_Buffers/Handles remain the same set.
// This ensures that we do not need to create new textures.
static VectorTileHandleSlice TilesFromBoundingBox(VtpkFile *vtpk_file, DrawCache *draw_cache,
                                                  S32 zoom_level) {
    assert(draw_cache->back_buffer.capacity == draw_cache->front_buffer.capacity);
    Temp_Arena_Memory scratch = GetScratch();

    // clear back buffer (becomes new front)
    // we need to iterate over the whole buffer (capaicty!) to not loose any tiles
    // 1. copy over all tiles still visible
    // 2. save all gpu_buffers that are now free for reuse on a stack
    VectorTileHandleArrayReset(&draw_cache->back_buffer);
    VectorTileGPU_DataArray unused_gpu_buffers =
        VectorTileGPU_DataArrayNew(scratch.arena, DRAW_CACHE_SIZE);
    const AABB bbox = vtpk_file->bounding_box;
    for (S32 i = 0; i < draw_cache->front_buffer.capacity; i += 1) {
        const VectorTileHandle tile = draw_cache->front_buffer.d[i];
        // just copy over the data
        if (i >= draw_cache->front_buffer.count) {
            VectorTileGPU_DataArrayPush(&unused_gpu_buffers, tile.gpu_data);
            continue;
        }

        // this tile was visible before, perform the actual test
        if (tile.coordinate.level == zoom_level && AABBContains(bbox, tile.coordinate)) {
            VectorTileHandleArrayPush(&draw_cache->back_buffer, tile);
        } else {
            VectorTileGPU_DataArrayPush(&unused_gpu_buffers, tile.gpu_data);
        }
    }

    const S32 quad_tree_zoom_level = ClampTop(zoom_level, vtpk_file->root_propreties.lod_max);
    S32Array visible_tile_indices = S32ArrayNew(scratch.arena, DRAW_CACHE_SIZE);
    QuadTreeFind(QuadTreeNodeSliceFromArray(&vtpk_file->quad_tree), vtpk_file->root_node, bbox,
                 quad_tree_zoom_level, &visible_tile_indices);
#ifdef DEBUG
    fprintf(stderr, "Visible tiles: ");
    PrintMissingTiles(S32SliceFromArray(&visible_tile_indices), vtpk_file->quad_tree);
#endif

    VT_CoordinateArray visible_coordinates = VT_CoordinateArrayNew(scratch.arena, DRAW_CACHE_SIZE);
    for (S32 i = 0; i < visible_tile_indices.count; i += 1) {
        VT_Coordinate coord = vtpk_file->quad_tree.d[visible_tile_indices.d[i]].tile.coordinate;
        VT_CoordinateArrayPush(&visible_coordinates,
                               (VT_Coordinate){coord.row, coord.col, zoom_level});
    }
#ifdef DEBUG
    fprintf(stderr, "Visible Coordinates: ");
    PrintMissingCoordinates(VT_CoordinateSliceFromArray(&visible_coordinates));
#endif

    const VT_CoordinateSlice missing_coordinates =
        NonCachedTileIndices(scratch.arena, &visible_coordinates,
                             VectorTileHandleSliceFromArray(&draw_cache->back_buffer));
#ifdef DEBUG
    fprintf(stderr, "Missing Coordinates: ");
    PrintMissingCoordinates(missing_coordinates);
#endif
    GPU_DataFromCoordinates(vtpk_file, missing_coordinates, &draw_cache->back_buffer,
                            &unused_gpu_buffers);

    // copy over the rest of the gpu_buffers in the back of the array
    // this is to not loose any buffers
    const S32 visible_tiles_count = draw_cache->back_buffer.count;
    while (unused_gpu_buffers.count > 0) {
        VectorTileHandle dummy_handle = {0};
        dummy_handle.gpu_data = VectorTileGPU_DataArrayPop(&unused_gpu_buffers);
        VectorTileHandleArrayPush(&draw_cache->back_buffer, dummy_handle);
    }
    draw_cache->back_buffer.count = visible_tiles_count;

    // swap back and front buffers
    Swap(VectorTileHandleArray, draw_cache->front_buffer, draw_cache->back_buffer);
    temp_arena_memory_end(scratch);
    return VectorTileHandleSliceFromArray(&draw_cache->front_buffer);
}

typedef struct TileCamera TileCamera;
struct TileCamera {
    // Camera2D, defines position/orientation in 2d space
    Vector2 offset;   // Camera offset (screen space offset from window origin)
    Vector2 target;   // Camera target (world space target point that is mapped to screen
                      // space offset)
    float rotation;   // Camera rotation in degrees (pivots around target)
    float zoom;       // Camera zoom (scaling around target), must not be set to 0, set to 1.0f
                      // for no scale
    S32 tile_lod_max; // the highest level of detail represented through the quad-tree.
                      // All higher zooms have to be done by zooming in a "real" camera and
                      // providing the correct level information to the tile reader.
};
static Camera2D Camera2DFromTileCamera(TileCamera camera) {
    Camera2D c =
        (Camera2D){camera.offset, camera.target, camera.rotation, 1.f + fmodf(camera.zoom, 1.f)};

    // account for "overzooming" past the tile lod, therefore we need to zoom in the real camera.
    c.zoom += ClampBot(0.f, floorf(camera.zoom) - (F32)camera.tile_lod_max);
    return c;
}

// Initialize 2D mode with custom camera (2D)
static void BeginModeTile(TileCamera camera) {
    rlDrawRenderBatchActive(); // Update and draw internal render batch

    rlLoadIdentity(); // Reset current matrix (modelview)

    // Apply 2d camera transformation to modelview
    const Camera2D camera_2d = Camera2DFromTileCamera(camera);
    // assert(camera_2d.zoom >= 5.f);
    // assert(camera_2d.zoom < 6.f);
    rlMultMatrixf(MatrixToFloat(GetCameraMatrix2D(camera_2d)));
}

// End 2D mode with custom camera
static void EndModeTile(void) { EndMode2D(); }

static TileCamera ResetTileCamera(Screen screen, S32 tile_lod_max) {
    return (TileCamera){
        .zoom = 1.0f, // speicfies the zoom level. Is in the range (0; +infinity).
                      // the non integer part is used to interpolate between the tile sizes
                      // meaning it zooms the camera until the next tile size is hit.
        .rotation = 0.0f,
        .offset = (Vector2){(float)screen.width / 2.0f, (float)screen.height / 2.0f},
        .target.x = (F32)DRAW_TILE_PIXEL_COUNT / 2.f,
        .target.y = (F32)DRAW_TILE_PIXEL_COUNT / 2.f,
        .tile_lod_max = tile_lod_max,
    };
}

static void UpdateTileCameraPos(TileCamera *tile_camera, Screen screen) {

    const Camera2D camera_2d = Camera2DFromTileCamera(*tile_camera);

    // Translate based on mouse right click
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        const Vector2 delta = GetMouseDelta();
        const Vector2 delta_scaled = Vector2Scale(delta, -1.0f / camera_2d.zoom);
        tile_camera->target = Vector2Add(tile_camera->target, delta_scaled);
    }

    // Zoom based on mouse wheel
    const F32 wheel = GetMouseWheelMove();
    if (wheel != 0) {
        // Get the world point that is under the mouse
        const Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), camera_2d);
        const F32 wheel_scaled = 0.2f * wheel;
        // if we cross a zoom boundary (from one level to another update position)
        const S32 zoom_level_old = (S32)tile_camera->zoom;
        const S32 zoom_level_new = (S32)(wheel_scaled + tile_camera->zoom);
        if (zoom_level_old < zoom_level_new && zoom_level_new <= tile_camera->tile_lod_max) {
            tile_camera->target = Vector2Scale(mouseWorldPos, 2.f);
            tile_camera->target =
                Vector2Subtract(tile_camera->target, (Vector2){0.f, (F32)DRAW_TILE_PIXEL_COUNT});
        } else if (zoom_level_old > zoom_level_new && zoom_level_new < tile_camera->tile_lod_max) {
            tile_camera->target = Vector2Scale(mouseWorldPos, 0.5f);
            tile_camera->target =
                Vector2Add(tile_camera->target, (Vector2){0.f, (F32)DRAW_TILE_PIXEL_COUNT / 2.f});
        } else {
            tile_camera->target = mouseWorldPos;
        }

        // Set the offset to where the mouse is
        tile_camera->offset = GetMousePosition();

        // Set the target to match, so that the camera maps the world space
        // point under the cursor to the screen space point under the cursor at
        // any zoom
        tile_camera->zoom = Clamp(tile_camera->zoom + (0.2f * wheel), 0.001f, 50.f);
    }

    // Camera reset (zoom and rotation)
    if (IsKeyPressed(KEY_R)) {
        *tile_camera = ResetTileCamera(screen, tile_camera->tile_lod_max);
    }
}

static Matrix ModelTransformFromCoords(VT_Coordinate coords, U32 units_per_tile_current,
                                       U32 units_per_tile_max) {
    const F32 scaling = (F32)units_per_tile_max / (F32)units_per_tile_current;
    const F32 world_pixel_size = (F32)units_per_tile_max * exp2f((F32)coords.level);
    const F32 tile_world_size = world_pixel_size / exp2f((F32)coords.level);
    return MatrixCompose(
        (Vector3){(F32)coords.col * tile_world_size, (F32)coords.row * tile_world_size, 0.0f},
        QuaternionIdentity(), (Vector3){scaling, scaling, 1.f});
}

static void DrawWorldGrid(Camera2D camera, Screen screen, F32 gridSize, Color gridColor,
                          Color axisColor) {
    // 1. Calculate the visible world-space bounds from the screen viewport
    Vector2 topLeftWorld = GetScreenToWorld2D((Vector2){0, 0}, camera);
    Vector2 bottomRightWorld =
        GetScreenToWorld2D((Vector2){(float)screen.width, (float)screen.height}, camera);

    // Find min and max bounds for drawing
    const F32 minX = fminf(topLeftWorld.x, bottomRightWorld.x);
    const F32 maxX = fmaxf(topLeftWorld.x, bottomRightWorld.x);
    const F32 minY = fminf(topLeftWorld.y, bottomRightWorld.y);
    const F32 maxY = fmaxf(topLeftWorld.y, bottomRightWorld.y);

    // 2. Snap start positions to the nearest grid step
    const F32 startX = floorf(minX / gridSize) * gridSize;
    const F32 endX = ceilf(maxX / gridSize) * gridSize;
    const F32 startY = floorf(minY / gridSize) * gridSize;
    const F32 endY = ceilf(maxY / gridSize) * gridSize;

    // 3. Draw Vertical Grid Lines
    for (F32 x = startX; x <= endX; x += gridSize) {
        // Highlight main origin axis (x = 0)
        Color col = (fabsf(x) < 0.001f) ? axisColor : gridColor;
        DrawLineV((Vector2){x, startY}, (Vector2){x, endY}, col);
    }

    // 4. Draw Horizontal Grid Lines
    for (F32 y = startY; y <= endY; y += gridSize) {
        // Highlight main origin axis (y = 0)
        Color col = (fabsf(y) < 0.001f) ? axisColor : gridColor;
        DrawLineV((Vector2){startX, y}, (Vector2){endX, y}, col);
    }
}

static void UpdateVisibleBoundingBox(VtpkFile *vtpk_file, TileCamera camera_tile, Screen screen) {
    const Camera2D camera_2d = Camera2DFromTileCamera(camera_tile);
    {
        const Vector2 world_top_left = GetScreenToWorld2D(Vector2Zero(), camera_2d);
        const Vector2 tile_top_left = Vector2Scale(world_top_left, 1.f / DRAW_TILE_PIXEL_COUNT);
        // TraceLog(LOG_INFO, "top_left tile: (col: %.1f, row: %.1f)", tile_top_left.x,
        // tile_top_left.y);
        vtpk_file->bounding_box.min_x = (S32)floorf(tile_top_left.x);
        vtpk_file->bounding_box.min_y = (S32)floorf(tile_top_left.y) - 1;
    }
    {
        const Vector2 world_bot_right =
            GetScreenToWorld2D((Vector2){(F32)screen.width, (F32)screen.height}, camera_2d);
        const Vector2 tile_bot_right = Vector2Scale(world_bot_right, 1.f / DRAW_TILE_PIXEL_COUNT);
        // TraceLog(LOG_INFO, "bot_right tile: (col: %.0f, row: %.0f)",
        // ceilf(tile_bot_right.x), ceilf(tile_bot_right.y));
        vtpk_file->bounding_box.max_x = (S32)floorf(tile_bot_right.x);
        vtpk_file->bounding_box.max_y = (S32)floorf(tile_bot_right.y) - 1;
    }
}

static void VtpkDisplayFile(const char *filename, Screen screen) {
    Temp_Arena_Memory scratch = GetScratch();

    VTPK_RenderOptions render_options = {.show_meshes = true, .show_textures = true};
    VtpkFile *vtpk_file = VtpkParseFile(scratch.arena, filename);
    vtpk_file->bounding_box = (AABB){min_S32, min_S32, max_S32, max_S32};

    TileCamera camera = ResetTileCamera(screen, vtpk_file->root_propreties.lod_max);

    DrawCache draw_cache = (DrawCache){VectorTileHandleArrayNew(scratch.arena, DRAW_CACHE_SIZE),
                                       VectorTileHandleArrayNew(scratch.arena, DRAW_CACHE_SIZE)};
    InitDrawCache(&draw_cache);
    Material material = LoadMaterialDefault();
    material.maps[MATERIAL_MAP_DIFFUSE].color = BLUE;

    // Shader shader_blur = LoadShader(0, "shaders/blur.fs");

    SetTargetFPS(60);

    // Detect window close button or ESC key
    while (!WindowShouldClose()) {
        // Update
        //----------------------------------------------------------------------------------
        if (IsKeyPressed(KEY_B))
            render_options.show_bounding_box ^= true;
        if (IsKeyPressed(KEY_M))
            render_options.show_meshes ^= true;
        if (IsKeyReleased(KEY_T))
            render_options.show_textures ^= true;
        if (IsKeyReleased(KEY_G))
            render_options.show_grid ^= true;
        UpdateTileCameraPos(&camera, screen);
        UpdateVisibleBoundingBox(vtpk_file, camera, screen);

        //----------------------------------------------------------------------------------
        // Draw
        //----------------------------------------------------------------------------------

        Vector3 mesh_pos = {0};

        BeginDrawing();

        ClearBackground(RAYWHITE);
        BeginModeTile(camera);
        if (render_options.show_grid) {
            DrawWorldGrid(Camera2DFromTileCamera(camera), screen, (F32)DRAW_TILE_PIXEL_COUNT,
                          LIGHTGRAY, RED);
        }
        {
            const VectorTileHandleSlice tiles =
                TilesFromBoundingBox(vtpk_file, &draw_cache, ClampBot((S32)camera.zoom, 0));
            for (S32 i = 0; i < tiles.count; i += 1) {
                const VectorTileHandle tile = tiles.v[i];
                if (render_options.show_meshes) {
                    const Matrix mesh_transform = ModelTransformFromCoords(
                        tile.coordinate, MVT_MESH_SIZE, DRAW_TILE_PIXEL_COUNT);
                    rlPushMatrix();
                    rlLoadIdentity();
                    rlMultMatrixf(MatrixToFloat(mesh_transform));

                    material.maps[MATERIAL_MAP_DIFFUSE].color = WHITE;
                    mesh_pos.x = mesh_transform.m12;
                    mesh_pos.y = mesh_transform.m13;
                    mesh_pos.z = mesh_transform.m14;

                    DrawMesh(tile.gpu_data.mesh, material,
                             MatrixTranslate(0, MVT_MESH_SIZE - (DRAW_TILE_PIXEL_COUNT / 2), 0));
                    if (render_options.show_bounding_box) {
                        const BoundingBox bbox = GetMeshBoundingBox(tile.gpu_data.mesh);
                        DrawRectangleLines((S32)bbox.min.x, (S32)bbox.min.y,
                                           (S32)(bbox.max.x - bbox.min.x),
                                           (S32)(bbox.max.y - bbox.min.y), GREEN);
                        DrawTextEx(GetFontDefault(),
                                   TextFormat("[ROW: %d COL: %d LVL: %d]", tile.coordinate.row,
                                              tile.coordinate.col, tile.coordinate.level),
                                   (Vector2){bbox.min.x + 10, bbox.min.y + 10}, 100, 1, RED);
                        const Vector2 screen_pos_min = GetWorldToScreen2D(
                            (Vector2){bbox.min.x, bbox.min.y}, Camera2DFromTileCamera(camera));
                        TraceLog(LOG_INFO, "bbox min screen coords: [%f, %f]", screen_pos_min.x,
                                 screen_pos_min.y);
                        const Vector2 screen_pos_max = GetWorldToScreen2D(
                            (Vector2){bbox.max.x, bbox.max.y}, Camera2DFromTileCamera(camera));
                        TraceLog(LOG_INFO, "bbox max screen coords: [%f, %f]", screen_pos_max.x,
                                 screen_pos_max.y);
                    }
                    rlPopMatrix();
                }
                if (render_options.show_textures) {
                    const Matrix texture_transform = ModelTransformFromCoords(
                        tile.coordinate, MVT_TEXTURE_SIZE, DRAW_TILE_PIXEL_COUNT);
                    rlPushMatrix();
                    rlLoadIdentity();
                    rlMultMatrixf(MatrixToFloat(texture_transform));
                    // BeginShaderMode(shader_blur);
                    assert(IsTextureValid(tile.gpu_data.texture.texture));
                    DrawTextureRec(
                        tile.gpu_data.texture.texture,
                        (Rectangle){
                            .x = 0, .y = 0, .width = MVT_TEXTURE_SIZE, .height = -MVT_TEXTURE_SIZE},
                        (Vector2){.x = 0, .y = MVT_TEXTURE_SIZE}, WHITE);
                    // EndShaderMode();
                    rlPopMatrix();
                }
            }
        }
        EndModeTile();
        //  --------------------- HUD -------------------------
        DrawText(TextFormat("CURRENT ZOOM: %03.04f", camera.zoom), 640, 10, 20, BLACK);
        DrawText(TextFormat("CAMERA TARGET: [%03.04f, %03.04f]", camera.target.x, camera.target.y),
                 640, 40, 20, BLACK);
        DrawFPS(640, 70);
        Vector2 mouseWorldPos =
            GetScreenToWorld2D(GetMousePosition(), Camera2DFromTileCamera(camera));
        DrawText(TextFormat("MOUSE POS : [%03.04f, %03.04f]", mouseWorldPos.x, mouseWorldPos.y),
                 100, 10, 20, BLACK);
        DrawText(TextFormat("FRAME TIME: %f MS", Thousand(GetFrameTime())), 640, 100, 20, BLACK);
        EndDrawing();
        //----------------------------------------------------------------------------------
    }
}
