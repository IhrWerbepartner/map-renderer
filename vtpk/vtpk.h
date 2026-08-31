#include "../arena.c"
#include "../base.h"
#include "../vendor/raymath.h"
#include "raymath.h"
#include "vtpk_reader.h"
#include <assert.h>
#include <math.h>
#include <raylib.h>
#include <rlgl.h>
#include <stdbool.h>
#include <stdio.h>

#define DRAW_CACHE_SIZE (1024) // TODO: fine tune

typedef struct DrawCache DrawCache;
struct DrawCache {
    VectorTileHandleArray front_buffer;
    VectorTileHandleArray back_buffer;
};

typedef struct VTPK_RenderOptions VTPK_RenderOptions;
struct VTPK_RenderOptions {
    bool show_grid;
    bool show_bounding_box;
};

static void PrintMissingTiles(S32Slice s, QuadTreeNodeArray quad_tree) {
    fprintf(stderr, "[");
    for (S32 i = 0; i < s.count; i += 1) {
        if (i > 0) {
            fprintf(stderr, ", ");
        }
        VectorTileCoordinate coords = quad_tree.d[s.v[i]].tile.coordinate;
        fprintf(stderr, "(r: %d, c: %d, l: %d)", coords.row, coords.col, coords.level);
    }
    fprintf(stderr, "]\n");
}

// returns a []S32 with all indices of the quad_tree that are visible but not currently in
// the cache
// TODO: make this not a trivial O(n^2) operation but something smarter like using a
// hashset
static S32Slice NonCachedTileIndices(Arena *arena, const S32Slice visible_tiles,
                                     VectorTileHandleSlice cached_tiles) {
    S32Array missing_tiles = S32ArrayNew(arena, visible_tiles.count);
    for (S32 i = 0; i < visible_tiles.count; i += 1) {
        bool found = false;
        for (S32 j = 0; j < cached_tiles.count; j += 1) {
            if (visible_tiles.v[i] == cached_tiles.v[j].quad_tree_node) {
                found = true;
                break;
            }
        }
        if (!found) {
            S32ArrayPush(&missing_tiles, visible_tiles.v[i]);
        }
    }
    return S32SliceFromArray(&missing_tiles);
}

// copy tiles within AABB to back buffer and request missing ones fromt the
// tilecache. finally swap back and front buffer.
static VectorTileHandleSlice
MeshesFromBoundingBox(VtpkFile *vtpk_file, DrawCache *draw_cache, S32 zoom_level) {
    // clear back buffer (becomes new front)
    VectorTileHandleArrayReset(&draw_cache->back_buffer);
    const AABB bbox = vtpk_file->bounding_box;
    for (S32 i = 0; i < draw_cache->front_buffer.count; i += 1) {
        const VectorTileHandle handle = draw_cache->front_buffer.d[i];
        if (handle.coordinate.level == zoom_level &&
            AABBContains(bbox,
                         vtpk_file->quad_tree.d[handle.quad_tree_node].tile.coordinate)) {
            VectorTileHandleArrayPush(&draw_cache->back_buffer, handle);
        }
    }

    ScratchArena() {
        S32Array visible_tiles =
            S32ArrayNew(arena_auto_close_latch.scratch.arena, DRAW_CACHE_SIZE);
        QuadTreeFind(QuadTreeNodeSliceFromArray(&vtpk_file->quad_tree),
                     vtpk_file->root_node, bbox, zoom_level, &visible_tiles);
#ifdef DEBUG
        fprintf(stderr, "visible_tiles: ");
        PrintMissingTiles(S32SliceFromArray(&visible_tiles), vtpk_file->quad_tree);
#endif
        // TODO: it would be cool to get these indices in a sorted order
        // this makes intersecting them much cheaper.
        // lets see if we can get this invariant enforced in the quad tree!!
        const S32Slice missing_tile_indices = NonCachedTileIndices(
            arena_auto_close_latch.scratch.arena, S32SliceFromArray(&visible_tiles),
            VectorTileHandleSliceFromArray(&draw_cache->back_buffer));
#ifdef DEBUG
        fprintf(stderr, "missing_tiles: ");
        PrintMissingTiles(missing_tile_indices, vtpk_file->quad_tree);
#endif
        VectorTileHandlesFromFile(vtpk_file, missing_tile_indices);
        for (S32 i = 0; i < missing_tile_indices.count; i += 1) {
            const VectorTileHandle handle =
                vtpk_file->quad_tree.d[missing_tile_indices.v[i]].tile;
            assert(handle.status == DATA_PRESENT);
            VectorTileHandleArrayPush(&draw_cache->back_buffer, handle);
        }
    }

    // swap back and front buffers
    Swap(VectorTileHandleArray, draw_cache->front_buffer, draw_cache->back_buffer);
    return VectorTileHandleSliceFromArray(&draw_cache->front_buffer);
}

// represents world space [0; 2^zoom - 1] X [0; 2^zoom - 1]. in integer coords to not
// loose precision. maybe unecessary?
typedef struct LonLat2 LonLat2;
struct LonLat2 {
    F32 lon, lat;
};
static LonLat2 LonLat2FromVector2(Vector2 vec) { return (LonLat2){vec.x, vec.y}; }

typedef struct TileCamera TileCamera;
struct TileCamera {
    // Camera2D, defines position/orientation in 2d space
    Vector2 offset; // Camera offset (screen space offset from window origin)
    Vector2 target; // Camera target (world space target point that is mapped to screen
                    // space offset)
    float rotation; // Camera rotation in degrees (pivots around target)
    float zoom; // Camera zoom (scaling around target), must not be set to 0, set to 1.0f
                // for no scale
};
static Camera2D Camera2DFromTileCamera(TileCamera camera) {
    return (Camera2D){camera.offset, camera.target, camera.rotation,
                      1.f + fmodf(camera.zoom, 1.f)};
}

// Initialize 2D mode with custom camera (2D)
static void BeginModeTile(TileCamera camera) {
    rlDrawRenderBatchActive(); // Update and draw internal render batch

    rlLoadIdentity(); // Reset current matrix (modelview)

    // Apply 2d camera transformation to modelview
    Camera2D camera_2d = Camera2DFromTileCamera(camera);
    // assert(camera_2d.zoom >= 5.f);
    // assert(camera_2d.zoom < 6.f);
    rlMultMatrixf(MatrixToFloat(GetCameraMatrix2D(camera_2d)));
}

// End 2D mode with custom camera
static void EndModeTile(void) { EndMode2D(); }

static TileCamera ResetTileCamera(Screen screen, S32 tile_size) {
    return (TileCamera){
        .zoom =
            1.0f, // speicfies the zoom level. Is in the range (0; +infinity).
                  // the non integer part is used to interpolate between the tile sizes
                  // meaning it zooms the camera until the next tile size is hit.
        .rotation = 0.0f,
        .offset = (Vector2){(float)screen.width / 2.0f, (float)screen.height / 2.0f},
        .target.x = (F32)tile_size / 2.f,
        .target.y = (F32)tile_size / 2.f,
    };
}

static void UpdateTileCameraPos(TileCamera *tile_camera, Screen screen, S32 tile_size) {

    Camera2D camera_2d = Camera2DFromTileCamera(*tile_camera);

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
        if (zoom_level_old < zoom_level_new) {
            tile_camera->target = Vector2Scale(mouseWorldPos, 2.f);
            tile_camera->target =
                Vector2Subtract(tile_camera->target, (Vector2){0.f, (F32)tile_size});
        } else if (zoom_level_old > zoom_level_new) {
            tile_camera->target = Vector2Scale(mouseWorldPos, 0.5f);
            tile_camera->target =
                Vector2Add(tile_camera->target, (Vector2){0.f, (F32)tile_size / 2.f});
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
        *tile_camera = ResetTileCamera(screen, tile_size);
    }
}

static Matrix ModelTransformFromCoords(VectorTileCoordinate coords,
                                       U32 units_per_tile_max) {
    const F32 world_pixel_size = (F32)units_per_tile_max * exp2f((F32)coords.level);
    const F32 tile_world_size = world_pixel_size / exp2f((F32)coords.level);
    const Matrix tile_position = MatrixTranslate((F32)coords.col * tile_world_size,
                                                 (F32)coords.row * tile_world_size, 0.0f);
    return tile_position;
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

void VtpkDisplayFile(const char *filename, Screen screen) {
    Temp_Arena_Memory scratch = GetScratch();

    VTPK_RenderOptions render_options = {0};
    VtpkFile *vtpk_file = VtpkParseFile(scratch.arena, filename);
    vtpk_file->bounding_box = (AABB){min_S32, min_S32, max_S32, max_S32};

    const S32 tile_size = 512;
    const S32 tile_units_max = 512;
    TileCamera camera = ResetTileCamera(screen, tile_size);

    const S32 draw_cache_size = DRAW_CACHE_SIZE;
    DrawCache draw_cache =
        (DrawCache){VectorTileHandleArrayNew(scratch.arena, draw_cache_size),
                    VectorTileHandleArrayNew(scratch.arena, draw_cache_size)};
    Material material = LoadMaterialDefault();
    material.maps[MATERIAL_MAP_DIFFUSE].color = BLUE;

    SetTargetFPS(30); // NOTE: for now
    Color colors[13] = {RED,  GOLD,   LIME,  BLUE,    VIOLET, BROWN, LIGHTGRAY,
                        PINK, YELLOW, GREEN, SKYBLUE, PURPLE, BEIGE};
    S32 current_color = 0;
    while (!WindowShouldClose()) // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------
        UpdateTileCameraPos(&camera, screen, tile_size);

        //----------------------------------------------------------------------------------
        // Draw
        //----------------------------------------------------------------------------------

        Vector3 mesh_pos = {0};

        BeginDrawing();

        ClearBackground(RAYWHITE);
        BeginModeTile(camera);
        // Camera2D *c = (Camera2D *)&camera;
        // BeginMode2D(*c);
        if (render_options.show_grid) {
            DrawWorldGrid(Camera2DFromTileCamera(camera), screen, (F32)tile_size,
                          LIGHTGRAY, RED);
        }
        {
            // DrawRectangle(0, 512, tile_size, tile_size, RED);
            VectorTileHandleSlice tiles = MeshesFromBoundingBox(
                vtpk_file, &draw_cache, ClampBot((S32)camera.zoom, 0));
            current_color = 0;
            for (S32 i = 0; i < tiles.count; i += 1) {
                VectorTileHandle tile = tiles.v[i];
                assert(tile.status == DATA_PRESENT);
                const Matrix transform =
                    ModelTransformFromCoords(tile.coordinate, tile_units_max);
                rlPushMatrix();
                rlLoadIdentity();
                rlMultMatrixf(MatrixToFloat(transform));
                {
                    for (S32 j = 0; j < tile.gpu_data.meshes.count; j += 1) {
                        material.maps[MATERIAL_MAP_DIFFUSE].color = colors[current_color];
                        current_color = (current_color + 1) % 13;
                        mesh_pos.x = transform.m12;
                        mesh_pos.y = transform.m13;
                        mesh_pos.z = transform.m14;
                        DrawMesh(tile.gpu_data.meshes.v[j], material, MatrixIdentity());
                        if (render_options.show_bounding_box) {
                            BoundingBox bbox =
                                GetMeshBoundingBox(tile.gpu_data.meshes.v[j]);
                            DrawRectangleLines((S32)bbox.min.x, (S32)bbox.min.y,
                                               (S32)(bbox.max.x - bbox.min.x),
                                               (S32)(bbox.max.y - bbox.min.y), GREEN);
                            DrawTextEx(
                                GetFontDefault(),
                                TextFormat("[ROW: %d COL: %d LVL: %d]",
                                           tile.coordinate.row, tile.coordinate.col,
                                           tile.coordinate.level),
                                (Vector2){bbox.min.x + 10, bbox.min.y + 10}, 10, 1, RED);
                            Vector2 screen_pos_min =
                                GetWorldToScreen2D((Vector2){bbox.min.x, bbox.min.y},
                                                   Camera2DFromTileCamera(camera));
                            TraceLog(LOG_INFO, "bbox min screen coords: [%f, %f]",
                                     screen_pos_min.x, screen_pos_min.y);
                            Vector2 screen_pos_max =
                                GetWorldToScreen2D((Vector2){bbox.max.x, bbox.max.y},
                                                   Camera2DFromTileCamera(camera));
                            TraceLog(LOG_INFO, "bbox max screen coords: [%f, %f]",
                                     screen_pos_max.x, screen_pos_max.y);
                        }
                    }
                }
                rlPopMatrix();

                // for (S32 j = 0; j < tile.gpu_data.meshes.count; j += 1) {
                //     DrawMesh(tile.gpu_data.meshes.v[j], material, transform);
                // }
                // for (S32 j = 0; j < tiles.v[i].gpu_data.textures.count; j += 1) {
                //    DrawTextureV(tiles.v[i].gpu_data.textures.v[j].texture,
                //                 (Vector2){0, 0}, WHITE);
                //}
            }
        }
        EndModeTile();
        //  --------------------- HUD -------------------------
        DrawText(TextFormat("CURRENT ZOOM: %03.04f", camera.zoom), 640, 10, 20, RED);
        DrawText(TextFormat("CAMERA TARGET: [%03.04f, %03.04f]", camera.target.x,
                            camera.target.y),
                 640, 40, 20, RED);
        DrawFPS(640, 70);
        Vector2 mouseWorldPos =
            GetScreenToWorld2D(GetMousePosition(), Camera2DFromTileCamera(camera));
        DrawText(TextFormat("MOUSE POS : [%03.04f, %03.04f]", mouseWorldPos.x,
                            mouseWorldPos.y),
                 100, 10, 20, RED);
        DrawText(TextFormat("MESH POS: [%03.04f, %03.04f]", mesh_pos.x, mesh_pos.y), 640,
                 100, 20, RED);
        EndDrawing();
        //----------------------------------------------------------------------------------
    }
}
