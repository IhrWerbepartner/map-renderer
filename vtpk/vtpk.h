#include "../arena.c"
#include "../base.h"
#include "raymath.h"
#include "vtpk_reader.h"
#include <assert.h>
#include <math.h>
#include <raylib.h>
#include <stdbool.h>
#include <stdio.h>

#define DRAW_CACHE_SIZE (1024) // TODO: fine tune

typedef struct DrawCache DrawCache;
struct DrawCache {
    VectorTileHandleArray front_buffer;
    VectorTileHandleArray back_buffer;
};

static void PrintS32Slice(S32Slice s) {
    fprintf(stderr, "[");
    for (S32 i = 0; i < s.count; i += 1) {
        if (i > 0) {
            fprintf(stderr, ", ");
        }
        fprintf(stderr, "%d", s.v[i]);
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
        PrintS32Slice(S32SliceFromArray(&visible_tiles));
#endif
        // TODO: it would be cool to get these indices in a sorted order
        // this makes intersecting them much cheaper.
        // lets see if we can get this invariant enforced in the quad tree!!
        const S32Slice missing_tile_indices = NonCachedTileIndices(
            arena_auto_close_latch.scratch.arena, S32SliceFromArray(&visible_tiles),
            VectorTileHandleSliceFromArray(&draw_cache->back_buffer));
#ifdef DEBUG
        fprintf(stderr, "missing_tiles: ");
        PrintS32Slice(missing_tile_indices);
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

// TODO: figure out how zooming works
static S32 ZoomLevelFromCamera(Camera2D camera) {
    return (S32)Clamp(logf(camera.zoom) - 1.f, 0.f, 16.f);
}

static void VtpkUpdateCameraPos(Camera2D *camera, Screen screen) {
    // Translate based on mouse right click
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        Vector2 delta = GetMouseDelta();
        delta = Vector2Scale(delta, -1.0f / camera->zoom);
        camera->target = Vector2Add(camera->target, delta);
    }

    // Zoom based on mouse wheel
    F32 wheel = GetMouseWheelMove();
    if (wheel != 0) {
        // Get the world point that is under the mouse
        Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), *camera);

        // Set the offset to where the mouse is
        camera->offset = GetMousePosition();

        // Set the target to match, so that the camera maps the world space
        // point under the cursor to the screen space point under the cursor at
        // any zoom
        camera->target = mouseWorldPos;

        // Zoom increment
        // Uses log scaling to provide consistent zoom speed
        F32 scale = 0.2f * wheel;
        camera->zoom = Clamp(expf(logf(camera->zoom) + scale), 0.001f, 1000000.0f);
    }

    // Camera reset (zoom and rotation)
    if (IsKeyPressed(KEY_R)) {
        camera->zoom = 4.0f;
        camera->rotation = 0.0f;
        camera->offset =
            (Vector2){(float)screen.width / 2.0f, (float)screen.height / 2.0f};
        camera->target.x = 0;
        camera->target.y = 0;
    }
}

void VtpkDisplayFile(const char *filename, Screen screen) {
    Temp_Arena_Memory scratch = GetScratch();

    VtpkFile *vtpk_file = VtpkParseFile(scratch.arena, filename);
    vtpk_file->bounding_box = (AABB){min_S32, min_S32, max_S32, max_S32};

    Camera2D camera = {
        .offset = {(float)screen.width / 2.0f, (float)screen.height / 2.0f},
        .rotation = 0.0f,
        .zoom = 4.0f,
        .target = {0, 0}};

    const S32 draw_cache_size = DRAW_CACHE_SIZE;
    DrawCache draw_cache =
        (DrawCache){VectorTileHandleArrayNew(scratch.arena, draw_cache_size),
                    VectorTileHandleArrayNew(scratch.arena, draw_cache_size)};
    Material material = LoadMaterialDefault();
    material.maps[MATERIAL_MAP_DIFFUSE].color = BLUE;

    SetTargetFPS(5); // NOTE: for now

    while (!WindowShouldClose()) // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------
        VtpkUpdateCameraPos(&camera, screen);

        //----------------------------------------------------------------------------------
        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

        ClearBackground(RAYWHITE);

        BeginMode2D(camera);
        {
            VectorTileHandleSlice gpu_data = MeshesFromBoundingBox(
                vtpk_file, &draw_cache, ZoomLevelFromCamera(camera));
            for (S32 i = 0; i < gpu_data.count; i += 1) {
                assert(gpu_data.v[i].status == DATA_PRESENT);
                for (S32 j = 0; j < gpu_data.v[i].gpu_data.meshes.count; j += 1) {
                    DrawMesh(gpu_data.v[i].gpu_data.meshes.v[j], material,
                             MatrixIdentity());
                }
                for (S32 j = 0; j < gpu_data.v[i].gpu_data.textures.count; j += 1) {
                    DrawTextureV(gpu_data.v[i].gpu_data.textures.v[j].texture,
                                 (Vector2){0, 0}, WHITE);
                }
            }
        }
        EndMode2D();
        // --------------------- HUD -------------------------
        DrawText(TextFormat("CURRENT ZOOM: %03.04f", camera.zoom), 640, 10, 20, RED);
        DrawText(TextFormat("CAMERA TARGET: [%03.04f, %03.04f]", camera.target.x,
                            camera.target.y),
                 640, 40, 20, RED);
        DrawFPS(640, 70);
        Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), camera);
        DrawText(TextFormat("MOUSE POS : [%03.04f, %03.04f]", mouseWorldPos.x,
                            mouseWorldPos.y),
                 100, 10, 20, RED);
        EndDrawing();
        //----------------------------------------------------------------------------------
    }
}
