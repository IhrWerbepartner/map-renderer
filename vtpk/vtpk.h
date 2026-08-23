#include "../arena.c"
#include "../base.h"
#include "raymath.h"
#include "vtpk_reader.h"
#include <assert.h>
#include <raylib.h>

#define DRAW_CACHE_SIZE (1024) // TODO: fine tune

typedef struct DrawCache DrawCache;
struct DrawCache {
    VectorTileHandleArray front_buffer;
    VectorTileHandleArray back_buffer;
};

static S32Slice RemoveAllCachedTileIndices(const S32Slice visible_tiles,
                                           VectorTileHandleSlice cached_tiles) {
    // TODO: implement
}

// copy tiles within AABB to back buffer and request missing ones fromt the
// tilecache. finally swap back and front buffer.
static VectorTileHandleSlice MeshesFromBoundingBox(VtpkFile *vtpk_file, AABB bounding_box,
                                                   DrawCache *draw_cache,
                                                   S32 zoom_level) {
    // clear back buffer (becomes new front)
    VectorTileHandleArrayReset(&draw_cache->back_buffer);
    for (S32 i = 0; i < draw_cache->front_buffer.count; i += 1) {
        const VectorTileHandle handle = draw_cache->front_buffer.d[i];
        if (handle.coordinate.level == zoom_level &&
            AABBContains(bounding_box,
                         vtpk_file->quad_tree.d[handle.quad_tree_node].tile.coordinate)) {
            VectorTileHandleArrayPush(&draw_cache->back_buffer, handle);
        }
    }

    ScratchArena() {
        S32Array visible_tiles =
            S32ArrayNew(arena_auto_close_latch.scratch.arena, DRAW_CACHE_SIZE);
        QuadTreeFind(QuadTreeNodeSliceFromArray(&vtpk_file->quad_tree),
                     vtpk_file->root_node, bounding_box, zoom_level, &visible_tiles);
        // TODO: it would be cool to get these indices in a sorted order
        // this makes intersecting them much cheaper.
        // lets see if we can get this invariant enforced in the quad tree!!
        const S32Slice missing_tile_indices = RemoveAllCachedTileIndices(
            S32SliceFromArray(&visible_tiles),
            VectorTileHandleSliceFromArray(&draw_cache->back_buffer));
        VectorTileHandlesFromFile(vtpk_file, missing_tile_indices);
        for (S32 i = 0; i < missing_tile_indices.count; i += 1) {
            const VectorTileHandle handle = vtpk_file->quad_tree.d[i].tile;
            assert(handle.status == DATA_PRESENT);
            VectorTileHandleArrayPush(&draw_cache->back_buffer, handle);
        }
    }

    // swap back and front buffers
    Swap(VectorTileHandleArray, draw_cache->front_buffer, draw_cache->back_buffer);
    return VectorTileHandleSliceFromArray(&draw_cache->front_buffer);
}

static S32 ZoomLevelFromCamera(Camera2D camera) {
    // TODO:
}

void VtpkDisplayFile(char *filename, Screen screen) {
    Temp_Arena_Memory scratch = GetScratch();

    VtpkFile *vtpk_file = VtpkParseFile(scratch.arena, filename);
    AABB bounding_box = vtpk_file->bounding_box;

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

    while (!WindowShouldClose()) // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------
        // UpdateCameraPos(&camera, screen);

        //----------------------------------------------------------------------------------
        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

        ClearBackground(RAYWHITE);

        BeginMode2D(camera);
        {
            VectorTileHandleSlice gpu_data = MeshesFromBoundingBox(
                vtpk_file, bounding_box, &draw_cache, ZoomLevelFromCamera(camera));
            for (S32 i = 0; i < gpu_data.count; i += 1) {
                assert(gpu_data.v[i].status == DATA_PRESENT);
                for (S32 j = 0; j < gpu_data.v[i].gpu_data.meshes.count; j += 1) {
                    DrawMesh(gpu_data.v[i].gpu_data.meshes.v[j], material,
                             MatrixIdentity());
                }
            }
        }
        EndMode2D();
        EndDrawing();
        //----------------------------------------------------------------------------------
    }
}
