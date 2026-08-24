#pragma once

#include "../arena.c"
#include "../base.h"
#include "../vendor/sort_r.h"
#include <math.h>
#include <stddef.h>

// TODO: figure out invariants to put as validation functions.

typedef struct EarcutPolygon EarcutPolygon;
struct EarcutPolygon {
    Coord2Slice coords;
    RangeSlice contours;
};

typedef struct ZOrderInfo ZOrderInfo;
struct ZOrderInfo {
    bool hashing; // if false we do no hashing and the other fields are irrelevant
    F64 minX, maxX;
    F64 minY, maxY;
    F64 inv_size;
};

typedef struct Node Node;
struct Node {
    // i is a (bits(N)-1)-wide field packed alongside the 1-bit steiner flag;
    // mask index to that width so it fits without a narrowing warning (a no-op
    // for any real vertex index).
    F64 x;
    F64 y;

    // previous and next vertice nodes in a polygon ring
    S32 prev;
    S32 next;
    // z-order curve value
    S32 z;
    S32 vertex; // lowest bit is used for the steiner flag
    S32 prevZ;
    S32 nextZ;
    bool steiner;
#ifdef DEBUG
#ifdef DEBUG_EARCUT
    bool removed;
#endif
#endif
};
DeclFixedArray(NodeArray, Node);

typedef struct BoundedTriangle BoundedTriangle;
struct BoundedTriangle {
    F64 ax, ay;
    F64 bx, by;
    F64 cx, cy;
    // triangle bounding box, used to cheaply reject most candidate points
    // before the full point-in-triangle test (which is 6 multiplies)
    F64 minX, minY, maxX, maxY;
};

typedef struct BlockBoundingBox BlockBoundingBox;
struct BlockBoundingBox {
    F64 minX, minY, maxX, maxY;
};
DeclFixedArray(BlockBoundingBoxArray, BlockBoundingBox);

static inline void NewBoundedTriangle(BoundedTriangle *t, const NodeSlice nodes,
                                      const S32 a, const S32 b, const S32 c) {
    t->ax = nodes.v[a].x;
    t->ay = nodes.v[a].y;
    t->bx = nodes.v[b].x;
    t->by = nodes.v[b].y;
    t->cx = nodes.v[c].x;
    t->cy = nodes.v[c].y;
    t->minX = Min(t->ax, Min(t->bx, t->cx));
    t->minY = Min(t->ay, Min(t->by, t->cy));
    t->maxX = Max(t->ax, Max(t->bx, t->cx));
    t->maxY = Max(t->ay, Max(t->by, t->cy));
}

static inline bool inBBox(BoundedTriangle t, F64 px, F64 py) {
    return px >= t.minX && px <= t.maxX && py >= t.minY && py <= t.maxY;
}

static inline bool containsPoint(BoundedTriangle t, F64 px, F64 py) {
    return (t.cx - px) * (t.ay - py) >= (t.ax - px) * (t.cy - py) &&
           (t.ax - px) * (t.by - py) >= (t.bx - px) * (t.ay - py) &&
           (t.bx - px) * (t.cy - py) >= (t.cx - px) * (t.by - py);
}

// as containsPoint, but false when the point coincides with the triangle's
// first vertex (a)
static inline bool containsPointExceptFirst(BoundedTriangle t, F64 px, F64 py) {
    return !(t.ax == px && t.ay == py) && containsPoint(t, px, py);
}

static S32 LinkedList(NodeArray *nodes, Coord2Slice coords, Range contour,
                      bool clockwise);
static bool IsEar(NodeSlice nodes, S32 ear);
static bool IsEarHashed(NodeSlice nodes, ZOrderInfo bounds, S32 ear);
static S32 cureLocalIntersections(NodeSlice nodes, S32 start, TriangleArray *triangles);
internal void SplitEarcut(NodeArray *nodes, ZOrderInfo bounds, S32 start,
                          TriangleArray *triangles);
static F64 Area(NodeSlice nodes, S32 p, S32 q, S32 r);

static S32 eliminateHoles(EarcutPolygon polygon, NodeArray *nodes, S32 outerNode);
static S32 eliminateHole(BlockBoundingBoxArray *blocks, S32Array *block_head,
                         S32Array *block_stop, NodeArray *nodes, S32 hole, S32 outerNode);
static S32 findHoleBridge(BlockBoundingBoxSlice blocks, S32Slice block_head,
                          S32Slice block_stop, NodeSlice nodes, S32 hole, S32 outerNode);
static void indexSegment(NodeSlice nodes, BlockBoundingBoxArray *blocks,
                         S32Array *block_head, S32Array *block_stop, S32 head, S32 stop);
static void growBlock(NodeSlice nodes, BlockBoundingBoxSlice blocks, S32 head,
                      S32 tail_index);
static S32 liveBlockHead(NodeSlice nodes, S32Slice block_head, S32 b);
static S32 liveBlockStop(NodeSlice nodes, S32Slice block_stop, S32 b);
static bool sectorContainsSector(NodeSlice nodes, S32 m, S32 p);
static void indexCurve(NodeSlice nodes, ZOrderInfo bounds, S32 start);
static S32 sortLinked(NodeSlice nodes, S32 list);
static S32 zOrder(ZOrderInfo bounds, F64 x_, F64 y_);
static S32 getLeftmost(NodeSlice nodes, S32 start);
static bool pointInTriangle(F64 ax, F64 ay, F64 bx, F64 by, F64 cx, F64 cy, F64 px,
                            F64 py);
static bool isValidDiagonal(NodeSlice nodes, S32 a, S32 b);
static bool equals(NodeSlice nodes, S32 p1, S32 p2);
static bool intersects(NodeSlice nodes, S32 p1, S32 q1, S32 p2, S32 q2,
                       bool includeBoundary); // true by default
static bool onSegment(NodeSlice nodes, S32 p, S32 q, S32 r);
static bool intersectsPolygon(NodeSlice nodes, S32 a, S32 b);
static bool locallyInside(NodeSlice nodes, S32 a, S32 b);
static bool middleInside(NodeSlice nodes, S32 a, S32 b);
static S32 splitPolygon(NodeArray *nodes, S32 a, S32 b);
static void RemoveNode(NodeSlice nodes, S32 p);
static void RemoveNodeAndUpdateIndex(NodeSlice nodes, BlockBoundingBoxSlice blocks,
                                     S32 p);
internal S32 insertNode(NodeArray *nodes, S32 vertex, Coord2 pt, S32 last);

// --------------- VALIDATION FUNCTIONS --------------
#ifdef DEBUG
#ifdef DEBUG_EARCUT
static void ValidateNodeStructure(NodeSlice nodes);
static void ValidateZOrderCurve(NodeSlice nodes);

static void PrintLinkedList(const NodeSlice nodes, const S32 start) {
    fprintf(stderr, "%d(%d) -> ", start, nodes.v[start].vertex);
    S32 n = nodes.v[start].next;
    do {
        fprintf(stderr, "%d(%d) -> ", n, nodes.v[n].vertex);
        n = nodes.v[n].next;
    } while (n != start);
    fprintf(stderr, "%d(%d)\n", n, nodes.v[n].vertex);
}
#endif
#endif

static bool isContourClockwise(const Coord2Slice coords) {
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
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x) > 0.f;
}

// create a circular doubly linked list from polygon points in the specified
// winding order
static S32 LinkedList(NodeArray *nodes, const Coord2Slice coords, const Range contour,
                      const bool clockwise) {
#ifdef DEBUG
#ifdef DEBUG_EARCUT
    ValidateNodeStructure(NodeSliceFromArray(nodes));
#endif
#endif

    // calculate original winding order of a polygon ring
    // F64 sum = 0;
    // for (S32 i = contour.min, j = contour.min + contour.count - 1;
    //     i < contour.count; j = i++) {
    //  sum += (coords.v[j].x - coords.v[i].x) * (coords.v[i].y + coords.v[j].y);
    //}

    // link points into circular doubly-linked list in the specified winding order
    const S32 already_inserted_vertices = contour.min;
    S32 last = 0;
    if (clockwise !=
        isContourClockwise((Coord2Slice){coords.v + contour.min, contour.count})) {
        for (S32 i = 0; i < contour.count; i += 1) {
            const S32 vertex = already_inserted_vertices + i;
            last = insertNode(nodes, vertex, coords.v[vertex], last);
        }
    } else {
        for (S32 i = contour.count; i > 0; i -= 1) {
            const S32 vertex = already_inserted_vertices + i - 1;
            last = insertNode(nodes, vertex, coords.v[vertex], last);
        }
    }

    if (last && equals(NodeSliceFromArray(nodes), last, nodes->d[last].next)) {
        RemoveNode(NodeSliceFromArray(nodes), last);
        last = nodes->d[last].next;
    }
#ifdef DEBUG
#ifdef DEBUG_EARCUT
    ValidateNodeStructure(NodeSliceFromArray(nodes));
#endif
#endif
    return last;
}

typedef struct FilterPointsResult FilterPointsResult;
struct FilterPointsResult {
    S32 ear;
    bool filtered_out;
};

// Remove collinear or coincident points; removability depends only on a node's
// immediate neighbors, so we sweep forward and re-check the predecessor after
// each removal. With no `end` we sweep the whole ring, lapping until nothing is
// removable (the fixpoint the clipper needs). With an explicit `end` we heal
// only the dirty window around a bridge/diagonal cut, stopping at `end` rather
// than lapping — O(window) instead of O(ring).

internal FilterPointsResult filterPoints(NodeSlice nodes, S32 start, S32 end) {
#ifdef DEBUG
#ifdef DEBUG_EARCUT
    ValidateNodeStructure(nodes);
    ValidateZOrderCurve(nodes);
#endif
#endif
    if (!start)
        return (FilterPointsResult){start, false};
    const bool full = !end;
    bool filtered_out = false;
    if (full)
        end = start;

    S32 p = start;
    bool again;
    do {
        again = false;
        if (p != nodes.v[p].next && !nodes.v[p].steiner &&
            (equals(nodes, p, nodes.v[p].next) ||
             Area(nodes, nodes.v[p].prev, p, nodes.v[p].next) == 0)) {
            if (full || p == end)
                end = nodes.v[p].prev; // pull the stop bound back past the removal
            filtered_out = true;
            RemoveNode(nodes, p);
            p = nodes.v[p].prev; // re-check the predecessor
            again = true;
        } else if (full || p != end) {
            p = nodes.v[p].next;
            again = !full; // local heal: keep looping until the sweep reaches end
        }
    } while (again || p != end);
#ifdef DEBUG
#ifdef DEBUG_EARCUT
    ValidateNodeStructure(nodes);
    ValidateZOrderCurve(nodes);
#endif
#endif

    return (FilterPointsResult){end, filtered_out};
}

internal FilterPointsResult filterPointsAndUpdateIndex(NodeSlice nodes,
                                                       BlockBoundingBoxSlice blocks,
                                                       S32 start, S32 end) {
#ifdef DEBUG
#ifdef DEBUG_EARCUT
    ValidateNodeStructure(nodes);
#endif
#endif
    if (!start)
        return (FilterPointsResult){start, false};
    const bool full = !end;
    bool filtered_out = false;
    if (full)
        end = start;

    S32 p = start;
    bool again;
    do {
        again = false;
        if (p != nodes.v[p].next && !nodes.v[p].steiner &&
            (equals(nodes, p, nodes.v[p].next) ||
             Area(nodes, nodes.v[p].prev, p, nodes.v[p].next) == 0)) {
            if (full || p == end)
                end = nodes.v[p].prev; // pull the stop bound back past the removal
            filtered_out = true;
            RemoveNodeAndUpdateIndex(nodes, blocks, p);
            p = nodes.v[p].prev; // re-check the predecessor
            again = true;
        } else if (full || p != end) {
            p = nodes.v[p].next;
            again = !full; // local heal: keep looping until the sweep reaches end
        }
    } while (again || p != end);

#ifdef DEBUG
#ifdef DEBUG_EARCUT
    ValidateNodeStructure(nodes);
#endif
#endif
    return (FilterPointsResult){end, filtered_out};
}

// main ear slicing loop which triangulates a polygon (given as a linked list)

internal void EarcutLinked(NodeArray *nodes, ZOrderInfo bounds, S32 ear,
                           TriangleArray *triangles) {
#ifdef DEBUG
#ifdef DEBUG_EARCUT
    ValidateNodeStructure(NodeSliceFromArray(nodes));
#endif
#endif
    if (!ear)
        return;
    const NodeSlice node_slice = NodeSliceFromArray(nodes);
    if (bounds.hashing) {
        indexCurve(node_slice, bounds, ear);
    }

    S32 stop = ear;
    bool cured = false;

    // iterate through ears, slicing them one by one
    while (node_slice.v[ear].prev != node_slice.v[ear].next) {
        const S32 prev = node_slice.v[ear].prev;
        const S32 next = node_slice.v[ear].next;

        // reflex check is hoisted here to avoid constructing the Triangle for
        // reflex corners
        if (Area(node_slice, prev, ear, next) < 0 &&
            (bounds.hashing ? IsEarHashed(node_slice, bounds, ear)
                            : IsEar(node_slice, ear))) {
            // cut off the triangle
            ASSERT(node_slice.v[prev].vertex, "vertex prev is 0");
            ASSERT(node_slice.v[ear].vertex, "vertex ear is 0");
            ASSERT(node_slice.v[next].vertex, "vertex next is 0");
            TriangleArrayPush(triangles, (Triangle){.a = node_slice.v[prev].vertex,
                                                    .b = node_slice.v[ear].vertex,
                                                    .c = node_slice.v[next].vertex});

            RemoveNode(node_slice, ear);
            ear = next;
            stop = next;
            continue;
        }

        ear = next;

        // if we looped through the whole remaining polygon and can't find any more
        // ears
        if (ear == stop) {
            // try filtering collinear/coincident points and slicing again — repeat as
            // long as filtering actually removes nodes, since each removal can expose
            // new ears
            const FilterPointsResult res = filterPoints(node_slice, ear, 0);
            ear = res.ear;
            if (res.filtered_out) {
                stop = ear;
                continue;
            }

            // filtering is exhausted: cure small local self-intersections once, then
            // retry
            if (!cured) {
                ear = cureLocalIntersections(node_slice, ear, triangles);
                stop = ear;
                cured = true;
                continue;
            }

            // as a last resort, try splitting the remaining polygon into two
            SplitEarcut(nodes, bounds, ear, triangles);
            break;
        }
    }
#ifdef DEBUG
#ifdef DEBUG_EARCUT
    ValidateNodeStructure(NodeSliceFromArray(nodes));
    ValidateZOrderCurve(NodeSliceFromArray(nodes));
#endif
#endif
}

// check whether a polygon node forms a valid ear with adjacent nodes

bool IsEar(const NodeSlice nodes, const S32 ear) {
#ifdef DEBUG
#ifdef DEBUG_EARCUT
    ValidateNodeStructure(nodes);
#endif
#endif
    const S32 a = nodes.v[ear].prev;
    const S32 b = ear;
    const S32 c = nodes.v[ear].next;

    // reflex check is hoisted into the earcutLinked caller
    BoundedTriangle t = {0};
    NewBoundedTriangle(&t, nodes, a, b, c);

    // now make sure we don't have other points inside the potential ear
    S32 p = nodes.v[nodes.v[ear].next].next;

    while (p != nodes.v[ear].prev) {
        if (inBBox(t, nodes.v[p].x, nodes.v[p].y) &&
            containsPointExceptFirst(t, nodes.v[p].x, nodes.v[p].y) &&
            Area(nodes, nodes.v[p].prev, p, nodes.v[p].next) >= 0)
            return false;
        p = nodes.v[p].next;
    }

#ifdef DEBUG
#ifdef DEBUG_EARCUT
    ValidateNodeStructure(nodes);
#endif
#endif
    return true;
}

bool IsEarHashed(const NodeSlice nodes, const ZOrderInfo bounds, const S32 ear) {
#ifdef DEBUG
#ifdef DEBUG_EARCUT
    ValidateNodeStructure(nodes);
#endif
#endif
    const S32 a = nodes.v[ear].prev;
    const S32 b = ear;
    const S32 c = nodes.v[ear].next;

    // reflex check is hoisted into the earcutLinked caller
    BoundedTriangle t = {0};
    NewBoundedTriangle(&t, nodes, a, b, c);

    // z-order range for the current triangle bbox;
    const S32 minZ = zOrder(bounds, t.minX, t.minY);
    const S32 maxZ = zOrder(bounds, t.maxX, t.maxY);

    // first look for points inside the triangle in increasing z-order
    S32 p = nodes.v[ear].nextZ;

    while (p && nodes.v[p].z <= maxZ) {
        if (p != nodes.v[ear].next && inBBox(t, nodes.v[p].x, nodes.v[p].y) &&
            containsPointExceptFirst(t, nodes.v[p].x, nodes.v[p].y) &&
            Area(nodes, nodes.v[p].prev, p, nodes.v[p].next) >= 0)
            return false;
        p = nodes.v[p].nextZ;
    }

    // then look for points in decreasing z-order
    p = nodes.v[ear].prevZ;

    while (p && nodes.v[p].z >= minZ) {
        if (p != nodes.v[ear].next && inBBox(t, nodes.v[p].x, nodes.v[p].y) &&
            containsPointExceptFirst(t, nodes.v[p].x, nodes.v[p].y) &&
            Area(nodes, nodes.v[p].prev, p, nodes.v[p].next) >= 0)
            return false;
        p = nodes.v[p].prevZ;
    }

#ifdef DEBUG
#ifdef DEBUG_EARCUT
    ValidateNodeStructure(nodes);
#endif
#endif
    return true;
}

// go through all polygon nodes and cure small local self-intersections

S32 cureLocalIntersections(NodeSlice nodes, S32 start, TriangleArray *triangles) {
#ifdef DEBUG
#ifdef DEBUG_EARCUT
    ValidateNodeStructure(nodes);
    ValidateZOrderCurve(nodes);
#endif
#endif
    S32 p = start;
    bool cured = false;
    do {
        S32 a = nodes.v[p].prev;
        S32 b = nodes.v[nodes.v[p].next].next;

        // a self-intersection where edge (v[i-1],v[i]) intersects (v[i+1],v[i+2]);
        // includeBoundary=false so a mere collinear touch isn't treated as a
        // crossing
        if (intersects(nodes, a, p, nodes.v[p].next, b, false) &&
            locallyInside(nodes, a, b) && locallyInside(nodes, b, a)) {
            ASSERT(nodes.v[a].vertex, "vertex a is 0");
            ASSERT(nodes.v[p].vertex, "vertex p is 0");
            ASSERT(nodes.v[b].vertex, "vertex b is 0");
            TriangleArrayPush(triangles, (Triangle){.a = nodes.v[a].vertex,
                                                    .b = nodes.v[p].vertex,
                                                    .c = nodes.v[b].vertex});

            // remove two nodes involved
            RemoveNode(nodes, p);
            RemoveNode(nodes, nodes.v[p].next);

            p = start = b;
            cured = true;
        }
        p = nodes.v[p].next;
    } while (p != start);

    S32 res = p;
    if (cured) {
        res = filterPoints(nodes, p, 0).ear;
    }
#ifdef DEBUG
#ifdef DEBUG_EARCUT
    ValidateNodeStructure(nodes);
    ValidateZOrderCurve(nodes);
#endif
#endif
    return res;
}

// try splitting polygon into two and triangulate them independently

internal void SplitEarcut(NodeArray *nodes, const ZOrderInfo bounds, const S32 start,
                          TriangleArray *triangles) {
#ifdef DEBUG
#ifdef DEBUG_EARCUT
    ValidateNodeStructure(NodeSliceFromArray(nodes));
    ValidateZOrderCurve(NodeSliceFromArray(nodes));
#endif
#endif
    // look for a valid diagonal that divides the polygon into two
    NodeSlice node_slice = NodeSliceFromArray(nodes);
    S32 a = start;
    do {
        S32 b = nodes->d[nodes->d[a].next].next;
        while (b != nodes->d[a].prev) {
            if (nodes->d[a].vertex != nodes->d[b].vertex &&
                isValidDiagonal(node_slice, a, b)) {
                // split the polygon in two by the diagonal
                S32 c = splitPolygon(nodes, a, b);

                // filter collinear points around the cuts
                a = filterPoints(node_slice, a, nodes->d[a].next).ear;
                c = filterPoints(node_slice, c, nodes->d[c].next).ear;

                // run earcut on each half
                EarcutLinked(nodes, bounds, a, triangles);
                EarcutLinked(nodes, bounds, c, triangles);
                return;
            }
            b = nodes->d[b].next;
        }
        a = nodes->d[a].next;
    } while (a != start);
#ifdef DEBUG
#ifdef DEBUG_EARCUT
    ValidateNodeStructure(NodeSliceFromArray(nodes));
    ValidateZOrderCurve(NodeSliceFromArray(nodes));
#endif
#endif
}

static S32 hole_queue_comparator(const void *_a, const void *_b, void *_nodes) {
    const S32 a = *(const S32 *)_a;
    const S32 b = *(const S32 *)_b;
    Node *nodes = (Node *)_nodes;

    if (nodes[a].x != nodes[b].x) {
        return nodes[a].x < nodes[b].x ? -1 : 1;
    }
    if (nodes[a].y != nodes[b].y)
        return nodes[a].y < nodes[b].y ? -1 : 1;
    const F64 adx = nodes[nodes[a].next].x - nodes[a].x;
    const F64 ady = nodes[nodes[a].next].y - nodes[a].y;

    const F64 bdx = nodes[nodes[b].next].x - nodes[b].x;
    const F64 bdy = nodes[nodes[b].next].y - nodes[b].y;

    const bool aDegenerate = adx == 0.f && ady == 0.f;
    const bool bDegenerate = bdx == 0.f && bdy == 0.f;
    if (aDegenerate != bDegenerate)
        return aDegenerate ? -1 : 1;
    return ady * bdx < bdy * adx ? -1 : 1;
}

// link every hole into the outer loop, producing a single-ring polygon without
// holes

S32 eliminateHoles(const EarcutPolygon polygon, NodeArray *nodes, S32 outerNode) {

    const Temp_Arena_Memory scratch = GetScratch();
    S32Array hole_queue = S32ArrayNew(scratch.arena, (polygon.coords.count * 3) / 2);
    NodeSlice node_slice = NodeSliceFromArray(nodes);

    for (S32 i = 1; i < polygon.contours.count; i += 1) {
        const S32 list = LinkedList(nodes, polygon.coords, polygon.contours.v[i], false);
        if (list) {
            if (list == nodes->d[list].next)
                nodes->d[list].steiner = true;
            S32ArrayPush(&hole_queue, getLeftmost(node_slice, list));
        }
    }
    // compareXYSlope: sort by x, then y, then slope. When two holes' leftmost
    // points coincide, the slope tiebreak makes the bridge land on the shared
    // vertex instead of bridging the wrong hole.
    ASSERT(hole_queue.count >= 0, "invalid count");
    qsort_r(hole_queue.d, (size_t)hole_queue.count, sizeof(hole_queue.d[0]),
            hole_queue_comparator, nodes->d);

    // upper bound: every input node indexed once, +2 bridge nodes per hole, plus
    // a partial trailing block per appended segment (outer ring + one per hole)
    const S32 K = 16;
    const S32 blocks_max =
        (nodes->count + 2 * hole_queue.count + K - 1) / K + hole_queue.count + 2;
    BlockBoundingBoxArray blocks = BlockBoundingBoxArrayNew(scratch.arena, blocks_max);
    S32Array block_head = S32ArrayNew(scratch.arena, blocks_max);
    S32Array block_stop = S32ArrayNew(scratch.arena, blocks_max);
    indexSegment(node_slice, &blocks, &block_head, &block_stop, outerNode, outerNode);

    // process holes from left to right; indexActive lets removeNode keep block
    // bboxes live as filterPoints heals edges during merges (see growBlock)
    for (S32 i = 0; i < hole_queue.count; i++) {
#ifdef DEBUG
#ifdef DEBUG_EARCUT
        PrintLinkedList(node_slice, outerNode);
#endif
#endif
        outerNode = eliminateHole(&blocks, &block_head, &block_stop, nodes,
                                  hole_queue.d[i], outerNode);
    }

    // collapse collinear/coincident points across the whole merged ring once
    // before clipping

    const S32 ear = filterPoints(node_slice, outerNode, 0).ear;
    temp_arena_memory_end(scratch);
    return ear;
}

// find a bridge between vertices that connects hole with an outer ring and and
// link it

S32 eliminateHole(BlockBoundingBoxArray *blocks, S32Array *block_head,
                  S32Array *block_stop, NodeArray *nodes, const S32 hole,
                  const S32 outerNode) {
    ASSERT(hole, "hole not valid node");
    NodeSlice node_slice = NodeSliceFromArray(nodes);
    BlockBoundingBoxSlice blocks_slice = BlockBoundingBoxSliceFromArray(blocks);
    const S32 bridge =
        findHoleBridge(blocks_slice, S32SliceFromArray(block_head),
                       S32SliceFromArray(block_stop), node_slice, hole, outerNode);
    if (!bridge) {
        return outerNode;
    }
    // ASSERT(isValidDiagonal(node_slice, bridge, hole), "proposed bridge %d <->
    // %d not a valid diagonal", bridge, hole);

    S32 bridgeReverse = splitPolygon(nodes, bridge, hole);

    // index the merged-in segment before filtering: in ring order the splice runs
    // bridge -> hole -> bridgeReverse -> bridge2 -> (bridge's old next), covering
    // the hole's edges and both new slit edges. filterPoints below only drops
    // collinear/coincident points, so these bboxes stay valid (conservative)
    // supersets.
    S32 bridge2 = nodes->d[bridgeReverse].next;
    indexSegment(node_slice, blocks, block_head, block_stop, bridge,
                 nodes->d[bridge2].next);

    // heal collinear/coincident points around the two new slit edges
    filterPointsAndUpdateIndex(node_slice, blocks_slice, bridgeReverse,
                               nodes->d[bridgeReverse].next);
    return filterPointsAndUpdateIndex(node_slice, blocks_slice, bridge,
                                      nodes->d[bridge].next)
        .ear;
}

// David Eberly's algorithm for finding a bridge between hole and outer polygon

S32 findHoleBridge(BlockBoundingBoxSlice blocks, S32Slice block_head, S32Slice block_stop,
                   NodeSlice nodes, S32 hole, S32 outerNode) {
#ifdef DEBUG
#ifdef DEBUG_EARCUT
    ValidateNodeStructure(nodes);
#endif
#endif
    S32 p = outerNode;
    const F64 hx = nodes.v[hole].x;
    const F64 hy = nodes.v[hole].y;
    F64 qx = min_F64;
    S32 bridge = 0;

    // find a segment intersected by a ray from the hole's leftmost Vertex to the
    // left; segment's endpoint with lesser x will be potential connection Vertex,
    // unless they intersect at a vertex, then choose the vertex
    if (equals(nodes, hole, p))
        return p;

    // scan blocks; skip any whose bbox can't hold a crossing that beats qx and
    // lies left of hx (the prune Morton order can't express — explicit per-axis
    // [minY,maxY]/[minX,maxX])
    for (S32 b = 0; b < blocks.count; b += 1) {
        if (hy < blocks.v[b].minY || hy > blocks.v[b].maxY || blocks.v[b].minX > hx ||
            blocks.v[b].maxX <= qx)
            continue;

        // ensure the walk's exclusive bound is live so we don't overrun into other
        // blocks
        const S32 stop = liveBlockStop(nodes, block_stop, b);
        p = liveBlockHead(nodes, block_head, b);
        do {
            if (nodes.v[nodes.v[p].prev].next ==
                p) { // skip nodes removed by filterPoints (stale in the index)
                if (equals(nodes, hole, nodes.v[p].next))
                    return nodes.v[p].next;
                else if (hy <= nodes.v[p].y && hy >= nodes.v[nodes.v[p].next].y &&
                         nodes.v[nodes.v[p].next].y != nodes.v[p].y) {
                    const F64 x =
                        nodes.v[p].x + (hy - nodes.v[p].y) *
                                           (nodes.v[nodes.v[p].next].x - nodes.v[p].x) /
                                           (nodes.v[nodes.v[p].next].y - nodes.v[p].y);
                    if (x <= hx && x > qx) {
                        qx = x;
                        bridge = nodes.v[p].x < nodes.v[nodes.v[p].next].x
                                     ? p
                                     : nodes.v[p].next;
                        if (x == hx)
                            return bridge; // hole touches outer segment; pick leftmost
                                           // endpoint
                    }
                }
            }
            p = nodes.v[p].next;
        } while (p != stop);
    }

    if (!bridge)
        return 0;

    // look for points inside the triangle of hole Vertex, segment intersection
    // and endpoint; if there are no points found, we have a valid connection;
    // otherwise choose the Vertex of the minimum angle with the ray as connection
    // Vertex

    const double mx = nodes.v[bridge].x;
    const double my = nodes.v[bridge].y;
    const double tminY = Min(hy, my); // the triangle's y span; x span is [mx, hx]
    const double tmaxY = Max(hy, my);
    double tanMin = max_F64;

    // scan the same blocks; skip any whose bbox can't overlap the triangle's
    // [mx,hx]x[tminY,tmaxY] box
    for (S32 b = 0; b < blocks.count; b += 1) {
        if (blocks.v[b].maxX < mx || blocks.v[b].minX > hx || blocks.v[b].maxY < tminY ||
            blocks.v[b].minY > tmaxY)
            continue;

        const S32 stop = liveBlockStop(nodes, block_stop, b);
        p = liveBlockHead(nodes, block_head, b);
        do {
            if (nodes.v[nodes.v[p].prev].next == p && hx >= nodes.v[p].x &&
                nodes.v[p].x >= mx && hx != nodes.v[p].x && // skip dead nodes
                pointInTriangle(hy < my ? hx : qx, hy, mx, my, hy < my ? qx : hx, hy,
                                nodes.v[p].x, nodes.v[p].y)) {
                const F64 tanCur =
                    fabs(hy - nodes.v[p].y) / (hx - nodes.v[p].x); // tangential

                // if hole point sits on p's horizontal edge (T-junction touch): the
                // bridge runs along that edge — locallyInside rejects it as collinear,
                // but it's valid
                if ((locallyInside(nodes, p, hole) ||
                     (nodes.v[p].y == hy && nodes.v[nodes.v[p].next].y == hy &&
                      nodes.v[nodes.v[p].next].x > hx)) &&
                    (tanCur < tanMin ||
                     (tanCur == tanMin && (nodes.v[p].x > nodes.v[bridge].x ||
                                           (nodes.v[p].x == nodes.v[bridge].x &&
                                            sectorContainsSector(nodes, bridge, p)))))) {
                    bridge = p;
                    tanMin = tanCur;
                }
            }
            p = nodes.v[p].next;
        } while (p != stop);
    }

#ifdef DEBUG
#ifdef DEBUG_EARCUT
    ValidateNodeStructure(nodes);
#endif
#endif
    return bridge;
}

// index the ring run head..stop (exclusive) as ceil(len / K) blocks; head ==
// stop means the whole ring. each block's bbox covers both endpoints of every
// edge it owns.

void indexSegment(NodeSlice nodes, BlockBoundingBoxArray *blocks, S32Array *block_head,
                  S32Array *block_stop, S32 head, S32 stop) {
#ifdef DEBUG
#ifdef DEBUG_EARCUT
    ValidateNodeStructure(nodes);
#endif
#endif
    S32 p = head;
    do {
        const S32 K = 16;
        const S32 block_index = S32ArrayPush(block_head, p);
        F64 bMinX = max_F64;
        F64 bMinY = max_F64;
        F64 bMaxX = min_F64;
        F64 bMaxY = min_F64;
        S32 k = 0;
        do {
            S32 c = nodes.v[p].next;    // edge p->c; bbox must bound both endpoints
            nodes.v[p].z = block_index; // reuse z as the owning block during
                                        // eliminateHoles (see growBlock)
            if (nodes.v[p].x < bMinX)
                bMinX = nodes.v[p].x;
            if (nodes.v[p].x > bMaxX)
                bMaxX = nodes.v[p].x;
            if (nodes.v[p].y < bMinY)
                bMinY = nodes.v[p].y;
            if (nodes.v[p].y > bMaxY)
                bMaxY = nodes.v[p].y;
            if (nodes.v[c].x < bMinX)
                bMinX = nodes.v[c].x;
            if (nodes.v[c].x > bMaxX)
                bMaxX = nodes.v[c].x;
            if (nodes.v[c].y < bMinY)
                bMinY = nodes.v[c].y;
            if (nodes.v[c].y > bMaxY)
                bMaxY = nodes.v[c].y;
            p = c;
        } while (++k < K && p != stop);
        S32ArrayPush(block_stop, p);
        BlockBoundingBoxArrayPush(blocks, (BlockBoundingBox){bMinX, bMinY, bMaxX, bMaxY});
    } while (p != stop);
#ifdef DEBUG
#ifdef DEBUG_EARCUT
    ValidateNodeStructure(nodes);
#endif
#endif
}

// when filterPoints heals an edge head->tail (removing the collinear node
// between them), the healed edge can extend past head's frozen block bbox if
// its old far endpoint lived in another block; grow head's block bbox to cover
// tail so the leftward-ray prune can't false-skip it.

void growBlock(NodeSlice nodes, BlockBoundingBoxSlice blocks, const S32 head,
               const S32 tail_index) {
    const S32 g = nodes.v[head].z;
    const Node tail = nodes.v[tail_index];
    if (tail.x < blocks.v[g].minX)
        blocks.v[g].minX = tail.x;
    if (tail.y < blocks.v[g].minY)
        blocks.v[g].minY = tail.y;
    if (tail.x > blocks.v[g].maxX)
        blocks.v[g].maxX = tail.x;
    if (tail.y > blocks.v[g].maxY)
        blocks.v[g].maxY = tail.y;
}

// the block's head node can be removed by filterPoints during merges; advance
// it to the next live node so the walk doesn't start on (and immediately
// terminate at) a dead node. For the single full-ring seed block (head == stop)
// the same forward advance keeps them equal, so the do-while still laps the
// whole ring instead of collapsing to an empty walk.

S32 liveBlockHead(const NodeSlice nodes, S32Slice block_head, const S32 b) {
    S32 head = block_head.v[b];
    while (nodes.v[nodes.v[head].prev].next != head)
        head = nodes.v[head].next;
    block_head.v[b] = head;
    return head;
}

S32 liveBlockStop(const NodeSlice nodes, S32Slice block_stop, const S32 b) {
    S32 stop = block_stop.v[b];
    while (nodes.v[nodes.v[stop].prev].next != stop)
        stop = nodes.v[stop].next;
    block_stop.v[b] = stop;
    return stop;
}

// whether sector in vertex m contains sector in vertex p in the same
// coordinates

bool sectorContainsSector(const NodeSlice nodes, const S32 m, const S32 p) {
    return Area(nodes, nodes.v[m].prev, m, nodes.v[p].prev) < 0 &&
           Area(nodes, nodes.v[p].next, m, nodes.v[m].next) < 0;
}

// interlink polygon nodes in z-order

void indexCurve(NodeSlice nodes, const ZOrderInfo bounds, const S32 start) {
    assert(start);
    S32 p = start;

    do {
        // always (re)compute: z may still hold a block index left over from
        // eliminateHoles
        // fprintf(stderr, "%d -> ", p);
        nodes.v[p].z = zOrder(bounds, nodes.v[p].x, nodes.v[p].y);
        nodes.v[p].prevZ = nodes.v[p].prev;
        nodes.v[p].nextZ = nodes.v[p].next;
        p = nodes.v[p].next;
    } while (p != start);
    // fprintf(stderr, "%d\n", p);

    ASSERT(nodes.v[p].prevZ, "nodes.v[p].prevZ == 0");
    nodes.v[nodes.v[p].prevZ].nextZ = 0;
    nodes.v[p].prevZ = 0;

#ifdef DEBUG
#ifdef DEBUG_EARCUT
    ValidateZOrderCurve(nodes);
#endif
#endif
    sortLinked(nodes, p);
#ifdef DEBUG
#ifdef DEBUG_EARCUT
    ValidateZOrderCurve(nodes);
#endif
#endif
}

internal S32 linked_sort_comparator(const void *_a, const void *_b, void *_nodes) {
    const S32 a = *(const S32 *)_a;
    const S32 b = *(const S32 *)_b;
    Node *nodes = (Node *)_nodes;
    return nodes[a].z < nodes[b].z ? -1 : 1;
}

// Sort the z-linked ring by z-order. Upstream earcut replaced its linked merge
// sort with an array sort (materialize node refs → sort → relink); in C++
// std::sort over a contiguous Node* buffer inlines the comparator fully and
// beats both a linked merge sort and a hand radix (measured on the MVT tiles
// fixture) — JS's rejection of native Array.sort does not transfer.

S32 sortLinked(NodeSlice nodes, const S32 list) {
#ifdef DEBUG
#ifdef DEBUG_EARCUT
    ValidateNodeStructure(nodes);
    ValidateZOrderCurve(nodes);
#endif
#endif
    assert(list);
    // list is a null-terminated nextZ chain (see indexCurve); walk it into the
    // scratch buffer
    Temp_Arena_Memory scratch = GetScratch();
    S32Array sort_buffer = S32ArrayNew(scratch.arena, nodes.count);
    for (S32 p = list; p; p = nodes.v[p].nextZ) {
        S32ArrayPush(&sort_buffer, p);
    }

    ASSERT(sort_buffer.count >= 0, "invalid count");
    sort_r(sort_buffer.d, (size_t)sort_buffer.count, sizeof(sort_buffer.d[0]),
           linked_sort_comparator, nodes.v);

    // relink in sorted order
    S32 prev = 0;
    for (S32 i = 0; i < sort_buffer.count; i += 1) {
        const S32 p = sort_buffer.d[i];
        nodes.v[p].prevZ = prev;
        if (prev) {
            nodes.v[prev].nextZ = p;
        }
        prev = p;
    }
    nodes.v[prev].nextZ = 0;
    S32 sort_buffer_first = sort_buffer.d[0];
    temp_arena_memory_end(scratch);
#ifdef DEBUG
#ifdef DEBUG_EARCUT
    ValidateNodeStructure(nodes);
    ValidateZOrderCurve(nodes);
    ASSERT(sort_buffer_first, "sort buffer first has to be a valid node");
#endif
#endif
    return sort_buffer_first;
}

// z-order of a Vertex given coords and size of the data bounding box

S32 zOrder(const ZOrderInfo bounds, const F64 x_, const F64 y_) {
    // coords are transformed into non-negative 15-bit integer range
    S32 x = (S32)((x_ - bounds.minX) * bounds.inv_size);
    S32 y = (S32)((y_ - bounds.minY) * bounds.inv_size);

    x = (x | (x << 8)) & 0x00FF00FF;
    x = (x | (x << 4)) & 0x0F0F0F0F;
    x = (x | (x << 2)) & 0x33333333;
    x = (x | (x << 1)) & 0x55555555;

    y = (y | (y << 8)) & 0x00FF00FF;
    y = (y | (y << 4)) & 0x0F0F0F0F;
    y = (y | (y << 2)) & 0x33333333;
    y = (y | (y << 1)) & 0x55555555;

    return x | (y << 1);
}

// find the leftmost node of a polygon ring

S32 getLeftmost(const NodeSlice nodes, const S32 start) {
    S32 p = start;
    S32 leftmost = start;
    do {
        if (nodes.v[p].x < nodes.v[leftmost].x ||
            (nodes.v[p].x == nodes.v[leftmost].x && nodes.v[p].y < nodes.v[leftmost].y)) {
            leftmost = p;
        }
        p = nodes.v[p].next;
    } while (p != start);

    return leftmost;
}

// check if a point lies within a convex triangle

bool pointInTriangle(const F64 ax, const F64 ay, const F64 bx, const F64 by, const F64 cx,
                     const F64 cy, const F64 px, const F64 py) {
    return (cx - px) * (ay - py) >= (ax - px) * (cy - py) &&
           (ax - px) * (by - py) >= (bx - px) * (ay - py) &&
           (bx - px) * (cy - py) >= (cx - px) * (by - py);
}

// check if a diagonal between two polygon nodes is valid (lies in polygon
// interior)

bool isValidDiagonal(const NodeSlice nodes, const S32 a, const S32 b) {
    ASSERT(a, "endpoint a invalid");
    ASSERT(b, "endpoint a invalid");
    // degenerate zero-length case
    const bool zeroLength = equals(nodes, a, b) &&
                            Area(nodes, nodes.v[a].prev, a, nodes.v[a].next) > 0.f &&
                            Area(nodes, nodes.v[b].prev, b, nodes.v[b].next) > 0.f;
    return nodes.v[nodes.v[a].next].vertex != nodes.v[b].vertex &&
           (zeroLength || (locallyInside(nodes, a, b) &&
                           locallyInside(nodes, b, a) && // locally visible
                           (Area(nodes, nodes.v[a].prev, a, nodes.v[b].prev) != 0.f ||
                            Area(nodes, a, nodes.v[b].prev, b) !=
                                0.f))) &&             // no opposite-facing sectors
           !intersectsPolygon(nodes, a, b) &&         // doesn't intersect other edges
           (zeroLength || middleInside(nodes, a, b)); // diagonal inside polygon
}

// signed area of a triangle

double Area(const NodeSlice nodes, const S32 p, const S32 q, const S32 r) {
    return (nodes.v[q].y - nodes.v[p].y) * (nodes.v[r].x - nodes.v[q].x) -
           (nodes.v[q].x - nodes.v[p].x) * (nodes.v[r].y - nodes.v[q].y);
}

// check if two points are equal

bool equals(const NodeSlice nodes, const S32 p1, const S32 p2) {
    return nodes.v[p1].x == nodes.v[p2].x && nodes.v[p1].y == nodes.v[p2].y;
}

// check if two segments intersect; by default includes collinear boundary
// touches

bool intersects(const NodeSlice nodes, const S32 p1, const S32 q1, const S32 p2,
                const S32 q2, bool includeBoundary) {
    const F64 o1 = Area(nodes, p1, q1, p2);
    const F64 o2 = Area(nodes, p1, q1, q2);
    const F64 o3 = Area(nodes, p2, q2, p1);
    const F64 o4 = Area(nodes, p2, q2, q1);

    // general case: the two segments straddle each other (proper crossing)
    if (((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) &&
        ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0))) {
        return true;
    }

    if (!includeBoundary) {
        return false;
    }

    if (o1 == 0 && onSegment(nodes, p1, p2, q1))
        return true; // p1, q1 and p2 are collinear and p2 lies on p1q1
    if (o2 == 0 && onSegment(nodes, p1, q2, q1))
        return true; // p1, q1 and q2 are collinear and q2 lies on p1q1
    if (o3 == 0 && onSegment(nodes, p2, p1, q2))
        return true; // p2, q2 and p1 are collinear and p1 lies on p2q2
    if (o4 == 0 && onSegment(nodes, p2, q1, q2))
        return true; // p2, q2 and q1 are collinear and q1 lies on p2q2

    return false;
}

// for collinear points p, q, r, check if point q lies on segment pr

bool onSegment(const NodeSlice nodes, const S32 p, const S32 q, const S32 r) {
    return nodes.v[q].x <= Max(nodes.v[p].x, nodes.v[r].x) &&
           nodes.v[q].x >= Min(nodes.v[p].x, nodes.v[r].x) &&
           nodes.v[q].y <= Max(nodes.v[p].y, nodes.v[r].y) &&
           nodes.v[q].y >= Min(nodes.v[p].y, nodes.v[r].y);
}

// check if a polygon diagonal intersects any polygon segments

bool intersectsPolygon(const NodeSlice nodes, const S32 a, const S32 b) {
    // diagonal bbox; an edge whose bbox can't overlap it can't intersect it, so
    // skip the orientation test for those (the common case — the diagonal is
    // short)
    const F64 diagMinX = Min(nodes.v[a].x, nodes.v[b].x);
    const F64 diagMaxX = Max(nodes.v[a].x, nodes.v[b].x);
    const F64 diagMinY = Min(nodes.v[a].y, nodes.v[b].y);
    const F64 diagMaxY = Max(nodes.v[a].y, nodes.v[b].y);

    S32 p = a;
    do {
        const S32 n = nodes.v[p].next;
        if ((nodes.v[p].x > diagMaxX && nodes.v[n].x > diagMaxX) ||
            (nodes.v[p].x < diagMinX && nodes.v[n].x < diagMinX) ||
            (nodes.v[p].y > diagMaxY && nodes.v[n].y > diagMaxY) ||
            (nodes.v[p].y < diagMinY && nodes.v[n].y < diagMinY)) {
            p = n;
            continue;
        }
        if (nodes.v[p].vertex != nodes.v[a].vertex &&
            nodes.v[n].vertex != nodes.v[a].vertex &&
            nodes.v[p].vertex != nodes.v[b].vertex &&
            nodes.v[n].vertex != nodes.v[b].vertex &&
            intersects(nodes, p, n, a, b, true)) {
            return true;
        }
        p = n;
    } while (p != a);

    return false;
}

// check if a polygon diagonal is locally inside the polygon

bool locallyInside(const NodeSlice nodes, const S32 a, const S32 b) {
    return Area(nodes, nodes.v[a].prev, a, nodes.v[a].next) < 0
               ? Area(nodes, a, b, nodes.v[a].next) >= 0 &&
                     Area(nodes, a, nodes.v[a].prev, b) >= 0
               : Area(nodes, a, b, nodes.v[a].prev) < 0 ||
                     Area(nodes, a, nodes.v[a].next, b) < 0;
}

// check if the middle Vertex of a polygon diagonal is inside the polygon

bool middleInside(const NodeSlice nodes, const S32 a, const S32 b) {
    S32 p = a;
    bool inside = false;
    F64 px = (nodes.v[a].x + nodes.v[b].x) / 2;
    F64 py = (nodes.v[a].y + nodes.v[b].y) / 2;
    do {
        S32 n = nodes.v[p].next;
        if (((nodes.v[p].y > py) != (nodes.v[n].y > py)) &&
            (px < (nodes.v[n].x - nodes.v[p].x) * (py - nodes.v[p].y) /
                          (nodes.v[n].y - nodes.v[p].y) +
                      nodes.v[p].x))
            inside = !inside;
        p = n;
    } while (p != a);

    return inside;
}

// link two polygon vertices with a bridge; if the vertices belong to the same
// ring, it splits polygon into two; if one belongs to the outer ring and
// another to a hole, it merges it into a single ring

S32 splitPolygon(NodeArray *nodes, S32 a, S32 b) {
    const S32 a2 = NodeArrayPush(
        nodes,
        (Node){.vertex = nodes->d[a].vertex, .x = nodes->d[a].x, .y = nodes->d[a].y});
    const S32 b2 = NodeArrayPush(
        nodes,
        (Node){.vertex = nodes->d[b].vertex, .x = nodes->d[b].x, .y = nodes->d[b].y});
    const S32 an = nodes->d[a].next;
    const S32 bp = nodes->d[b].prev;

    nodes->d[a].next = b;
    nodes->d[b].prev = a;

    nodes->d[a2].next = an;
    nodes->d[an].prev = a2;

    nodes->d[b2].next = a2;
    nodes->d[a2].prev = b2;

    nodes->d[bp].next = b2;
    nodes->d[b2].prev = bp;
    return b2;
}

// create a node and util::optionally link it with previous one (in a circular
// doubly linked list)

internal S32 insertNode(NodeArray *nodes, S32 vertex, const Coord2 pt, S32 last) {
#ifdef DEBUG
#ifdef DEBUG_EARCUT
    ValidateNodeStructure(NodeSliceFromArray(nodes));
    ASSERT(vertex, "can not create node with null vertex");
#endif
#endif
    S32 p = NodeArrayPush(nodes, (Node){.vertex = vertex, .x = pt.x, .y = pt.y});

    if (!last) {
        nodes->d[p].prev = p;
        nodes->d[p].next = p;
    } else {
        assert(last);
        nodes->d[p].next = nodes->d[last].next;
        nodes->d[p].prev = last;
        nodes->d[nodes->d[last].next].prev = p;
        nodes->d[last].next = p;
    }
#ifdef DEBUG
#ifdef DEBUG_EARCUT
    ValidateNodeStructure(NodeSliceFromArray(nodes));
    ASSERT(p >= 0 && p < nodes->count, "invalid node index");
#endif
#endif
    return p;
}

void RemoveNode(NodeSlice nodes, S32 p) {
#ifdef DEBUG
#ifdef DEBUG_EARCUT
    ValidateNodeStructure(nodes);
#endif
#endif
    nodes.v[nodes.v[p].next].prev = nodes.v[p].prev;
    nodes.v[nodes.v[p].prev].next = nodes.v[p].next;

    if (nodes.v[p].prevZ)
        nodes.v[nodes.v[p].prevZ].nextZ = nodes.v[p].nextZ;
    if (nodes.v[p].nextZ)
        nodes.v[nodes.v[p].nextZ].prevZ = nodes.v[p].prevZ;
#ifdef DEBUG
#ifdef DEBUG_EARCUT
    nodes.v[p].removed = true;
    ValidateNodeStructure(nodes);
#endif
#endif
}

void RemoveNodeAndUpdateIndex(NodeSlice nodes, BlockBoundingBoxSlice blocks, S32 p) {
#ifdef DEBUG
#ifdef DEBUG_EARCUT
    ValidateNodeStructure(nodes);
#endif
#endif
    nodes.v[nodes.v[p].next].prev = nodes.v[p].prev;
    nodes.v[nodes.v[p].prev].next = nodes.v[p].next;

    if (nodes.v[p].prevZ)
        nodes.v[nodes.v[p].prevZ].nextZ = nodes.v[p].nextZ;
    if (nodes.v[p].nextZ)
        nodes.v[nodes.v[p].nextZ].prevZ = nodes.v[p].prevZ;

    // keep the hole-bridge index's block bboxes covering the healed prev->next
    // edge
    growBlock(nodes, blocks, nodes.v[p].prev, nodes.v[p].next);
#ifdef DEBUG
#ifdef DEBUG_EARCUT
    nodes.v[p].removed = true;
    ValidateNodeStructure(nodes);
#endif
#endif
}

void Earcut(TriangleArray *triangles, const Coord2Slice coords,
            const S32Slice contour_sizes) {
    {
        ASSERT(coords.v[0].x == 0.f && coords.v[0].y == 0.f, "Coords[0] not zeroed")
        ASSERT(coords.count > 3, "Can not triangulate polygon with < 3 coordinates")
        S32 sum = 1; // account for zeroed element at coords[0].
        for (S32 i = 0; i < contour_sizes.count; i += 1) {
            sum += contour_sizes.v[i];
        }
        ASSERT(coords.count == sum,
               "coordinate count: %d and contour sizes: %d do not match", coords.count,
               sum);
    }

    // FAST PATH for Polygons with no holes and 4 Vertices.
    // Simply return [{1, 2, 3}, {1, 3, 4}].
    if (contour_sizes.count == 1 && contour_sizes.v[0] == 4) {
        if (isContourClockwise(coords)) {
            TriangleArrayPush(triangles, (Triangle){1, 2, 3});
            TriangleArrayPush(triangles, (Triangle){1, 3, 4});
        } else {
            TriangleArrayPush(triangles, (Triangle){1, 4, 3});
            TriangleArrayPush(triangles, (Triangle){1, 3, 2});
        }
        return;
    }

    const S32 triangle_index_min = triangles->count;
    Temp_Arena_Memory scratch = GetScratch();

    // THIS ARRAY IS 1 indexed 0 is the sentinel value
    NodeArray nodes = NodeArrayNew(scratch.arena, 2 * coords.count);
    NodeArrayPush(&nodes, (Node){0});

    RangeArray contours = RangeArrayNew(scratch.arena, contour_sizes.count);
    S32 coords_so_far = 1;
    for (S32 i = 0; i < contour_sizes.count; i += 1) {
        RangeArrayPush(&contours,
                       (Range){.min = coords_so_far, .count = contour_sizes.v[i]});
        coords_so_far += contour_sizes.v[i];
    }

    EarcutPolygon polygon = {.coords = coords,
                             .contours = RangeSliceFromArray(&contours)};

    S32 outerNode = LinkedList(&nodes, coords, polygon.contours.v[0], true);
    if (!outerNode || nodes.d[outerNode].prev == nodes.d[outerNode].next)
        return;

    outerNode = eliminateHoles(polygon, &nodes, outerNode);
#ifdef DEBUG
#ifdef DEBUG_EARCUT
    PrintLinkedList(NodeSliceFromArray(&nodes), outerNode);
#endif
#endif

    // if the shape is not too simple, we'll use z-order curve hash later;
    // calculate polygon bbox
    ZOrderInfo bounds = {0};
    bounds.hashing = coords.count > 160;
    if (bounds.hashing) {
        S32 p = nodes.d[outerNode].next;
        bounds.minX = bounds.maxX = nodes.d[outerNode].x;
        bounds.minY = bounds.maxY = nodes.d[outerNode].y;
        do {
            const F64 x = nodes.d[p].x;
            const F64 y = nodes.d[p].y;
            bounds.minX = Min(bounds.minX, x);
            bounds.minY = Min(bounds.minY, y);
            bounds.maxX = Max(bounds.maxX, x);
            bounds.maxY = Max(bounds.maxY, y);
            p = nodes.d[p].next;
        } while (p != outerNode);

        // minX, minY and inv_size are later used to transform coords into
        // integers for z-order calculation
        bounds.inv_size = Max(bounds.maxX - bounds.minX, bounds.maxY - bounds.minY);
        bounds.inv_size = bounds.inv_size != 0.f ? (32767.f / bounds.inv_size) : 0.f;
    }

    EarcutLinked(&nodes, bounds, outerNode, triangles);

#ifdef DEBUG
#ifdef DEBUG_EARCUT
    for (S32 i = triangle_index_min; i < triangles->count; i += 1) {
        const Triangle t = triangles->d[i];
        fprintf(stderr,
                "Triangle [%d, %d, %d] "
                "[(%f, %f), (%f, %f), (%f, %f)]\n",
                t.a, t.b, t.c, coords.v[t.a].x, coords.v[t.a].y, coords.v[t.b].x,
                coords.v[t.b].y, coords.v[t.c].x, coords.v[t.c].y);
    }
#endif
#endif
#if 0
  {
    for (S32 i = triangle_index_min; i < triangles->count; i += 1) {
      Triangle t = triangles->d[i];
      F64 cross = CROSS(coords.v[t.a], coords.v[t.b], coords.v[t.c]);
      ASSERT(cross >= -0.0001f,
             "Triangle [%d, %d, %d] not counter-clockwise, cross: %f\n"
             "[(%f, %f), (%f, %f), (%f, %f)]",
             t.a, t.b, t.c, cross, coords.v[t.a].x, coords.v[t.a].y,
             coords.v[t.b].x, coords.v[t.b].y, coords.v[t.c].x, coords.v[t.c].y)
    }
  }
#endif
    temp_arena_memory_end(scratch);
}

// Refine a triangulation toward the constrained Delaunay triangulation by
// legalizing every interior edge in place with Lawson flips — maximizing the
// minimum angle and removing most slivers. Adapted from delaunator's edge
// legalization. Uses non-robust predicates: float input is fine, and the
// worst case is a not-quite-Delaunay edge, never an invalid mesh. Ported from
// earcut v3.2.3.

/*
class Refiner {
public:
  // triangles: triangle indices as returned by earcut, mutated in place.
  // coords: random-access container of points, indexed by vertex index
  // (coords[i] -> point i), read through the same util::nth<0>/<1> accessors
as
  // earcut's input.

  void operator()(std::vector<N> &triangles, const Coords &coords) {
    using Point = std::decay<decltype(coords[0])>::type;
    const int n = static_cast<int>(triangles.size());
    if (n < 6)
      return;
    ensureScratch(static_cast<S32>(n));
    gen++; // bumping the generation logically empties the hash (no clearing)
    std::fill(heVec.begin(), heVec.begin() + n, -1);

    // Raw pointers into the scratch: indexed by the int/uint half-edge and
hash
    // indices below, where operator[]'s size_type would trip
-Wsign-conversion
    // on every subscript.
    N *t = triangles.data();
    int32_t *he = heVec.data();
    int32_t *edgeStack = edgeStackVec.data();
    int32_t *hTable = hTableVec.data();
    uint32_t *hStamp = hStampVec.data();
    uint8_t *edgeStamp = edgeStampVec.data();

    auto X = [&](N p) -> double {
      return static_cast<double>(util::nth<0, Point>::get(coords[p]));
    };
    auto Y = [&](N p) -> double {
      return static_cast<double>(util::nth<1, Point>::get(coords[p]));
    };

    // Build half-edge twins with an undirected-edge hash; consumed slots mark
    // linked pairs. As each pair is linked we seed the stack with one
    // representative (s, the earlier-inserted edge) — this fuses the initial
    // "push every interior edge" pass into the build, saving a full O(n)
scan.
    // edgeStamp is all-zero here (balanced push/pop leaves it clean) and each
    // pair links once, so the seed write needs no dedup guard.
    int i = 0;
    for (int e = 0; e < n; e++) {
      const N a = t[e], b = t[nextHE(e)];
      const N lo = a < b ? a : b, hi = a < b ? b : a;
      uint32_t h =
          (uint32_t(lo) * 0x9e3779b1u ^ uint32_t(hi) * 0x85ebca6bu) & hMask;
      while (hStamp[h] == gen) {
        const int32_t s = hTable[h];
        // s == -1 marks a consumed slot (a pair already linked) — skip past
it if (s != -1) { const N sa = t[s], sb = t[nextHE(s)]; if ((sa == lo && sb ==
hi) || (sa == hi && sb == lo)) { he[e] = s; he[s] = e; hTable[h] = -1; //
link, then consume the slot edgeStamp[s] = 1; edgeStack[i++] = s; // seed the
interior edge for the cascade break;
          }
        }
        h = (h + 1) & hMask;
      }
      if (hStamp[h] != gen) {
        hTable[h] = e;
        hStamp[h] = gen;
      } // first occurrence: insert
    }

    while (i > 0) {
      const int a = edgeStack[--i];
      edgeStamp[a] = 0;
      const int b = he[a];
      if (b == -1)
        continue;

      const int a0 = a - a % 3;
      const int b0 = b - b % 3;
      const int ar = a0 + (a + 2) % 3;
      const int al = a0 + (a + 1) % 3;
      const int bl = b0 + (b + 2) % 3;
      const int br = b0 + (b + 1) % 3;
      const N p0 = t[ar], pr = t[a], pl = t[al], p1 = t[bl];

      const double x0 = X(p0), y0 = Y(p0);
      const double xr = X(pr), yr = Y(pr);
      const double xl = X(pl), yl = Y(pl);
      const double x1 = X(p1), y1 = Y(p1);

      // Test inCircle first: most interior edges are already Delaunay
(inCircle
      // true → no flip), so this short-circuits before the two convexity
      // orients on the common path. The quad must also be convex (both new
      // triangles CCW) — flipping a reflex quad would push a triangle outside
      // the polygon. Boundary/hole edges self-protect via he == -1.
      if (!inCircle(x0, y0, xr, yr, xl, yl, x1, y1) &&
          orient(x0, y0, xr, yr, x1, y1) > 0 &&
          orient(x0, y0, x1, y1, xl, yl) > 0) {
        t[a] = p1;
        t[b] = p0;
        const int32_t hbl = he[bl], har = he[ar];
        he[a] = hbl;
        if (hbl != -1)
          he[hbl] = a;
        he[b] = har;
        if (har != -1)
          he[har] = b;
        he[ar] = bl;
        he[bl] = ar;

        // re-check the quad's four outer edges; skip boundary edges (he ==
-1)
        // and any already queued (edgeStamp), which also keeps the stack
        // bounded by n.
        if (hbl != -1 && edgeStamp[a] == 0) {
          edgeStamp[a] = 1;
          edgeStack[i++] = a;
        }
        if (har != -1 && edgeStamp[b] == 0) {
          edgeStamp[b] = 1;
          edgeStack[i++] = b;
        }
        if (he[al] != -1 && edgeStamp[al] == 0) {
          edgeStamp[al] = 1;
          edgeStack[i++] = al;
        }
        if (he[br] != -1 && edgeStamp[br] == 0) {
          edgeStamp[br] = 1;
          edgeStack[i++] = br;
        }
      }
    }
  }

private:
  // Reusable scratch, grown on demand like earcut's z-order arrays and reused
  // across calls:
  //   he      = twin half-edge of each edge, or -1 on the polygon boundary
  //   hTable  = open-addressing hash, slot -> half-edge index, valid iff
  //   hStamp[slot] == gen edgeStamp = pending-in-stack flag, cleared when the
  //   edge is popped
  std::vector<int32_t> heVec, edgeStackVec, hTableVec;
  std::vector<uint32_t> hStampVec;
  std::vector<uint8_t> edgeStampVec;
  uint32_t hMask = 0, gen = 0;

  static int nextHE(int e) {
    return e - e % 3 + (e + 1) % 3;
  } // next half-edge in same triangle

  static double orient(double ax, double ay, double bx, double by, double cx,
                       double cy) {
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
  }

  // Whether p is inside or exactly on the circumcircle of triangle (a, b, c).
  // Sign is negated vs the usual predicate to match earcut's CCW winding —
the
  // standard sign builds the anti-Delaunay mesh. Cocircular quads are legal
  // ties, so refine only flips when this returns false.
  static bool inCircle(double ax, double ay, double bx, double by, double cx,
                       double cy, double px, double py) {
    const double dx = ax - px, dy = ay - py, ex = bx - px, ey = by - py,
                 fx = cx - px, fy = cy - py;
    const double ap = dx * dx + dy * dy, bp = ex * ex + ey * ey,
                 cp = fx * fx + fy * fy;
    // A near-cocircular quad is a legal Delaunay tie, but roundoff can flag
    // both an edge and its flip as illegal, cascading into an endless flip
loop
    // (#205) — so treat a determinant within a small margin of zero as a tie.
    // The determinant's worst-case roundoff error is provably below
    // 9e-16·(ap+bp+cp)² (Shewchuk-style bound), so the margin guarantees
every
    // executed flip is illegal in exact arithmetic, and Lawson flipping
always
    // terminates.
    const double s = ap + bp + cp;
    return dx * (ey * cp - bp * fy) - dy * (ex * cp - bp * fx) +
               ap * (ex * fy - ey * fx) <=
           1e-13 * s * s;
  }

  void ensureScratch(S32 n) {
    // edgeStack holds at most one entry per half-edge (edgeStamp dedups), so
n
    // is a safe cap.
    if (edgeStackVec.size() < n)
      edgeStackVec.resize(n);
    if (heVec.size() < n)
      heVec.resize(n);
    if (edgeStampVec.size() < n)
      edgeStampVec.resize(n, 0);
    S32 size = 1;
    while (size < n * 4)
      size <<= 1; // power-of-two table, load factor <= 0.25
    if (hTableVec.size() < size) {
      hTableVec.resize(size);
      hStampVec.resize(size, 0);
    }
    hMask = uint32_t(size) - 1;
  }
};

// Opt-in Delaunay-refinement post-pass for earcut() output (or any manifold
// triangle-index array). Legalizes every interior edge in place with Lawson
// flips. See detail::Refiner. `coords` is a random-access container of points
// indexed by vertex index; `triangles` is mutated in place.

void refine(std::vector<N> &triangles, const Coords &coords) {
  static thread_local mapbox::detail::Refiner<N> refiner;
  refiner(triangles, coords);
}
*/
#ifdef DEBUG
#ifdef DEBUG_EARCUT
static void ValidateNodeStructure(NodeSlice nodes) {
    ASSERT(nodes.v[0].x == 0.f, "nodes[0] != 0");
    ASSERT(nodes.v[0].y == 0.f, "nodes[0] != 0");
    ASSERT(nodes.v[0].prev == 0, "nodes[0] != 0");
    ASSERT(nodes.v[0].next == 0, "nodes[0] != 0");
    ASSERT(nodes.v[0].z == 0, "nodes[0] != 0");
    ASSERT(nodes.v[0].vertex == 0, "nodes[0] != 0");
    ASSERT(nodes.v[0].prevZ == 0, "nodes[0] != 0");
    ASSERT(nodes.v[0].nextZ == 0, "nodes[0] != 0");
    ASSERT(nodes.v[0].steiner == false, "nodes[0] != 0");
    ASSERT(nodes.v[0].removed == false, "nodes[0] != 0");
    for (S32 i = 1; i < nodes.count; i += 1) {
        Node n = nodes.v[i];
        if (n.vertex && !n.removed) {
            // ASSERT(n.x != 0.f, "nodes[%d].x == 0", i);
            // ASSERT(n.y != 0.f, "nodes[%d].y == 0", i);
            ASSERT(n.vertex != 0, "nodes[%d].vertex == 0", i);
            const S32 prev = n.prev;
            const S32 next = n.next;
            ASSERT(nodes.v[prev].next == i, "invalid node information");
            ASSERT(nodes.v[next].prev == i, "invalid node information");
        }
    }
}

static void ValidateZOrderCurve(NodeSlice nodes) {
    for (S32 i = 0; i < nodes.count; i += 1) {
        Node n = nodes.v[i];
        if (n.vertex && n.z && !n.removed) {
            const S32 prevZ = n.prevZ;
            const S32 nextZ = n.nextZ;
            if (prevZ) {
                ASSERT(nodes.v[prevZ].nextZ == i, "invalid z-order information");
            }
            if (nextZ) {
                ASSERT(nodes.v[nextZ].prevZ == i, "invalid z-order information");
            }
        }
    }
}
#endif
#endif
