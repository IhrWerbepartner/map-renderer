#pragma once
#include "arena.c"
#include "base.h"
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct {
  S32 v0, v1;       // two endpoints in the coordinate array
  S32 root0, root1; // sinks in QueryStructure of v0 and v1
  S32 next, prev;   // Next/Prev logical segment in polygon
  bool is_inserted; // inserted in trapezoidation yet ?
} Segment;

typedef enum NodeType {
  X = 1, // make this distinct from non initialized node which is ZIIed.
  Y = 2,
  SINK = 3,
} NodeType;

typedef enum MergeSide {
  LEFT = 1,
  RIGHT,
} MergeSide;

typedef struct {
  NodeType node_type;
  union {
    S32 segment;   // node_type -> X
    Coord2 yval;   // node_type -> Y
    S32 trapezoid; // node_type -> SINK
  };
  S32 parent;
  S32 left, right; // children
} QueryNode;

typedef struct {
  Coord2 max_y, min_y;             // max/min y-values
  S32 left_segment, right_segment; // index of two adjoining segments
  S32 up0, up1;
  S32 down0, down1;
  S32 sink_node;    // pointer to corresponding node in QueryStructure
  S32 usave, uside; /* I forgot what this means */
  bool is_valid;
} Trapezoid;

DeclFixedArray(SegmentArray, Segment);
DeclFixedArray(QueryNodeArray, QueryNode);
DeclFixedArray(TrapezoidArray, Trapezoid);

// Circularly linked list describing the monotone polygon
typedef struct {
  S32 vertex;
  S32 next;
  S32 prev;
} MonotoneChain;

typedef struct {
  Coord2 pt;
  S32 next_vertex[4];                    /* next vertices for the 4 chains */
  S32 vertex_index_in_monotone_chain[4]; /* position of v in the 4 chains */
  S32 next_free;
} VertexChain;

DeclFixedArray(MonotoneChainArray, MonotoneChain);
DeclFixedArray(VertexChainArray, VertexChain);

typedef struct TraversalInfo TraversalInfo;
struct TraversalInfo {
  VertexChainArray *vertex_chains;
  MonotoneChainArray *monotone_polygon_chains;
  const TrapezoidSlice trapezoids;
  const SegmentSlice segments;
  const Coord2Slice vertices;
  S32Slice visited_trapezoids;
  S32Array *monotone_chain_start_vertex;
};

typedef enum TraversalDirection TraversalDirection;
enum TraversalDirection {
  UP,
  DOWN,
};

typedef enum MonotoneBaseSide MonotoneBaseSide;
enum MonotoneBaseSide {
  BASE_LEFT,
  BASE_RIGHT,
};

#define TRIANGULATE_INFINITY (1 << 30)
#define COORD2_MINUS_INFINITY                                                  \
  (Coord2) {                                                                   \
    .x = -1.f * TRIANGULATE_INFINITY, .y = -1.f * TRIANGULATE_INFINITY         \
  }
#define COORD2_PLUS_INFINITY                                                   \
  (Coord2) { .x = TRIANGULATE_INFINITY, .y = TRIANGULATE_INFINITY }
#define C_EPS 1.11e-16
#define FP_EQUAL(s, t) (fabs((s) - (t)) <= C_EPS)
#define CROSS(v0, v1, v2)                                                      \
  (((v1).x - (v0).x) * ((v2).y - (v0).y) -                                     \
   ((v1).y - (v0).y) * ((v2).x - (v0).x))

#define DOT(v0, v1) ((v0).x * (v1).x + (v0).y * (v1).y)

static void GeneratePermutation(S32Array *permutation);
static S32 RandomSegment(S32Array *permutation);
static S32 InitQueryStructure(QueryNodeArray *query_structure,
                              Coord2Slice vertices, TrapezoidArray *trapezoids,
                              SegmentArray *segments, S32 initial_segment);
static void ConstructTrapezoidation(QueryNodeArray *query_structure,
                                    Coord2Slice vertices,
                                    SegmentArray *segments,
                                    TrapezoidArray *trapezoids,
                                    S32Array *permutation);
static void AddSegment(QueryNodeArray *query_structure, Coord2Slice vertices,
                       SegmentSlice segments, TrapezoidArray *trapezoids,
                       S32 segment);
static void MergeTrapezoids(QueryNodeSlice query_structure,
                            TrapezoidArray *trapezoids, S32 segment,
                            S32 first_trapezoid, S32 last_trapezoid,
                            MergeSide side);
static S32 TrapezoidIndexFromVertex(QueryNodeSlice query_structure,
                                    SegmentSlice segments, Coord2Slice vertices,
                                    S32 vertex, S32 segment_endpoint, S32 root);
static void MonotonateTrapezoids(VertexChainArray *vertex_chains,
                                 MonotoneChainArray *monotone_polygon_chains,
                                 TrapezoidSlice trapezoids,
                                 SegmentSlice segments, Coord2Slice vertices,
                                 S32Slice visited_trapezoids,
                                 S32Array *monotone_chain_start_vertex);
static void TraversePolygon(TraversalInfo *TI, S32 current_monotone,
                            S32 current_trapezoid, S32 traversed_from,
                            TraversalDirection dir);
static S32 SplitPolygonByDiagonal(VertexChainSlice vertex_chains,
                                  MonotoneChainArray *monotone_polygon_chains,
                                  S32Array *monotone_chain_start_vertex,
                                  S32 current_monotone_polygon, S32 v0, S32 v1);
static void TriangulateMonotonePolygons(
    VertexChainSlice vertex_chains, MonotoneChainSlice monotone_polygon_chains,
    S32Slice monotone_chain_start_vertex, TriangleArray *op);
static void TriangulateSingleMonotonePolygon(VertexChainSlice vertex_chains,
                                             MonotoneChainSlice monotone_chains,
                                             S32 y_max_index,
                                             MonotoneBaseSide side,
                                             TriangleArray *triangles);
// ------------- MATH FUNCTIONS ---------------
static S32 MathLogStar(S32 n);
static S32 MathN(S32 n, S32 h);
static Coord2 Coord2Max(Coord2 v0, Coord2 v1);
static Coord2 Coord2Min(Coord2 v0, Coord2 v1);

static bool Coord2GreaterThan(Coord2 v0, Coord2 v1);
static bool Coord2EqualTo(Coord2 v0, Coord2 v1);
static bool Coord2GreaterThanEqualTo(Coord2 v0, Coord2 v1);
static bool Coord2LessThan(Coord2 v0, Coord2 v1);
static bool VertexLeftOfSegment(Coord2Slice vertices, SegmentSlice segments,
                                S32 segment, S32 vertex);

// --------------- VALIDATION FUNCTIONS --------------
#ifdef DEBUG
static void ValidateQueryStructure(QueryNodeSlice qs);
static void ValidateTrapezoidStructure(TrapezoidSlice ts);
static void ValidateQueryTrapezoidSegmentStructures(QueryNodeSlice qs,
                                                    TrapezoidSlice ts,
                                                    SegmentSlice segments,
                                                    Coord2Slice vertices);
static void ValidateMonotoneChains(const MonotoneChainSlice chains);
static void ValidateMonotoneAndVertexChains(const VertexChainSlice vc,
                                            const MonotoneChainSlice mc);
static void ValidateMonotonePolygon(VertexChainSlice vertex_chains,
                                    MonotoneChainSlice mc, S32 y_max_index,
                                    MonotoneBaseSide side);
#endif

// computes the triangulation of the coordinates which are partitioned by
// contour_sizes. Coords[0] is NOT USED. so coords[1..=cs[0]] polygons are
// the first ones, coords[cs[0]..cs[1]] the next etc. returns the triangles
// as indices into the coords array.
void TessalatePolygon(TriangleArray *triangles, const Coord2Slice coords,
                      const S32Slice contour_sizes) {
  Temp_Arena_Memory scratch = GetScratch();
  QueryNodeArray query_structure =
      QueryNodeArrayNew(scratch.arena, 8 * coords.count);
  SegmentArray segments = SegmentArrayNew(scratch.arena, coords.count);
  TrapezoidArray trapezoids =
      TrapezoidArrayNew(scratch.arena, 4 * coords.count);
  S32Array permutated_segments = S32ArrayNew(scratch.arena, coords.count);
  MonotoneChainArray monotone_polygon_chains =
      MonotoneChainArrayNew(scratch.arena, 4 * coords.count);
  S32Array visited_trapezoids = S32ArrayNew(scratch.arena, 4 * coords.count);
  visited_trapezoids.len = visited_trapezoids.capacity;
  VertexChainArray vertex_chains =
      VertexChainArrayNew(scratch.arena, coords.count);
  S32Array monotone_chain_start_vertex =
      S32ArrayNew(scratch.arena, coords.count);

  SegmentArrayPush(&segments, (Segment){0});
  for (S32 contour = 0; contour < contour_sizes.count; contour += 1) {
    const S32 contour_point_count = contour_sizes.v[contour];
    const S32 first = segments.len;
    const S32 last = first + contour_point_count - 1;

    for (S32 j = 0; j < contour_point_count; j++) {
      const S32 i = SegmentArrayPush(&segments, (Segment){0});
      segments.data[i].v0 = i;
      segments.data[i].is_inserted = false;
      if (i == last) {
        segments.data[i].next = first;
        segments.data[i].prev = i - 1;
        segments.data[i - 1].v1 = segments.data[i].v0;
        segments.data[i].v1 = segments.data[first].v0;
      } else if (i == first) {
        segments.data[i].next = i + 1;
        segments.data[i].prev = last;
      } else {
        segments.data[i].prev = i - 1;
        segments.data[i].next = i + 1;
        segments.data[i - 1].v1 = segments.data[i].v0;
      }
    }
  }

  GeneratePermutation(&permutated_segments);
  ConstructTrapezoidation(&query_structure, coords, &segments, &trapezoids,
                          &permutated_segments);
  MonotonateTrapezoids(&vertex_chains, &monotone_polygon_chains,
                       TrapezoidSliceFromArray(&trapezoids),
                       SegmentSliceFromArray(&segments), coords,
                       S32SliceFromArray(&visited_trapezoids),
                       &monotone_chain_start_vertex);
  TriangulateMonotonePolygons(
      VertexChainSliceFromArray(&vertex_chains),
      MonotoneChainSliceFromArray(&monotone_polygon_chains),
      S32SliceFromArray(&monotone_chain_start_vertex), triangles);
  temp_arena_memory_end(scratch);
}

/* For each monotone polygon, find the ymax and ymin (to determine the
 * two y-monotone chains) and pass on this monotone polygon for greedy
 * triangulation. Take care not to triangulate duplicate monotone polygons
 */
static void TriangulateMonotonePolygons(
    VertexChainSlice vertex_chains, MonotoneChainSlice monotone_polygon_chains,
    S32Slice monotone_chain_start_vertex, TriangleArray *op) {
#ifdef DEBUG
  for (S32 i = 0; i < monotone_chain_start_vertex.count; i++) {
    fprintf(stderr, "\n\nPolygon %d: ", i);
    S32 first_vertex =
        monotone_polygon_chains.v[monotone_chain_start_vertex.v[i]].vertex;
    S32 p = monotone_polygon_chains.v[monotone_chain_start_vertex.v[i]].next;
    fprintf(stderr, "%d ",
            monotone_polygon_chains.v[monotone_chain_start_vertex.v[i]].vertex);
    while (monotone_polygon_chains.v[p].vertex != first_vertex) {
      fprintf(stderr, "%d ", monotone_polygon_chains.v[p].vertex);
      p = monotone_polygon_chains.v[p].next;
    }
  }
  fprintf(stderr, "\n");
#endif

  for (S32 i = 0; i < monotone_chain_start_vertex.count; i++) {
    S32 vertex_count = 1;
    const S32 first_vertex =
        monotone_polygon_chains.v[monotone_chain_start_vertex.v[i]].vertex;
    Coord2 ymax = vertex_chains.v[first_vertex].pt;
    Coord2 ymin = vertex_chains.v[first_vertex].pt;
    S32 posmax = monotone_chain_start_vertex.v[i];
    S32 p = monotone_polygon_chains.v[monotone_chain_start_vertex.v[i]].next;
    S32 v;
    while ((v = monotone_polygon_chains.v[p].vertex) != first_vertex) {
      if (Coord2GreaterThan(vertex_chains.v[v].pt, ymax)) {
        ymax = vertex_chains.v[v].pt;
        posmax = p;
      }
      if (Coord2LessThan(vertex_chains.v[v].pt, ymin)) {
        ymin = vertex_chains.v[v].pt;
      }
      p = monotone_polygon_chains.v[p].next;
      vertex_count += 1;
    }

    if (vertex_count == 3) /* already a triangle */
    {
      TriangleArrayPush(
          op, (Triangle){
                  monotone_polygon_chains.v[p].vertex,
                  monotone_polygon_chains.v[monotone_polygon_chains.v[p].next]
                      .vertex,
                  monotone_polygon_chains.v[monotone_polygon_chains.v[p].prev]
                      .vertex});
    } else { /* triangulate the polygon */
      v = monotone_polygon_chains.v[monotone_polygon_chains.v[posmax].next]
              .vertex;
      if (Coord2EqualTo(vertex_chains.v[v].pt,
                        ymin)) { /* LHS is a single line */
        TriangulateSingleMonotonePolygon(vertex_chains, monotone_polygon_chains,
                                         posmax, BASE_LEFT, op);
      } else
        TriangulateSingleMonotonePolygon(vertex_chains, monotone_polygon_chains,
                                         posmax, BASE_RIGHT, op);
    }
  }

#ifdef DEBUG
  {
    for (S32 i = 0; i < TriangleArrayLength(op); i++)
      fprintf(stderr, "tri #%d: (%d, %d, %d)\n", i, op->data[i].a,
              op->data[i].b, op->data[i].c);
  }
#endif
}

/* A greedy corner-cutting algorithm to triangulate a y-monotone
 * polygon in O(n) time.
 * Joseph O-Rourke, Computational Geometry in C. Page 47
 * Uses y_max_index as the vertex with is the maximum in the y-monotone chain.
 * Side determines if the chain is left or right of the single segment.
 *(y_max)                   (y_max)
 * |  \ BASE_LEFT  BASE_RIGHT/   ^
 * |   \                    /    |
 * |    \        or        /     |
 * |    /                 /      |
 * |   /                   \     |
 * |  /                     \    |
 * v /                       \   |
 * (y_min)                  (y_min)
 */
static void TriangulateSingleMonotonePolygon(VertexChainSlice vertex_chains,
                                             MonotoneChainSlice monotone_chains,
                                             S32 y_max_index,
                                             MonotoneBaseSide side,
                                             TriangleArray *triangles) {
#ifdef DEBUG
  ValidateMonotonePolygon(vertex_chains, monotone_chains, y_max_index, side);
#endif

  S32 v;
  Temp_Arena_Memory scratch = GetScratch();
  S32Array reflex_chain = S32ArrayNew(scratch.arena, triangles->capacity);
  S32 endv, vpos;
  // RHS segment is a single segment
  if (side == BASE_RIGHT) {
    S32ArrayPush(&reflex_chain, monotone_chains.v[y_max_index].vertex);
    S32 tmp = monotone_chains.v[y_max_index].next;
    S32ArrayPush(&reflex_chain, monotone_chains.v[tmp].vertex);

    vpos = monotone_chains.v[tmp].next;
    v = monotone_chains.v[vpos].vertex;
    endv = monotone_chains.v[monotone_chains.v[y_max_index].prev].vertex;
  } else {
    // LHS is a single segment
    S32 tmp = monotone_chains.v[y_max_index].next;
    S32ArrayPush(&reflex_chain, monotone_chains.v[tmp].vertex);
    tmp = monotone_chains.v[tmp].next;
    S32ArrayPush(&reflex_chain, monotone_chains.v[tmp].vertex);

    vpos = monotone_chains.v[tmp].next;
    v = monotone_chains.v[vpos].vertex;
    endv = monotone_chains.v[y_max_index].vertex;
  }

  while (v != endv || reflex_chain.len > 2) {
    if (reflex_chain.len > 1 &&
        CROSS(vertex_chains.v[v].pt,
              vertex_chains.v[reflex_chain.data[reflex_chain.len - 2]].pt,
              vertex_chains.v[reflex_chain.data[reflex_chain.len - 1]].pt) >
            0) {
      // convex corner: cut if off
      const S32 v2 = S32ArrayPop(&reflex_chain);
      const S32 v1 = S32ArrayPop(&reflex_chain);
      TriangleArrayPush(triangles, (Triangle){v1, v2, v});
      S32ArrayPush(&reflex_chain, v1);
    } else { // non-convex or reflex-chain empty: add v to the chain
      S32ArrayPush(&reflex_chain, v);
      vpos = monotone_chains.v[vpos].next;
      v = monotone_chains.v[vpos].vertex;
    }
  }
  // reached the bottom vertex. Add in the triangle formed
  S32 v2 = S32ArrayPop(&reflex_chain);
  S32 v1 = S32ArrayPop(&reflex_chain);
  TriangleArrayPush(triangles, (Triangle){v1, v2, v});

  temp_arena_memory_end(scratch);
}

static void MonotonateTrapezoids(VertexChainArray *vertex_chains,
                                 MonotoneChainArray *monotone_polygon_chains,
                                 const TrapezoidSlice trapezoids,
                                 const SegmentSlice segments,
                                 const Coord2Slice vertices,
                                 S32Slice visited_trapezoids,
                                 S32Array *monotone_chain_start_vertex) {
  /* Initialize the mon data-structure and start spanning all the */
  /* trapezoids within the polygon */
  MonotoneChainArrayPush(monotone_polygon_chains, (MonotoneChain){0});
  VertexChainArrayPush(vertex_chains, (VertexChain){0});
  for (S32 i = 1; i < segments.count; i++) {
    MonotoneChainArrayPush(monotone_polygon_chains,
                           (MonotoneChain){.prev = segments.v[i].prev,
                                           .next = segments.v[i].next,
                                           .vertex = i});
    VertexChainArrayPush(vertex_chains,
                         (VertexChain){.pt = vertices.v[segments.v[i].v0],
                                       .next_vertex[0] = segments.v[i].next,
                                       .vertex_index_in_monotone_chain[0] = i,
                                       .next_free = 1});
  }
#ifdef DEBUG
  ValidateMonotoneChains(MonotoneChainSliceFromArray(monotone_polygon_chains));
  ValidateTrapezoidStructure(trapezoids);
  for (S32 i = 1; i < segments.count; i += 1) {
    ASSERT(segments.v[i].is_inserted, "Segment %d not inserted", i)
  }
#endif

  // position of a vertex in the first chain
  S32ArrayPush(monotone_chain_start_vertex, 1);

  // First locate a trapezoid which lies inside the polygon and which is
  // triangular
  for (S32 i = 1; i < trapezoids.count; i++) {
    Trapezoid t = trapezoids.v[i];
    if (t.is_valid && t.left_segment && t.right_segment) {
      // triangle
      if (!(t.up0 || t.up1) || !(t.down0 || t.down1)) {
        // check for winding
        if (Coord2GreaterThan(vertices.v[segments.v[t.right_segment].v1],
                              vertices.v[segments.v[t.right_segment].v0])) {
          if (trapezoids.v[i].up0) {
            TraversePolygon(&(TraversalInfo){vertex_chains,
                                             monotone_polygon_chains,
                                             trapezoids, segments, vertices,
                                             visited_trapezoids,
                                             monotone_chain_start_vertex},
                            0, i, trapezoids.v[i].up0, DOWN);
          }
          else if (trapezoids.v[i].down0) {
            TraversePolygon(&(TraversalInfo){vertex_chains,
                                             monotone_polygon_chains,
                                             trapezoids, segments, vertices,
                                             visited_trapezoids,
                                             monotone_chain_start_vertex},
                            0, i, trapezoids.v[i].down0, UP);
          }
          break;
        }
      }
    }
  }
}

// recursively visit all the trapezoids, we need to remember from which
// trapezoid we entered this one, as this is important for winding information.
static void TraversePolygon(TraversalInfo *TI, S32 current_monotone,
                            S32 current_trapezoid, S32 traversed_from,
                            TraversalDirection dir) {
#ifdef DEBUG
  ValidateMonotoneChains(
      MonotoneChainSliceFromArray(TI->monotone_polygon_chains));
#endif
  if (!current_trapezoid || TI->visited_trapezoids.v[current_trapezoid]) {
    return;
  }
#ifdef DEBUG
  fprintf(stderr, "traversal %d -> %d\n", traversed_from, current_trapezoid);
#endif
  const Trapezoid t = TI->trapezoids.v[current_trapezoid];
  if (t.right_segment && t.left_segment && t.left_segment == t.right_segment) {
    ERROR_MSG("Left segment = right segment")
  }
  // TODO: figure out why trapezoids 37 has left and right segment equal to 39 but not in
  // prior validation check.
  TI->visited_trapezoids.v[current_trapezoid] = true;

  /* We have much more information available here.
   * rseg: goes upwards
   * lseg: goes downwards
   * Initially assume that dir = TR_FROM_DN (from the left)
   * Switch v0 and v1 if necessary afterward
   * special cases for triangles with cusps at the opposite ends.
   * take care of this first
   */
  if (!(t.up0 || t.up1)) {
    if (t.down0 && t.down1) /* downward opening triangle */
    {
      /* connect v0 and v1
       *      (v1)
       *    /      \
       *   /  (v0)  \
       *  /  /    \  \
       */
      S32 v0 = TI->trapezoids.v[t.down1].left_segment;
      S32 v1 = t.left_segment;
      if (traversed_from == t.down1) {
        S32 new_monotone = SplitPolygonByDiagonal(
            VertexChainSliceFromArray(TI->vertex_chains),
            TI->monotone_polygon_chains, TI->monotone_chain_start_vertex,
            current_monotone, v1, v0);
        TraversePolygon(TI, current_monotone, t.down1, current_trapezoid, DOWN);
        TraversePolygon(TI, new_monotone, t.down0, current_trapezoid, DOWN);
      } else {
        S32 new_monotone = SplitPolygonByDiagonal(
            VertexChainSliceFromArray(TI->vertex_chains),
            TI->monotone_polygon_chains, TI->monotone_chain_start_vertex,
            current_monotone, v0, v1);
        TraversePolygon(TI, current_monotone, t.down0, current_trapezoid, DOWN);
        TraversePolygon(TI, new_monotone, t.down1, current_trapezoid, DOWN);
      }
    } else {
      /* Just traverse all neighbors */
      TraversePolygon(TI, current_monotone, t.down1, current_trapezoid, DOWN);
      TraversePolygon(TI, current_monotone, t.down0, current_trapezoid, DOWN);
    }
  } else if (!(t.down0 || t.down1)) {
    if (t.up0 && t.up1) /* upward opening triangle */
    {
      /* connect v0 and v1
       *    \      /
       *     \    /
       *   \  (v1)  /
       *    \      /
       *      (v0)
       */
      S32 v0 = t.right_segment;
      S32 v1 = TI->trapezoids.v[t.up0].right_segment;
      if (traversed_from == t.up1) {
        S32 new_monotone = SplitPolygonByDiagonal(
            VertexChainSliceFromArray(TI->vertex_chains),
            TI->monotone_polygon_chains, TI->monotone_chain_start_vertex,
            current_monotone, v1, v0);
        TraversePolygon(TI, current_monotone, t.up1, current_trapezoid, UP);
        TraversePolygon(TI, new_monotone, t.up0, current_trapezoid, UP);
      } else {
        S32 new_monotone = SplitPolygonByDiagonal(
            VertexChainSliceFromArray(TI->vertex_chains),
            TI->monotone_polygon_chains, TI->monotone_chain_start_vertex,
            current_monotone, v0, v1);
        TraversePolygon(TI, current_monotone, t.up0, current_trapezoid, UP);
        TraversePolygon(TI, new_monotone, t.up1, current_trapezoid, UP);
      }
    } else {
      TraversePolygon(TI, current_monotone, t.up0, current_trapezoid, UP);
      TraversePolygon(TI, current_monotone, t.up1, current_trapezoid, UP);
    }
  } else if (t.up0 && t.up1) {
    if (t.down0 && t.down1) {
      /* downward + upward cusps
       *  connect v0 and v1
       *     \    /
       *      (v1)
       *       || <- to insert
       *      (v0)
       *     /    \
       */
      S32 v0 = TI->trapezoids.v[t.down1].left_segment;
      S32 v1 = TI->trapezoids.v[t.up0].right_segment;
      if ((dir == UP && t.down1 == traversed_from) ||
          (dir == DOWN && t.up1 == traversed_from)) {
        S32 new_monotone = SplitPolygonByDiagonal(
            VertexChainSliceFromArray(TI->vertex_chains),
            TI->monotone_polygon_chains, TI->monotone_chain_start_vertex,
            current_monotone, v1, v0);
        TraversePolygon(TI, current_monotone, t.up1, current_trapezoid, UP);
        TraversePolygon(TI, current_monotone, t.down1, current_trapezoid, DOWN);
        TraversePolygon(TI, new_monotone, t.up0, current_trapezoid, UP);
        TraversePolygon(TI, new_monotone, t.down0, current_trapezoid, DOWN);
      } else {
        S32 new_monotone = SplitPolygonByDiagonal(
            VertexChainSliceFromArray(TI->vertex_chains),
            TI->monotone_polygon_chains, TI->monotone_chain_start_vertex,
            current_monotone, v0, v1);
        TraversePolygon(TI, current_monotone, t.up0, current_trapezoid, UP);
        TraversePolygon(TI, current_monotone, t.down0, current_trapezoid, DOWN);
        TraversePolygon(TI, new_monotone, t.up1, current_trapezoid, UP);
        TraversePolygon(TI, new_monotone, t.down1, current_trapezoid, DOWN);
      }
    } else {
      /* only downward cusp
       *  |     \    /
       *  |      (v0)
       *  |
       * (v1)
       */
      if (Coord2EqualTo(t.min_y,
                        TI->vertices.v[TI->segments.v[t.left_segment].v1])) {
        S32 v0 = TI->trapezoids.v[t.up0].right_segment;
        S32 v1 = TI->segments.v[t.left_segment].next;

        if (dir == DOWN && t.up0 == traversed_from) {
          S32 new_monotone = SplitPolygonByDiagonal(
              VertexChainSliceFromArray(TI->vertex_chains),
              TI->monotone_polygon_chains, TI->monotone_chain_start_vertex,
              current_monotone, v1, v0);
          TraversePolygon(TI, current_monotone, t.up0, current_trapezoid, UP);
          TraversePolygon(TI, new_monotone, t.down0, current_trapezoid, DOWN);
          TraversePolygon(TI, new_monotone, t.up1, current_trapezoid, UP);
          TraversePolygon(TI, new_monotone, t.down1, current_trapezoid, DOWN);
        } else {
          S32 new_monotone = SplitPolygonByDiagonal(
              VertexChainSliceFromArray(TI->vertex_chains),
              TI->monotone_polygon_chains, TI->monotone_chain_start_vertex,
              current_monotone, v0, v1);
          TraversePolygon(TI, current_monotone, t.up1, current_trapezoid, UP);
          TraversePolygon(TI, current_monotone, t.down0, current_trapezoid,
                          DOWN);
          TraversePolygon(TI, current_monotone, t.down1, current_trapezoid,
                          DOWN);
          TraversePolygon(TI, new_monotone, t.up0, current_trapezoid, UP);
        }
      } else {
        /*   \    /    |
         *    (v1)     |
         *             |
         *            (v0)
         */
        S32 v0 = t.right_segment;
        S32 v1 = TI->trapezoids.v[t.up0].right_segment;
        if (dir == DOWN && t.up1 == traversed_from) {
          S32 new_monotone = SplitPolygonByDiagonal(
              VertexChainSliceFromArray(TI->vertex_chains),
              TI->monotone_polygon_chains, TI->monotone_chain_start_vertex,
              current_monotone, v1, v0);
          TraversePolygon(TI, current_monotone, t.up1, current_trapezoid, UP);
          TraversePolygon(TI, new_monotone, t.down1, current_trapezoid, DOWN);
          TraversePolygon(TI, new_monotone, t.down0, current_trapezoid, DOWN);
          TraversePolygon(TI, new_monotone, t.up0, current_trapezoid, UP);
        } else {
          S32 new_monotone = SplitPolygonByDiagonal(
              VertexChainSliceFromArray(TI->vertex_chains),
              TI->monotone_polygon_chains, TI->monotone_chain_start_vertex,
              current_monotone, v0, v1);
          TraversePolygon(TI, current_monotone, t.up0, current_trapezoid, UP);
          TraversePolygon(TI, current_monotone, t.down0, current_trapezoid,
                          DOWN);
          TraversePolygon(TI, current_monotone, t.down1, current_trapezoid,
                          DOWN);
          TraversePolygon(TI, new_monotone, t.up1, current_trapezoid, UP);
        }
      }
    }
  } else if (t.up0 || t.up1) /* no downward cusp */
  {
    if (t.down0 && t.down1) /* only upward cusp */
    {
      if (Coord2EqualTo(t.max_y,
                        TI->vertices.v[TI->segments.v[t.left_segment].v0])) {
        /* (v1)
         *  |
         *  |
         *  |       (v0)
         *  |      /    \
         */
        S32 v0 = TI->trapezoids.v[t.down1].left_segment;
        S32 v1 = t.left_segment;
        if (!(dir == UP && traversed_from == t.down0)) {
          S32 new_monotone = SplitPolygonByDiagonal(
              VertexChainSliceFromArray(TI->vertex_chains),
              TI->monotone_polygon_chains, TI->monotone_chain_start_vertex,
              current_monotone, v1, v0);
          TraversePolygon(TI, current_monotone, t.up1, current_trapezoid, UP);
          TraversePolygon(TI, current_monotone, t.down1, current_trapezoid,
                          DOWN);
          TraversePolygon(TI, current_monotone, t.down0, current_trapezoid,
                          DOWN);
          TraversePolygon(TI, new_monotone, t.up0, current_trapezoid, UP);
        } else {
          S32 new_monotone = SplitPolygonByDiagonal(
              VertexChainSliceFromArray(TI->vertex_chains),
              TI->monotone_polygon_chains, TI->monotone_chain_start_vertex,
              current_monotone, v0, v1);
          TraversePolygon(TI, current_monotone, t.up0, current_trapezoid, UP);
          TraversePolygon(TI, new_monotone, t.down0, current_trapezoid, DOWN);
          TraversePolygon(TI, new_monotone, t.down1, current_trapezoid, DOWN);
          TraversePolygon(TI, new_monotone, t.up1, current_trapezoid, UP);
        }
      } else {
        //            (v1)
        //             |
        //             |
        //    (v0)     |
        //   /    \    |
        S32 v0 = TI->trapezoids.v[t.down1].left_segment;
        S32 v1 = TI->segments.v[t.right_segment].next;

        if (dir == UP && traversed_from == t.down1) {
          S32 new_monotone = SplitPolygonByDiagonal(
              VertexChainSliceFromArray(TI->vertex_chains),
              TI->monotone_polygon_chains, TI->monotone_chain_start_vertex,
              current_monotone, v1, v0);
          TraversePolygon(TI, current_monotone, t.down1, current_trapezoid,
                          DOWN);
          TraversePolygon(TI, new_monotone, t.up1, current_trapezoid, UP);
          TraversePolygon(TI, new_monotone, t.up0, current_trapezoid, UP);
          TraversePolygon(TI, new_monotone, t.down0, current_trapezoid, DOWN);
        } else {
          S32 new_monotone = SplitPolygonByDiagonal(
              VertexChainSliceFromArray(TI->vertex_chains),
              TI->monotone_polygon_chains, TI->monotone_chain_start_vertex,
              current_monotone, v0, v1);
          TraversePolygon(TI, current_monotone, t.up0, current_trapezoid, UP);
          TraversePolygon(TI, current_monotone, t.down0, current_trapezoid,
                          DOWN);
          TraversePolygon(TI, current_monotone, t.up1, current_trapezoid, UP);
          TraversePolygon(TI, new_monotone, t.down1, current_trapezoid, DOWN);
        }
      }
    } else { /* no cusp */
      S32 v0, v1;
      if (Coord2EqualTo(t.max_y,
                        TI->vertices.v[TI->segments.v[t.left_segment].v0]) &&
          Coord2EqualTo(t.min_y,
                        TI->vertices.v[TI->segments.v[t.right_segment].v0])) {
        //    (v1)
        //    /        |
        //   /         |
        //            (v0)
        v0 = t.right_segment;
        v1 = t.left_segment;
        if (dir == DOWN) {
          S32 new_monotone = SplitPolygonByDiagonal(
              VertexChainSliceFromArray(TI->vertex_chains),
              TI->monotone_polygon_chains, TI->monotone_chain_start_vertex,
              current_monotone, v1, v0);
          TraversePolygon(TI, current_monotone, t.up0, current_trapezoid, UP);
          TraversePolygon(TI, current_monotone, t.up1, current_trapezoid, UP);
          TraversePolygon(TI, new_monotone, t.down1, current_trapezoid, DOWN);
          TraversePolygon(TI, new_monotone, t.down0, current_trapezoid, DOWN);
        } else {
          S32 new_monotone = SplitPolygonByDiagonal(
              VertexChainSliceFromArray(TI->vertex_chains),
              TI->monotone_polygon_chains, TI->monotone_chain_start_vertex,
              current_monotone, v0, v1);
          TraversePolygon(TI, current_monotone, t.down1, current_trapezoid,
                          DOWN);
          TraversePolygon(TI, current_monotone, t.down0, current_trapezoid,
                          DOWN);
          TraversePolygon(TI, new_monotone, t.up0, current_trapezoid, UP);
          TraversePolygon(TI, new_monotone, t.up1, current_trapezoid, UP);
        }
      } else if (Coord2EqualTo(
                     t.max_y,
                     TI->vertices.v[TI->segments.v[t.right_segment].v1]) &&
                 Coord2EqualTo(
                     t.min_y,
                     TI->vertices.v[TI->segments.v[t.left_segment].v1])) {
        // go to the segment end if they are at opposite of one another.
        // same picture as above but with v1 of both segments
        v0 = TI->segments.v[t.right_segment].next;
        v1 = TI->segments.v[t.left_segment].next;
        if (dir == DOWN) {
          S32 new_monotone = SplitPolygonByDiagonal(
              VertexChainSliceFromArray(TI->vertex_chains),
              TI->monotone_polygon_chains, TI->monotone_chain_start_vertex,
              current_monotone, v1, v0);
          TraversePolygon(TI, current_monotone, t.up0, current_trapezoid, UP);
          TraversePolygon(TI, current_monotone, t.up1, current_trapezoid, UP);
          TraversePolygon(TI, new_monotone, t.down1, current_trapezoid, DOWN);
          TraversePolygon(TI, new_monotone, t.down0, current_trapezoid, DOWN);
        } else {
          S32 new_monotone = SplitPolygonByDiagonal(
              VertexChainSliceFromArray(TI->vertex_chains),
              TI->monotone_polygon_chains, TI->monotone_chain_start_vertex,
              current_monotone, v0, v1);
          TraversePolygon(TI, current_monotone, t.down1, current_trapezoid,
                          DOWN);
          TraversePolygon(TI, current_monotone, t.down0, current_trapezoid,
                          DOWN);
          TraversePolygon(TI, new_monotone, t.up0, current_trapezoid, UP);
          TraversePolygon(TI, new_monotone, t.up1, current_trapezoid, UP);
        }
      } else { /* no split possible */
        TraversePolygon(TI, current_monotone, t.up0, current_trapezoid, UP);
        TraversePolygon(TI, current_monotone, t.down0, current_trapezoid, DOWN);
        TraversePolygon(TI, current_monotone, t.up1, current_trapezoid, UP);
        TraversePolygon(TI, current_monotone, t.down1, current_trapezoid, DOWN);
      }
    }
  }
#ifdef DEBUG
  ValidateMonotoneChains(
      MonotoneChainSliceFromArray(TI->monotone_polygon_chains));
#endif
}

// compute the diamond angle respective to the x-axis. see
// https://www.freesteel.co.uk/wpblog/2009/06/05/encoding-2d-angles-without-trigonometry
static F64 DiamondAngle(F64 dx, F64 dy) {
  if (dy >= 0)
    return dx >= 0 ? dy / (dx + dy) : 1 - dx / (-dx + dy);
  else
    return dx < 0 ? 2 - dy / (-dx - dy) : 3 + dx / (dx - dy);
}

// computes the diamond angle between two vectors v0 -> v1, v0 -> vnext; where
// the vector v0 -> v1 acts as the base or the x-coordinate of the diamond angle
static F64 DiamondAngleBetweenVectors(Coord2 vp0, Coord2 vpnext, Coord2 vp1) {
  F64 base = DiamondAngle(vp1.x - vp0.x, vp1.y - vp0.y);
  F64 next = DiamondAngle(vpnext.x - vp0.x, vpnext.y - vp0.y);
  if (base < next) {
    return next - base;
  } else {
    return next - base + 4.0f;
  }
}

/* (v0, v1) is the new diagonal to be added to the polygon. Find which */
/* chain to use and return the positions of v0 and v1 in p and q */
// finds the segments with the smallest angle clockwise starting
// from v0 and v1 respectively to the diagonal to be added.
//     |  minimize this angle ->   \  /
//    (v0)- - - - - - - - - - - - -(v1)
//    / \ <- minimize this angle     |
static void NextVertexIndexForMonotoneChain(VertexChainSlice vertex_chains,
                                            S32 v0, S32 v1, S32 *ip, S32 *iq) {
  S32 tp = 0, tq = 0;
  const VertexChain vp0 = vertex_chains.v[v0];
  const VertexChain vp1 = vertex_chains.v[v1];

  /* p is identified as follows. Scan from (v0, v1) rightwards till */
  /* you hit the first segment starting from v0. That chain is the */
  /* chain of our interest */
  F64 angle = -1.0;
  F64 temp;
  bool found = false;
  for (S32 i = 0; i < 4; i++) {
    if (vp0.next_vertex[i] <= 0)
      continue;
    if ((temp = DiamondAngleBetweenVectors(
             vp0.pt, vertex_chains.v[vp0.next_vertex[i]].pt, vp1.pt)) > angle) {
      angle = temp;
      tp = i;
      found = true;
    }
  }
  ASSERT(found, "found no vertex in vertex chain");
  *ip = tp;

  /* Do similar actions for q */
  angle = -1.0;
  found = false;
  for (S32 i = 0; i < 4; i++) {
    if (vp1.next_vertex[i] <= 0)
      continue;
    if ((temp = DiamondAngleBetweenVectors(
             vp1.pt, vertex_chains.v[vp1.next_vertex[i]].pt, vp0.pt)) > angle) {
      angle = temp;
      tq = i;
      found = true;
    }
  }
  ASSERT(found, "found no vertex in vertex chain");
  *iq = tq;
}

/* v0 and v1 are specified in anti-clockwise order with respect to
 * the current monotone polygon mcur. Split the current polygon into
 * two polygons using the diagonal (v0, v1)
 * Returns the index of the newly created monotone polygon
 */
static S32 SplitPolygonByDiagonal(VertexChainSlice vertex_chains,
                                  MonotoneChainArray *monotone_polygon_chains,
                                  S32Array *monotone_chain_start_vertex,
                                  S32 current_monotone_polygon, S32 v0,
                                  S32 v1) {
  S32 ip, iq;

  VertexChain *vp0 = &vertex_chains.v[v0];
  VertexChain *vp1 = &vertex_chains.v[v1];

  NextVertexIndexForMonotoneChain(vertex_chains, v0, v1, &ip, &iq);

  const int p = vp0->vertex_index_in_monotone_chain[ip];
  const int q = vp1->vertex_index_in_monotone_chain[iq];

  /* At this stage, we have got the positions of v0 and v1 in the */
  /* desired chain. Now modify the linked lists */
  // adds two new points into the linked list i, j and connects them to the old
  // points which are p, q respectively.
  // before:
  // (p.prev)  (q.next)
  //     |         |
  //    (p)- - - -(q)
  //     |         |
  // (p.next)  (q.prev)
  //
  // after:
  // (p.prev) (q.next)
  //     |        |
  //    (p)<---->(q) (p, i) and (q, j) are at the same point
  //    (i)<---->(j)
  //     |        |
  // (i.next) (j.prev)

  const S32 p_next = monotone_polygon_chains->data[p].next;
  const S32 q_prev = monotone_polygon_chains->data[q].prev;
  // for the new list
  const S32 i =
      MonotoneChainArrayPush(monotone_polygon_chains, (MonotoneChain){0});
  const S32 j =
      MonotoneChainArrayPush(monotone_polygon_chains, (MonotoneChain){0});
  monotone_polygon_chains->data[p_next].prev = i;
  monotone_polygon_chains->data[q_prev].next = j;
  monotone_polygon_chains->data[i] =
      (MonotoneChain){.vertex = v0, .next = p_next, .prev = j};
  monotone_polygon_chains->data[j] =
      (MonotoneChain){.vertex = v1, .next = i, .prev = q_prev};

  monotone_polygon_chains->data[p].next = q;
  monotone_polygon_chains->data[q].prev = p;

  const S32 nf0 = vp0->next_free;
  const S32 nf1 = vp1->next_free;

  vp0->next_vertex[ip] = v1;
  vp0->vertex_index_in_monotone_chain[nf0] = i;
  vp0->next_vertex[nf0] =
      monotone_polygon_chains->data[monotone_polygon_chains->data[i].next]
          .vertex;
  vp1->vertex_index_in_monotone_chain[nf1] = j;
  vp1->next_vertex[nf1] = v0;

  vp0->next_free += 1;
  vp1->next_free += 1;

#ifdef DEBUG
  fprintf(stderr,
          "make_poly: current_monotone_polygon = %d, (v0, v1) = (%d, %d)\n",
          current_monotone_polygon, v0, v1);
  fprintf(stderr, "next posns = (p, q) = (%d, %d)\n", p, q);
#endif

  monotone_chain_start_vertex->data[current_monotone_polygon] = p;
  const S32 new_monotone = S32ArrayPush(monotone_chain_start_vertex, i);
#ifdef DEBUG
  ValidateMonotoneAndVertexChains(
      vertex_chains, MonotoneChainSliceFromArray(monotone_polygon_chains));
#endif
  return new_monotone;
}

static void ConstructTrapezoidation(QueryNodeArray *query_structure,
                                    Coord2Slice vertices,
                                    SegmentArray *segments,
                                    TrapezoidArray *trapezoids,
                                    S32Array *permutation) {
  // Add the first segment and get the query structure and trapezoid list
  // initialized
#ifdef DEBUG
  ValidateQueryStructure(QueryNodeSliceFromArray(query_structure));
#endif
  const S32 segment_count = segments->len - 1;
  S32 query_root = InitQueryStructure(query_structure, vertices, trapezoids,
                                      segments, RandomSegment(permutation));
  for (S32 i = 1; i <= segment_count; i++) {
    segments->data[i].root0 = segments->data[i].root1 = query_root;
  }
#ifdef DEBUG
  ValidateQueryTrapezoidSegmentStructures(
      QueryNodeSliceFromArray(query_structure),
      TrapezoidSliceFromArray(trapezoids), SegmentSliceFromArray(segments),
      vertices);
#endif

  for (S32 h = 1; h <= MathLogStar(segment_count); h++) {
    for (S32 i = MathN(segment_count, h - 1) + 1; i <= MathN(segment_count, h);
         i++) {
#ifdef DEBUG
      ValidateQueryTrapezoidSegmentStructures(
          QueryNodeSliceFromArray(query_structure),
          TrapezoidSliceFromArray(trapezoids), SegmentSliceFromArray(segments),
          vertices);
#endif
      AddSegment(query_structure, vertices, SegmentSliceFromArray(segments),
                 trapezoids, RandomSegment(permutation));
#ifdef DEBUG
      ValidateQueryTrapezoidSegmentStructures(
          QueryNodeSliceFromArray(query_structure),
          TrapezoidSliceFromArray(trapezoids), SegmentSliceFromArray(segments),
          vertices);
#endif
    }

    /* Find a new root for each of the segment endpoints */
    for (S32 i = 1; i <= segment_count; i++) {
      Segment *s = &segments->data[i];

      if (!s->is_inserted) {
        S32 endpoint = TrapezoidIndexFromVertex(
            QueryNodeSliceFromArray(query_structure),
            SegmentSliceFromArray(segments), vertices, s->v0, s->v1, s->root0);
        s->root0 = trapezoids->data[endpoint].sink_node;

        endpoint = TrapezoidIndexFromVertex(
            QueryNodeSliceFromArray(query_structure),
            SegmentSliceFromArray(segments), vertices, s->v1, s->v0, s->root1);
        s->root1 = trapezoids->data[endpoint].sink_node;
      }
    }
  }
#ifdef DEBUG
  ValidateQueryTrapezoidSegmentStructures(
      QueryNodeSliceFromArray(query_structure),
      TrapezoidSliceFromArray(trapezoids), SegmentSliceFromArray(segments),
      vertices);
#endif

  for (S32 i = MathN(segment_count, MathLogStar(segment_count)) + 1;
       i <= segment_count; i++) {
#ifdef DEBUG
    ValidateQueryTrapezoidSegmentStructures(
        QueryNodeSliceFromArray(query_structure),
        TrapezoidSliceFromArray(trapezoids), SegmentSliceFromArray(segments),
        vertices);
#endif
    AddSegment(query_structure, vertices, SegmentSliceFromArray(segments),
               trapezoids, RandomSegment(permutation));
#ifdef DEBUG
    ValidateQueryTrapezoidSegmentStructures(
        QueryNodeSliceFromArray(query_structure),
        TrapezoidSliceFromArray(trapezoids), SegmentSliceFromArray(segments),
        vertices);
#endif
  }
}

// locates vertex in the query structure, starting the search at root.
// returns the trapezoid index the node is currently located in.
static S32 TrapezoidIndexFromVertex(QueryNodeSlice query_structure,
                                    SegmentSlice segments, Coord2Slice vertices,
                                    S32 vertex, S32 segment_endpoint,
                                    S32 root) {
  QueryNode root_node = query_structure.v[root];

  while (true) {
    switch (root_node.node_type) {
    case SINK:
      return root_node.trapezoid;

    case Y:
      if (Coord2GreaterThan(vertices.v[vertex], root_node.yval)) {
        // above
        root_node = query_structure.v[root_node.right];
      } else if (Coord2EqualTo(vertices.v[vertex], root_node.yval)) {
        // the point is already inserted
        if (Coord2GreaterThan(vertices.v[segment_endpoint], root_node.yval)) {
          // above
          root_node = query_structure.v[root_node.right];
        } else {
          // below
          root_node = query_structure.v[root_node.left];
        }
      } else {
        // below
        root_node = query_structure.v[root_node.left];
      }
      break;

    case X:
      if (Coord2EqualTo(vertices.v[vertex],
                        vertices.v[segments.v[root_node.segment].v0]) ||
          Coord2EqualTo(vertices.v[vertex],
                        vertices.v[segments.v[root_node.segment].v1])) {
        // horizontal segment
        if (FP_EQUAL(vertices.v[vertex].y, vertices.v[segment_endpoint].y)) {
          if (vertices.v[segment_endpoint].x < vertices.v[vertex].x) {
            root_node = query_structure.v[root_node.left];
          } else {
            root_node = query_structure.v[root_node.right];
          }
        } else if (VertexLeftOfSegment(vertices, segments, root_node.segment,
                                       segment_endpoint)) {
          root_node = query_structure.v[root_node.left];
        } else {
          root_node = query_structure.v[root_node.right];
        }
      } else if (VertexLeftOfSegment(vertices, segments, root_node.segment,
                                     vertex)) {
        root_node = query_structure.v[root_node.left];
      } else {
        root_node = query_structure.v[root_node.right];
      }
      break;

    default:
      ERROR_MSG("Node has no type and can not be part of location query!\n")
    }
  }
}

// adds a vertex to the query structure and updates the trapezoidation.
// returns the first index of the affected trapezoid
// returns the lower trapezoid if this is the first vertex inserted
// the upper trapezoid if it was the second (see first_vertex)
static S32 AddVertex(QueryNodeArray *query_structure, Coord2Slice vertices,
                     SegmentSlice segments, TrapezoidArray *trapezoids, S32 v0,
                     S32 v1, S32 v0_root, bool first_vertex) {

  const S32 old_upper_trapezoid =
      TrapezoidIndexFromVertex(QueryNodeSliceFromArray(query_structure),
                               segments, vertices, v0, v1, v0_root);
  // tl is the new lower trapezoid
  const S32 new_lower_trapezoid =
      TrapezoidArrayPush(trapezoids, trapezoids->data[old_upper_trapezoid]);
  trapezoids->data[new_lower_trapezoid].is_valid = true;
  trapezoids->data[new_lower_trapezoid].up0 = old_upper_trapezoid;
  trapezoids->data[new_lower_trapezoid].up1 = 0;
  trapezoids->data[old_upper_trapezoid].min_y =
      trapezoids->data[new_lower_trapezoid].max_y = vertices.v[v0];
  trapezoids->data[old_upper_trapezoid].down0 = new_lower_trapezoid;
  trapezoids->data[old_upper_trapezoid].down1 = 0;

  // update neighboring information of nearby trapezoids (if existing)
  S32 neighbor = trapezoids->data[new_lower_trapezoid].down0;
  if (neighbor) {
    if (trapezoids->data[neighbor].up0 == old_upper_trapezoid) {
      trapezoids->data[neighbor].up0 = new_lower_trapezoid;
    }
    if (trapezoids->data[neighbor].up1 == old_upper_trapezoid) {
      trapezoids->data[neighbor].up1 = new_lower_trapezoid;
    }
  }

  neighbor = trapezoids->data[new_lower_trapezoid].down1;
  if (neighbor) {
    if (trapezoids->data[neighbor].up0 == old_upper_trapezoid) {
      trapezoids->data[neighbor].up0 = new_lower_trapezoid;
    }
    if (trapezoids->data[neighbor].up1 == old_upper_trapezoid) {
      trapezoids->data[neighbor].up1 = new_lower_trapezoid;
    }
  }

  /* Now update the query structure and obtain the sinks for the  two trapezoids
   *      Y(v0)
   *      /    \
   *     /      \
   *  SINK(tl) SINK(tu)
   */
  S32 upper_sink_node = QueryNodeArrayPush(query_structure, (QueryNode){0});
  S32 lower_sink_node = QueryNodeArrayPush(query_structure, (QueryNode){0});
  S32 old_sink = trapezoids->data[old_upper_trapezoid].sink_node;

  // update the existing node (old sink)
  query_structure->data[old_sink].node_type = Y;
  query_structure->data[old_sink].yval = vertices.v[v0];
  query_structure->data[old_sink].left = lower_sink_node;
  query_structure->data[old_sink].right = upper_sink_node;

  query_structure->data[upper_sink_node] = (QueryNode){
      .node_type = SINK, .trapezoid = old_upper_trapezoid, .parent = old_sink};
  query_structure->data[lower_sink_node] = (QueryNode){
      .node_type = SINK, .trapezoid = new_lower_trapezoid, .parent = old_sink};

  trapezoids->data[old_upper_trapezoid].sink_node = upper_sink_node;
  trapezoids->data[new_lower_trapezoid].sink_node = lower_sink_node;
  return first_vertex ? new_lower_trapezoid : old_upper_trapezoid;
}

// adds a segment to the query structure and updates the trapezoidation.
static void AddSegment(QueryNodeArray *query_structure, Coord2Slice vertices,
                       SegmentSlice segments, TrapezoidArray *trapezoids,
                       S32 segment) {
  bool tribot = false;
  bool is_swapped = false;
  Segment s = segments.v[segment];
  S32 first_trapezoid_right = 0;
  S32 last_trapezoid_right = 0;

  // swap v0, v1 such that v0 is higher than v1
  bool v0_inserted, v1_inserted;
  if (Coord2GreaterThan(vertices.v[s.v1], vertices.v[s.v0])) {
    S32 tmp = s.v0;
    s.v0 = s.v1;
    s.v1 = tmp;
    tmp = s.root0;
    s.root0 = s.root1;
    s.root1 = tmp;
    is_swapped = true;
    v0_inserted = segments.v[segments.v[segment].next].is_inserted;
    v1_inserted = segments.v[segments.v[segment].prev].is_inserted;
  } else {
    v0_inserted = segments.v[segments.v[segment].prev].is_inserted;
    v1_inserted = segments.v[segments.v[segment].next].is_inserted;
  }

  // defines the top and bottom most trapezoids affected by threading this
  // segment through the trapezoidation structure
  // -----------(v0)--------------
  //     top_trapezoid
  // -----------------------------
  //             .
  //             .
  //             .
  // -----------------------------
  //     bottom_trapezoid
  // -----------(v1)--------------
  S32 top_trapezoid, bottom_trapezoid;
  if (!v0_inserted) {
    top_trapezoid = AddVertex(query_structure, vertices, segments, trapezoids,
                              s.v0, s.v1, s.root0, true);
  } else {
    top_trapezoid =
        TrapezoidIndexFromVertex(QueryNodeSliceFromArray(query_structure),
                                 segments, vertices, s.v0, s.v1, s.root0);
  }
  if (!v1_inserted) {
    bottom_trapezoid = AddVertex(query_structure, vertices, segments,
                                 trapezoids, s.v1, s.v0, s.root1, false);
  } else {
    bottom_trapezoid =
        TrapezoidIndexFromVertex(QueryNodeSliceFromArray(query_structure),
                                 segments, vertices, s.v1, s.v0, s.root1);
    tribot = true;
  }
#ifdef DEBUG
  ValidateQueryTrapezoidSegmentStructures(
      QueryNodeSliceFromArray(query_structure),
      TrapezoidSliceFromArray(trapezoids), segments, vertices);
#endif

  // Thread the segment into the query tree creating a new X-node First, split
  // all the trapezoids which are intersected by s into two

  S32 t = top_trapezoid;
  const S32 first_trapezoid_left = top_trapezoid;
  const S32 last_trapezoid_left = bottom_trapezoid;
  // traverse from top to bot via the chain the trapezoids form
  while (t &&
         Coord2GreaterThanEqualTo(trapezoids->data[t].min_y,
                                  trapezoids->data[bottom_trapezoid].min_y)) {
    S32 sk = trapezoids->data[t].sink_node;
    S32 new_trapezoid =
        TrapezoidArrayPush(trapezoids, (Trapezoid){.is_valid = true});
    // left trapezoid sink
    S32 left_sink_node = QueryNodeArrayPush(
        query_structure,
        (QueryNode){.node_type = SINK, .trapezoid = t, .parent = sk});
    // right trapezoid sink
    S32 right_sink_node = QueryNodeArrayPush(
        query_structure, (QueryNode){.node_type = SINK,
                                     .trapezoid = new_trapezoid,
                                     .parent = sk});

    query_structure->data[sk].node_type = X;
    query_structure->data[sk].segment = segment;
    query_structure->data[sk].left = left_sink_node;
    query_structure->data[sk].right = right_sink_node;

    if (t == top_trapezoid)
      first_trapezoid_right = new_trapezoid;
    if (Coord2EqualTo(trapezoids->data[t].min_y,
                      trapezoids->data[bottom_trapezoid].min_y))
      last_trapezoid_right = new_trapezoid;

    trapezoids->data[new_trapezoid] = trapezoids->data[t];
    trapezoids->data[t].sink_node = left_sink_node;
    trapezoids->data[new_trapezoid].sink_node = right_sink_node;
    S32 t_sav = t;
    S32 tn_sav = new_trapezoid;

    // error case: no trapezoid below, cannot arise
    if (!(trapezoids->data[t].down0 || trapezoids->data[t].down1)) {
      ERROR_MSG("AddSegment: error, no lower trapezoid exists\n");
    }
    if (!(trapezoids->data[t].up0 || trapezoids->data[t].up1)) {
      ERROR_MSG(
          "AddSegment: error no upper trapezoid of the one to split exists, "
          "v0.y: %f, v1.y: %f, t.min_y: %f, FP_EQ(v0, t.min_y)?: %d",
          vertices.v[s.v0].y, vertices.v[s.v1].y, trapezoids->data[t].min_y.y,
          Coord2EqualTo(vertices.v[s.v0], trapezoids->data[t].min_y));
    }

    // only one trapezoid below. partition t into two and make the two resulting
    // trapezoids t and tn as the upper neighbors of the sole lower trapezoid
    if (trapezoids->data[t].down1 && !trapezoids->data[t].down0) {
      // merge the case where we have the sole neighbor in down1
      trapezoids->data[t].down0 = trapezoids->data[t].down1;
      trapezoids->data[t].down1 = 0;
      trapezoids->data[new_trapezoid].down0 =
          trapezoids->data[new_trapezoid].down1;
      trapezoids->data[new_trapezoid].down1 = 0;
    }
    if (trapezoids->data[t].down0 && !trapezoids->data[t].down1) {
      // continuation of a chain from above
      if (trapezoids->data[t].up0 && trapezoids->data[t].up1) {
        // three upper neighbors
        if (trapezoids->data[t].usave) {
          if (trapezoids->data[t].uside == LEFT) {
            /* usave \ t.up0 \  /  t.up1
             * -------\-------()-------------
             *         \
             *          \
             */
            trapezoids->data[new_trapezoid].up0 = trapezoids->data[t].up1;
            trapezoids->data[t].up1 = 0;
            trapezoids->data[new_trapezoid].up1 = trapezoids->data[t].usave;

            trapezoids->data[trapezoids->data[t].up0].down0 = t;
            trapezoids->data[trapezoids->data[new_trapezoid].up0].down0 =
                new_trapezoid;
            trapezoids->data[trapezoids->data[new_trapezoid].up1].down0 =
                new_trapezoid;
            /* t.up0 \ new.up0 \  /  new.up1
             * ------ \---^v----()-----^v------
             *    t    \   new_trapezoid
             *          \
             */
          } else {
            /*  t.up0   \  /   t.up1     /  t.usave
             * ----------()-------------/----
             *                         /
             *                        /
             */
            trapezoids->data[new_trapezoid].up1 = 0;
            trapezoids->data[new_trapezoid].up0 = trapezoids->data[t].up1;
            trapezoids->data[t].up1 = trapezoids->data[t].up0;
            trapezoids->data[t].up0 = trapezoids->data[t].usave;

            trapezoids->data[trapezoids->data[t].up0].down0 = t;
            trapezoids->data[trapezoids->data[t].up1].down0 = t;
            trapezoids->data[trapezoids->data[new_trapezoid].up0].down0 =
                new_trapezoid;
            /*  t.up0   \  /   t.up1     / newt.up0
             * ----^v----()----^v-------/----
             *                         /
             *          t             /  new_trapezoid
             */
          }

          trapezoids->data[t].usave = trapezoids->data[new_trapezoid].usave = 0;
          trapezoids->data[t].uside = trapezoids->data[new_trapezoid].uside = 0;
        } else {
          //  t.u0   \   t.u1
          // -------- \ ---------
          //    t      :  new
          // --------------------
          //         t.d0
          trapezoids->data[new_trapezoid].up0 = trapezoids->data[t].up1;
          trapezoids->data[t].up1 = trapezoids->data[new_trapezoid].up1 = 0;
          trapezoids->data[trapezoids->data[new_trapezoid].up0].down0 =
              new_trapezoid;
          // neigboring information updated below
        }
      } else { /* fresh seg. or upward cusp */
        S32 t_up0 = trapezoids->data[t].up0;
        S32 td0 = trapezoids->data[t_up0].down0;
        S32 td1 = trapezoids->data[t_up0].down1;
        if (td0 && td1) {
          /*      upward cusp
           *          t.u0
           * ---v----------v^----
           *         / \  new
           *        / t \
           * --------------------
           */
          if (trapezoids->data[td0].right_segment &&
              !VertexLeftOfSegment(vertices, segments,
                                   trapezoids->data[td0].right_segment, s.v1)) {
            trapezoids->data[t].up0 = trapezoids->data[t].up1 =
                trapezoids->data[new_trapezoid].up1 = 0;
            trapezoids->data[trapezoids->data[new_trapezoid].up0].down1 =
                new_trapezoid;
          } else {
            /*    cusp going leftwards
             *        t.u0
             * ----v---------------
             *         /   \
             *    t   / new \
             * --------------------
             */
            trapezoids->data[new_trapezoid].up0 =
                trapezoids->data[new_trapezoid].up1 = trapezoids->data[t].up1 =
                    0;
            trapezoids->data[trapezoids->data[t].up0].down0 = t;
          }
        } else {
          /*  fresh segment
           *       t.u0
           *     0        1
           *-----|--------| -----
           *     v   \    v
           *     t    \  new
           */
          trapezoids->data[trapezoids->data[t].up0].down0 = t;
          trapezoids->data[trapezoids->data[t].up0].down1 = new_trapezoid;
        }
      }

      // bottom forms a triangle
      if (Coord2EqualTo(trapezoids->data[t].min_y,
                        trapezoids->data[bottom_trapezoid].min_y) &&
          tribot) {
        S32 tmptriseg;
        if (is_swapped)
          tmptriseg = segments.v[segment].prev;
        else
          tmptriseg = segments.v[segment].next;

        if (tmptriseg &&
            VertexLeftOfSegment(vertices, segments, tmptriseg, s.v0)) {
          // L-R downward cusp
          //        \ new /
          //         s   tmptriseg
          //     t    \ /
          // ----^---------------
          //    t.d0
          trapezoids->data[trapezoids->data[t].down0].up0 = t;
          trapezoids->data[new_trapezoid].down0 =
              trapezoids->data[new_trapezoid].down1 = 0;
        } else {
          // R-L downward cusp
          //        \ t   /
          //   tmptriseg s
          //          \ /  new
          // ----^----------^----
          //          t.d0
          trapezoids->data[trapezoids->data[new_trapezoid].down0].up1 =
              new_trapezoid;
          trapezoids->data[t].down0 = trapezoids->data[t].down1 = 0;
        }
      } else {
        const S32 down0_of_t = trapezoids->data[t].down0;
        if (trapezoids->data[down0_of_t].up0 &&
            trapezoids->data[down0_of_t].up1) {
          // passes through LHS
          if (trapezoids->data[down0_of_t].up0 == t) {
            trapezoids->data[down0_of_t].usave =
                trapezoids->data[down0_of_t].up1;
            trapezoids->data[down0_of_t].uside = LEFT;
          } else {
            trapezoids->data[down0_of_t].usave =
                trapezoids->data[down0_of_t].up0;
            trapezoids->data[down0_of_t].uside = RIGHT;
          }
        }
        trapezoids->data[down0_of_t].up0 = t;
        trapezoids->data[down0_of_t].up1 = new_trapezoid;
      }

      t = trapezoids->data[t].down0;
    } else {
      /* two trapezoids below. Find out which one is intersected by */
      /* this segment and proceed down that one */
      int next_trapezoid;

      int i_d0 = false;
      if (FP_EQUAL(trapezoids->data[t].min_y.y, vertices.v[s.v0].y)) {
        if (trapezoids->data[t].min_y.x > vertices.v[s.v0].x)
          i_d0 = true;
      } else {
        Coord2 tmppt;
        F64 y0;
        tmppt.y = y0 = trapezoids->data[t].min_y.y;
        F64 yt = (y0 - vertices.v[s.v0].y) /
                 (vertices.v[s.v1].y - vertices.v[s.v0].y);
        tmppt.x =
            vertices.v[s.v0].x + yt * (vertices.v[s.v1].x - vertices.v[s.v0].x);

        if (Coord2LessThan(tmppt, trapezoids->data[t].min_y))
          i_d0 = true;
      }

      /* check continuity from the top so that the lower-neighbor */
      /* values are properly filled for the upper trapezoid */

      if (trapezoids->data[t].up0 &&
          trapezoids->data[t].up1) {   /* continuation of a chain from abv. */
        if (trapezoids->data[t].usave) /* three upper neighbours */
        {
          if (trapezoids->data[t].uside == LEFT) {
            trapezoids->data[new_trapezoid].up0 = trapezoids->data[t].up1;
            trapezoids->data[t].up1 = 0;
            trapezoids->data[new_trapezoid].up1 = trapezoids->data[t].usave;

            trapezoids->data[trapezoids->data[t].up0].down0 = t;
            trapezoids->data[trapezoids->data[new_trapezoid].up0].down0 =
                new_trapezoid;
            trapezoids->data[trapezoids->data[new_trapezoid].up1].down0 =
                new_trapezoid;
          } else /* intersects in the right */
          {
            trapezoids->data[new_trapezoid].up1 = 0;
            trapezoids->data[new_trapezoid].up0 = trapezoids->data[t].up1;
            trapezoids->data[t].up1 = trapezoids->data[t].up0;
            trapezoids->data[t].up0 = trapezoids->data[t].usave;

            trapezoids->data[trapezoids->data[t].up0].down0 = t;
            trapezoids->data[trapezoids->data[t].up1].down0 = t;
            trapezoids->data[trapezoids->data[new_trapezoid].up0].down0 =
                new_trapezoid;
          }

          trapezoids->data[t].usave = trapezoids->data[new_trapezoid].usave = 0;
        } else {
          /* No usave.... simple case */
          trapezoids->data[new_trapezoid].up0 = trapezoids->data[t].up1;
          trapezoids->data[new_trapezoid].up1 = 0;
          trapezoids->data[t].up1 = 0;
          trapezoids->data[trapezoids->data[new_trapezoid].up0].down0 =
              new_trapezoid;
        }
      } else { /* fresh seg. or upward cusp */
        S32 t_up0 = trapezoids->data[t].up0;
        S32 td0;
        if ((td0 = trapezoids->data[t_up0].down0) &&
            trapezoids->data[t_up0].down1) { /* upward cusp */
          if (trapezoids->data[td0].right_segment &&
              !VertexLeftOfSegment(vertices, segments,
                                   trapezoids->data[td0].right_segment, s.v1)) {
            trapezoids->data[t].up0 = trapezoids->data[t].up1 =
                trapezoids->data[new_trapezoid].up1 = 0;
            trapezoids->data[trapezoids->data[new_trapezoid].up0].down1 =
                new_trapezoid;
          } else {
            trapezoids->data[new_trapezoid].up0 =
                trapezoids->data[new_trapezoid].up1 = trapezoids->data[t].up1 =
                    0;
            trapezoids->data[trapezoids->data[t].up0].down0 = t;
          }
        } else /* fresh segment */
        {
          trapezoids->data[trapezoids->data[t].up0].down0 = t;
          trapezoids->data[trapezoids->data[t].up0].down1 = new_trapezoid;
        }
      }

      if (Coord2EqualTo(trapezoids->data[t].min_y,
                        trapezoids->data[bottom_trapezoid].min_y) &&
          tribot) {
        /* this case arises only at the lowest trapezoid.. i.e.
           tlast, if the lower endpoint of the segment is
           already inserted in the structure */

        trapezoids->data[trapezoids->data[t].down0].up0 = t;
        trapezoids->data[trapezoids->data[t].down0].up1 = 0;
        trapezoids->data[trapezoids->data[t].down1].up0 = new_trapezoid;
        trapezoids->data[trapezoids->data[t].down1].up1 = 0;

        trapezoids->data[new_trapezoid].down0 = trapezoids->data[t].down1;
        trapezoids->data[t].down1 = trapezoids->data[new_trapezoid].down1 = 0;

        next_trapezoid = trapezoids->data[t].down1;
      } else if (i_d0)
      /* intersecting d0 */
      {
        trapezoids->data[trapezoids->data[t].down0].up0 = t;
        trapezoids->data[trapezoids->data[t].down0].up1 = new_trapezoid;
        trapezoids->data[trapezoids->data[t].down1].up0 = new_trapezoid;
        trapezoids->data[trapezoids->data[t].down1].up1 = 0;

        /* new code to determine the bottom neighbors of the */
        /* newly partitioned trapezoid */

        trapezoids->data[t].down1 = 0;

        next_trapezoid = trapezoids->data[t].down0;
      } else /* intersecting d1 */
      {
        trapezoids->data[trapezoids->data[t].down0].up0 = t;
        trapezoids->data[trapezoids->data[t].down0].up1 = 0;
        trapezoids->data[trapezoids->data[t].down1].up0 = t;
        trapezoids->data[trapezoids->data[t].down1].up1 = new_trapezoid;

        /* new code to determine the bottom neighbors of the */
        /* newly partitioned trapezoid */

        trapezoids->data[new_trapezoid].down0 = trapezoids->data[t].down1;
        trapezoids->data[new_trapezoid].down1 = 0;

        next_trapezoid = trapezoids->data[t].down1;
      }

      t = next_trapezoid;
    }

    trapezoids->data[t_sav].right_segment =
        trapezoids->data[tn_sav].left_segment = segment;
  }

  /* Now combine those trapezoids which share common segments. We can */
  /* use the pointers to the parent to connect these together. This */
  /* works only because all these new trapezoids have been formed */
  /* due to splitting by the segment, and hence have only one parent */
  ASSERT(first_trapezoid_right > 0 && first_trapezoid_right < trapezoids->len,
         "First Trapezoid right of segment undefined")
  ASSERT(last_trapezoid_right > 0 && last_trapezoid_right < trapezoids->len,
         "Last Trapezoid right of segment undefined")

  segments.v[segment].is_inserted = true;
#ifdef DEBUG
  ValidateQueryTrapezoidSegmentStructures(
      QueryNodeSliceFromArray(query_structure),
      TrapezoidSliceFromArray(trapezoids), segments, vertices);
#endif
  MergeTrapezoids(QueryNodeSliceFromArray(query_structure), trapezoids, segment,
                  first_trapezoid_left, last_trapezoid_left, LEFT);
  MergeTrapezoids(QueryNodeSliceFromArray(query_structure), trapezoids, segment,
                  first_trapezoid_right, last_trapezoid_right, RIGHT);

  // TODO: figure out how to use this invariant.
  // currently it fails if a segment adjacent to a already bounding segment is inserted
  // that is collinear (?)
//#ifdef DEBUG
//  for (S32 i = 1; i < trapezoids->len; i += 1) {
//    const Trapezoid t = trapezoids->data[i];
//    if (!t.is_valid) {
//      continue;
//    }
//    if (t.left_segment && t.right_segment && abs(t.left_segment - segment) > 1 && abs(t.right_segment - segment) > 1) {
//      Coord2 left_v0 = vertices.v[segments.v[t.left_segment].v0];
//      Coord2 left_v1 = vertices.v[segments.v[t.left_segment].v1];
//      Coord2 right_v0 = vertices.v[segments.v[t.right_segment].v0];
//      Coord2 right_v1 = vertices.v[segments.v[t.right_segment].v1];
//      bool cond = false;
//      if (!(t.down0 && t.down1)) {
//        cond |= Coord2EqualTo(left_v0, t.min_y) ||
//                Coord2EqualTo(left_v1, t.min_y) ||
//                Coord2EqualTo(right_v0, t.min_y) ||
//                Coord2EqualTo(right_v1, t.min_y);
//      }
//      if (!(t.up0 && t.up1)) {
//        cond |= Coord2EqualTo(left_v0, t.max_y) ||
//                Coord2EqualTo(left_v1, t.max_y) ||
//                Coord2EqualTo(right_v0, t.max_y) ||
//                Coord2EqualTo(right_v1, t.max_y);
//      }
//      ASSERT(cond, "max/min of trapezoid %d do not match any segment endpoint",
//             i)
//    }
//  }
//#endif
#ifdef DEBUG
  ValidateQueryTrapezoidSegmentStructures(
      QueryNodeSliceFromArray(query_structure),
      TrapezoidSliceFromArray(trapezoids), segments, vertices);
#endif
}

/* Thread in the segment into the existing trapezoidation. The
 * limiting trapezoids are given by tfirst and tlast (which are the
 * trapezoids containing the two endpoints of the segment. Merges all
 * possible trapezoids which flank this segment and have been recently
 * divided because of its insertion
 */
static void MergeTrapezoids(QueryNodeSlice query_structure,
                            TrapezoidArray *trapezoids, S32 segment,
                            S32 first_trapezoid, S32 last_trapezoid,
                            MergeSide side) {
  /* First merge polys on the LHS */
  S32 t = first_trapezoid;
  while (t &&
         Coord2GreaterThanEqualTo(trapezoids->data[t].min_y,
                                  trapezoids->data[last_trapezoid].min_y)) {
    S32 next_trapezoid;
    bool next_trapezoid_shares_segment;
    if (side == LEFT) {
      next_trapezoid_shares_segment = ((next_trapezoid = trapezoids->data[t].down0) &&
               trapezoids->data[next_trapezoid].right_segment == segment) ||
              ((next_trapezoid = trapezoids->data[t].down1) &&
               trapezoids->data[next_trapezoid].right_segment == segment);
    } else {
      next_trapezoid_shares_segment = ((next_trapezoid = trapezoids->data[t].down0) &&
               trapezoids->data[next_trapezoid].left_segment == segment) ||
              ((next_trapezoid = trapezoids->data[t].down1) &&
               trapezoids->data[next_trapezoid].left_segment == segment);
    }
    if (next_trapezoid_shares_segment &&
        trapezoids->data[t].left_segment ==
            trapezoids->data[next_trapezoid].left_segment &&
        trapezoids->data[t].right_segment ==
            trapezoids->data[next_trapezoid].right_segment) {
      /* good neighbors merge them */
      /* Use the upper node as the new node i.e. t */

      const S32 common_parent =
          query_structure.v[trapezoids->data[next_trapezoid].sink_node].parent;
      if (query_structure.v[common_parent].left ==
          trapezoids->data[next_trapezoid].sink_node) {
        query_structure.v[common_parent].left = trapezoids->data[t].sink_node;
      } else {
        /* redirect parent */
        query_structure.v[common_parent].right = trapezoids->data[t].sink_node;
      }

      /* Change the upper neighbours of the lower trapezoids
       * | add_this      t            |
       * |--|--------------------^----|
       * |  |  next_trapezoid    |    |
       * |--v--------------------|----|
       * |  next_trap.down    add this|
       */
      trapezoids->data[t].down0 = trapezoids->data[next_trapezoid].down0;
      if (trapezoids->data[t].down0) {
        if (trapezoids->data[trapezoids->data[t].down0].up0 == next_trapezoid) {
          trapezoids->data[trapezoids->data[t].down0].up0 = t;
        }
        if (trapezoids->data[trapezoids->data[t].down0].up1 == next_trapezoid) {
          trapezoids->data[trapezoids->data[t].down0].up1 = t;
        }
      }
      trapezoids->data[t].down1 = trapezoids->data[next_trapezoid].down1;
      if (trapezoids->data[t].down1) {
        if (trapezoids->data[trapezoids->data[t].down1].up0 == next_trapezoid) {
          trapezoids->data[trapezoids->data[t].down1].up0 = t;
        }
        if (trapezoids->data[trapezoids->data[t].down1].up1 == next_trapezoid) {
          trapezoids->data[trapezoids->data[t].down1].up1 = t;
        }
      }

      trapezoids->data[t].min_y = trapezoids->data[next_trapezoid].min_y;
      // invalidate the lower trapezium
      trapezoids->data[next_trapezoid].is_valid = false;
    } else {
      // not good neighbors
      t = next_trapezoid;
    }
  }
  }

/* Initialise the query structure (Q) and the trapezoid table (T)
 * when the first segment is added to start the trapezoidation. The
 * query-tree starts out with 4 trapezoids, one S-node and 2 Y-nodes
 *
 *                4
 *   ---------------------------------
 *               \
 *           1    \     2
 *                 \
 *   ---------------------------------
 *                3
 */
static S32 InitQueryStructure(QueryNodeArray *query_structure,
                              Coord2Slice vertices, TrapezoidArray *trapezoids,
                              SegmentArray *segments, S32 initial_segment) {
  Segment *s = &segments->data[initial_segment];

  // first entry is zeroed
  QueryNodeArrayPush(query_structure, (QueryNode){0});
  TrapezoidArrayPush(trapezoids, (Trapezoid){0});
  QueryNode *qs = query_structure->data;

  const S32 root = QueryNodeArrayPush(query_structure, (QueryNode){0});
  const S32 i2 = QueryNodeArrayPush(query_structure, (QueryNode){0});
  const S32 i3 = QueryNodeArrayPush(query_structure, (QueryNode){0});
  const S32 i4 = QueryNodeArrayPush(query_structure, (QueryNode){0});
  const S32 i5 = QueryNodeArrayPush(query_structure, (QueryNode){0});
  const S32 i6 = QueryNodeArrayPush(query_structure, (QueryNode){0});
  const S32 i7 = QueryNodeArrayPush(query_structure, (QueryNode){0});

  const S32 t1 = TrapezoidArrayPush(trapezoids, (Trapezoid){0}); // middle left
  const S32 t2 = TrapezoidArrayPush(trapezoids, (Trapezoid){0}); // middle right
  const S32 t3 = TrapezoidArrayPush(trapezoids, (Trapezoid){0}); // bottom most
  const S32 t4 = TrapezoidArrayPush(trapezoids, (Trapezoid){0}); // top most

  qs[root] =
      (QueryNode){.node_type = Y,
                  .yval = Coord2Max(vertices.v[s->v0], vertices.v[s->v1]),
                  .right = i2,
                  .left = i3};
  qs[i2] = (QueryNode){.node_type = SINK, .parent = root, .trapezoid = t4};
  qs[i3] = (QueryNode){.node_type = Y,
                       .parent = root,
                       .yval = Coord2Min(vertices.v[s->v0], vertices.v[s->v1]),
                       .left = i4,
                       .right = i5};
  qs[i4] = (QueryNode){.node_type = SINK, .parent = i3, .trapezoid = t3};
  qs[i5] = (QueryNode){.node_type = X,
                       .segment = initial_segment,
                       .parent = i3,
                       .left = i6,
                       .right = i7};
  qs[i6] = (QueryNode){.node_type = SINK, .parent = i5, .trapezoid = t1};
  qs[i7] = (QueryNode){.node_type = SINK, .parent = i5, .trapezoid = t2};

  Trapezoid *ts = trapezoids->data;
  ts[t1] = (Trapezoid){.max_y = qs[root].yval,
                       .min_y = qs[i3].yval,
                       .right_segment = initial_segment,
                       .up0 = t4,
                       .down0 = t3,
                       .sink_node = i6,
                       .is_valid = true};
  ts[t2] = (Trapezoid){.max_y = qs[root].yval,
                       .min_y = qs[i3].yval,
                       .left_segment = initial_segment,
                       .up0 = t4,
                       .down0 = t3,
                       .sink_node = i7,
                       .is_valid = true};
  ts[t3] = (Trapezoid){.max_y = qs[i3].yval,
                       .min_y = COORD2_MINUS_INFINITY,
                       .up0 = t1,
                       .up1 = t2,
                       .sink_node = i4,
                       .is_valid = true};
  ts[t4] = (Trapezoid){.max_y = COORD2_PLUS_INFINITY,
                       .min_y = qs[root].yval,
                       .down0 = t1,
                       .down1 = t2,
                       .sink_node = i2,
                       .is_valid = true};

  s->is_inserted = true;
  return root;
}

// generates a random permutation from 1..=n inside the permutation array
// keeps permutation[0] = 0.
static void GeneratePermutation(S32Array *permutation) {
  // srand(2); //TODO: figure out where to seed prng
  const S32 n = permutation->capacity;
  for (S32 i = 0; i < n; i++) {
    S32ArrayPush(permutation, i);
  }
  for (S32 i = n - 1; i > 1; i--) {
    const S32 j = (rand() % i) + 1;
    const S32 tmp = permutation->data[j];
    permutation->data[j] = permutation->data[i];
    permutation->data[i] = tmp;
  }
#ifdef DEBUG
  S32 sum = 0;
  for (S32 i = 1; i < n; i++) {
    sum += permutation->data[i];
  }
  ASSERT(sum == ((n - 1) * n) / 2, "Incorrect permutation")
#endif
}

// selects the next segment from the permutation array
static S32 RandomSegment(S32Array *permutation) {
  if (permutation->len > 1) {
    const S32 segment_index = S32ArrayPop(permutation);
#ifdef DEBUG
    fprintf(stderr, "choose segment: %d, %d left\n", segment_index,
            permutation->len);
#endif
    return segment_index;
  }
  ASSERT(false, "RandomSegment has no more segment to return");
}

// computes log^*(n) (iterative logarithm)
static S32 MathLogStar(S32 n) {
  S32 i;
  F64 v = n;
  for (i = 0; v >= 1; i++) {
    v = log2(v);
  }
  return i - 1;
}

// returns ceil(n / log^h(n))
static S32 MathN(S32 n, S32 h) {
  F64 v = n;
  for (S32 i = 0; i < h; i++) {
    v = log2(v);
  }
  return (S32)ceil((F64)n / v);
}

// Return the maximum (sorting) of the two points (first y value than x value)
static Coord2 Coord2Max(Coord2 v0, Coord2 v1) {
  if (v0.y > v1.y + C_EPS) {
    return v0;
  }
  if (FP_EQUAL(v0.y, v1.y)) {
    if (v0.x > v1.x + C_EPS) {
      return v0;
    }
    return v1;
  }
  return v1;
}

static Coord2 Coord2Min(Coord2 v0, Coord2 v1) {
  if (v0.y < v1.y - C_EPS) {
    return v0;
  }
  if (FP_EQUAL(v0.y, v1.y)) {
    if (v0.x < v1.x - C_EPS) {
      return v0;
    }
    return v1;
  }
  return v1;
}

static bool Coord2GreaterThan(Coord2 v0, Coord2 v1) {
  if (v0.y > v1.y + C_EPS)
    return true;
  if (v0.y < v1.y - C_EPS)
    return false;
  return (v0.x > v1.x);
}

static bool Coord2EqualTo(Coord2 v0, Coord2 v1) {
  return (FP_EQUAL(v0.y, v1.y) && FP_EQUAL(v0.x, v1.x));
}

static bool Coord2GreaterThanEqualTo(Coord2 v0, Coord2 v1) {
  if (v0.y > v1.y + C_EPS)
    return true;
  if (v0.y < v1.y - C_EPS)
    return false;
  return (v0.x >= v1.x);
}

static bool Coord2LessThan(Coord2 v0, Coord2 v1) {
  if (v0.y < v1.y - C_EPS)
    return true;
  if (v0.y > v1.y + C_EPS)
    return false;
  return (v0.x < v1.x);
}

/* Return true if the vertex v is to the left of line segment no.
 * segment. Takes care of the degenerate cases when both the vertices
 * have the same y-coordinate, etc.
 */
static bool VertexLeftOfSegment(const Coord2Slice vertices,
                                const SegmentSlice segments, S32 segment,
                                S32 vertex) {
  const Coord2 segment_v0 = vertices.v[segments.v[segment].v0];
  const Coord2 segment_v1 = vertices.v[segments.v[segment].v1];
  const Coord2 v = vertices.v[vertex];
  if (Coord2GreaterThan(segment_v1, segment_v0)) /* seg. going upwards */
  {
    if (FP_EQUAL(segment_v1.y, v.y)) {
      return v.x < segment_v1.x;
    }
    if (FP_EQUAL(segment_v0.y, v.y)) {
      return v.x < segment_v0.x;
    }
    return CROSS(segment_v0, segment_v1, v) > 0.f;
  } else /* v0 > v1 */
  {
    if (FP_EQUAL(segment_v1.y, v.y)) {
      return v.x < segment_v1.x;
    }
    if (FP_EQUAL(segment_v0.y, v.y)) {
      return v.x < segment_v0.x;
    }
    return CROSS(segment_v1, segment_v0, v) > 0.f;
  }
}

// ------------------ VALIDATION FUNCTIONS ------------------------

static bool NodeIsReachable(const QueryNodeSlice qs, S32 root, S32 node) {
  if (root == node) {
    return true;
  }
  if (!root) {
    return false;
  }
  QueryNode q = qs.v[root];
  return NodeIsReachable(qs, q.left, node) ||
         NodeIsReachable(qs, q.right, node);
}

static void ValidateQueryStructure(const QueryNodeSlice qs) {
  // validate qs[0] is still zeroed.
  ASSERT(qs.v[0].node_type == 0, "QueryStructure[0] is not zeroed")
  ASSERT(qs.v[0].right == 0, "QueryStructure[0] is not zeroed")
  ASSERT(qs.v[0].left == 0, "QueryStructure[0] is not zeroed")
  ASSERT(qs.v[0].segment == 0, "QueryStructure[0] is not zeroed")
  for (S32 i = 1; i < qs.count; i += 1) {
    if (!NodeIsReachable(qs, 1, i)) {
      continue;
    }
    const QueryNode query_node = qs.v[i];
    ASSERT(query_node.node_type != 0, "Invalid node type");
    if (query_node.parent) {
      ASSERT(i == qs.v[query_node.parent].left ||
                 i == qs.v[query_node.parent].right,
             "Node: %d and parents children: %d, %d do not match", i,
             qs.v[query_node.parent].left, qs.v[query_node.parent].right)
    }
    if (query_node.node_type != SINK) {
      ASSERT(query_node.left && query_node.right,
             "X or Y node should have two children.")
    } else {
      ASSERT(!(query_node.left || query_node.right),
             "SINK node should not have children.")
    }
  }
}

static void ValidateTrapezoidStructure(const TrapezoidSlice ts) {
  ASSERT(!ts.v[0].down0, "Trapezoids[0].down0 is not zeroed, was %d instead",
         ts.v[0].down0)
  ASSERT(!ts.v[0].down1, "Trapezoids[0].down1 is not zeroed, was %d instead",
         ts.v[0].down1)
  ASSERT(!ts.v[0].up0, "Trapezoids[0].up0 is not zeroed, was %d instead",
         ts.v[0].up0)
  ASSERT(!ts.v[0].up1, "Trapezoids[0].up1 is not zeroed, was %d instead",
         ts.v[0].up1)
  for (S32 i = 1; i < ts.count; i += 1) {
    const Trapezoid t = ts.v[i];
    if (!t.is_valid) {
      continue;
    }
    ASSERT(Coord2GreaterThanEqualTo(t.max_y, t.min_y), "min_y > max_y for %d",
           i)
    if (!(t.left_segment || t.right_segment) &&
        (Coord2EqualTo(t.min_y, COORD2_MINUS_INFINITY) ||
         Coord2EqualTo(t.max_y, COORD2_PLUS_INFINITY))) {
      ASSERT(!(t.down0 || t.down1) || !(t.up0 || t.up1),
             "Trapezoid with no segment bounds can only have neighbors in one "
             "direction");
    }
    if (t.left_segment || t.right_segment) {
      ASSERT(!Coord2EqualTo(t.min_y, COORD2_MINUS_INFINITY) ||
                 !Coord2EqualTo(t.max_y, COORD2_PLUS_INFINITY),
             "Trapezoid with segment should have min/max_y != INFINITY")
      ASSERT(t.left_segment != t.right_segment, "Trapezoid %d has identical right/left segment: %d", i, t.left_segment)
    }
    ASSERT(t.down0 || !t.down1, "Sole downward neighbor in wrong field");
    ASSERT(t.up0 || !t.up1, "Sole upward neighbor in wrong field");
    if (!t.down0 && !(t.left_segment || t.right_segment)) {
      ASSERT(Coord2EqualTo(t.min_y, COORD2_MINUS_INFINITY),
             "Min_y of %d should be minus infinity if no lower neighbor exists",
             i)
    }
    if (!t.up0 && !(t.left_segment || t.right_segment)) {
      ASSERT(Coord2EqualTo(t.max_y, COORD2_PLUS_INFINITY),
             "Max_y %d should be plus infinity if no upper neighbor exists", i)
    }

    if (t.down0) {
      ASSERT(ts.v[t.down0].up0 == i || ts.v[t.down0].up1 == i,
             "Wrong neighbor info (down0) for %d and %d", i, t.down0);
    }
    if (t.down1) {
      ASSERT(ts.v[t.down1].up0 == i || ts.v[t.down1].up1 == i,
             "Wrong neighbor info (down1) for %d and %d", i, t.down1);
    }
    if (t.up0) {
      ASSERT(ts.v[t.up0].down0 == i || ts.v[t.up0].down1 == i,
             "Wrong neighbor info (up0) for %d and %d", i, t.up0);
    }
    if (t.up1) {
      ASSERT(ts.v[t.up1].down0 == i || ts.v[t.up1].down1 == i,
             "Wrong neighbor info (up1) for %d and %d", i, t.up1);
    }
    if (t.up0) {
      ASSERT(Coord2EqualTo(t.max_y, ts.v[t.up0].min_y),
             "Max of lower and min of upper Trapezoid disagree");
    }
    if (t.up1) {
      ASSERT(Coord2EqualTo(t.max_y, ts.v[t.up1].min_y),
             "Max of lower and min of upper Trapezoid disagree");
    }
    if (t.down0) {
      ASSERT(Coord2EqualTo(t.min_y, ts.v[t.down0].max_y),
             "Max of lower and min of upper Trapezoid disagree");
    }
    if (t.down1) {
      ASSERT(Coord2EqualTo(t.min_y, ts.v[t.down1].max_y),
             "Max of lower and min of upper Trapezoid disagree");
    }
    if (t.up0 && t.up1) {
      ASSERT(!(ts.v[t.up0].down0 && ts.v[t.up0].down1),
             "Trapezoid %d can not have two lower neighbors up0 of %d", t.up0,
             i)
      ASSERT(!(ts.v[t.up1].down0 && ts.v[t.up1].down1),
             "Trapezoid %d can not have two lower neighbors up1 of %d", t.up1,
             i)
    }
    if (t.down0 && t.down1) {
      ASSERT(!(ts.v[t.down0].up0 && ts.v[t.down0].up1),
             "Trapezoid %d can not have two upper neighbors", t.down0)
      ASSERT(!(ts.v[t.down1].up0 && ts.v[t.down1].up1),
             "Trapezoid %d can not have two upper neighbors", t.down1)
    }
  }
}

static void ValidateSegmentStructure(SegmentSlice segments) {
  ASSERT(!segments.v[0].is_inserted, "Segments[0] was wrongly inserted")
  ASSERT(!segments.v[0].next, "Segments[0] not zeroed")
  ASSERT(!segments.v[0].prev, "Segments[0] not zeroed")
  ASSERT(!segments.v[0].v0, "Segments[0] not zeroed")
  ASSERT(!segments.v[0].v1, "Segments[0] not zeroed")
  // TODO: detect possible duplicates? i.e Segment used in multiple cycles
  for (S32 i = 1; i < segments.count; i += 1) {
    Segment s = segments.v[i];
    ASSERT(s.next, "Segment %d should have next neighbor", i)
    ASSERT(s.prev, "Segment %d should have prev neighbor", i)
    ASSERT(s.v0, "Segment %d should have valid v0", i)
    ASSERT(s.v1, "Segment %d should have valid v1", i)
    if (s.is_inserted) {
      ASSERT(s.root0, "Segment %d should have valid root0", i)
      ASSERT(s.root1, "Segment %d should have valid root1", i)
    }
  }
}

static void ValidateQueryTrapezoidSegmentStructures(
    const QueryNodeSlice qs, const TrapezoidSlice ts,
    const SegmentSlice segments, const Coord2Slice vertices) {
  //ValidateQueryStructure(qs);
  ValidateTrapezoidStructure(ts);
  ValidateSegmentStructure(segments);
  for (S32 i = 1; i < qs.count; i += 1) {
    QueryNode node = qs.v[i];
    if (node.node_type == SINK) {
      ASSERT(ts.v[node.trapezoid].sink_node == i,
             "Query structure or trapezoid point to incorrect nodes");
    }
  }
  for (S32 i = 1; i < segments.count; i += 1) {
    const Segment s = segments.v[i];
    if (!s.is_inserted) {
      continue;
    }
    if (qs.v[s.root0].node_type == SINK) {
      const S32 computed_trapezoid_index =
          TrapezoidIndexFromVertex(qs, segments, vertices, s.v0, s.v1, 1);
      ASSERT(computed_trapezoid_index == qs.v[s.root0].trapezoid,
             "Saved Sink: %d and computed Sink: %d mismatch", s.root0,
             computed_trapezoid_index);
    }
    if (qs.v[s.root1].node_type == SINK) {
      const S32 computed_trapezoid_index =
          TrapezoidIndexFromVertex(qs, segments, vertices, s.v1, s.v0, 1);
      ASSERT(computed_trapezoid_index == qs.v[s.root1].trapezoid,
             "Saved Sink: %d and computed Sink: %d mismatch", s.root1,
             computed_trapezoid_index);
    }
  }
  for (S32 i = 1; i < ts.count; i += 1) {
    const Trapezoid t = ts.v[i];
    if (!t.is_valid) {
      continue;
    }

    if (t.up0 && t.up1 && ts.v[t.up0].right_segment &&
        ts.v[t.up1].right_segment) {
      ASSERT(!VertexLeftOfSegment(vertices, segments, ts.v[t.up0].right_segment,
                                  segments.v[ts.v[t.up1].right_segment].v0) ||
                 !VertexLeftOfSegment(vertices, segments,
                                      ts.v[t.up0].right_segment,
                                      segments.v[ts.v[t.up1].right_segment].v1),
             "Up0 is right of up1 for trapezoid %d", i)
    }
    if (t.up0 && t.up1) {
      const Segment up0_right_segment = segments.v[ts.v[t.up0].right_segment];
      const Segment up1_left_segment = segments.v[ts.v[t.up1].left_segment];
      ASSERT(up0_right_segment.v0 == up1_left_segment.v0 ||
                 up0_right_segment.v0 == up1_left_segment.v1 ||
                 up0_right_segment.v1 == up1_left_segment.v0 ||
                 up0_right_segment.v1 == up1_left_segment.v0,
             "Rigtht/Left Segments of %d and %d disagree", t.up0, t.up1)
    }
  }
}

static void ValidateMonotoneChains(const MonotoneChainSlice chains) {
  for (S32 i = 1; i < chains.count; i += 1) {
    const S32 i_prev = chains.v[i].prev;
    const S32 i_next = chains.v[i].next;
    ASSERT(i == chains.v[i_prev].next, "Prev and next of %d and %d disagree",
           i_prev, i)
    ASSERT(i == chains.v[i_next].prev, "Next and prev of %d and %d disagree",
           i_next, i)
  }
}
static void ValidateMonotoneAndVertexChains(const VertexChainSlice vc,
                                            const MonotoneChainSlice mc) {
  ValidateMonotoneChains(mc);
  for (S32 i = 1; i < vc.count; i += 1) {
    for (S32 j = 0; j < vc.v[i].next_free; j += 1) {
      ASSERT(mc.v[vc.v[i].vertex_index_in_monotone_chain[j]].vertex == i,
             "vc -> mc and mc -> vc indices do not match")
    }
  }
}

static void ValidateMonotonePolygon(VertexChainSlice vertex_chains,
                                    MonotoneChainSlice mc, S32 y_max_index,
                                    MonotoneBaseSide side) {

  // RHS segment is a single segment
  Coord2 y_min;
  Coord2 y_max = vertex_chains.v[mc.v[y_max_index].vertex].pt;
  Coord2 y_current = y_max;
  S32 current_vertex, y_min_index;
  if (side == BASE_RIGHT) {
    y_min_index = mc.v[y_max_index].prev;
    y_min = vertex_chains.v[mc.v[y_min_index].vertex].pt;
    current_vertex = mc.v[y_max_index].next;
  } else {
    y_min_index = mc.v[y_max_index].next;
    y_min = vertex_chains.v[mc.v[y_min_index].vertex].pt;
    current_vertex = mc.v[y_max_index].prev;
  }
  fprintf(stderr, "Polygon base: %d -> %d\n", mc.v[y_max_index].vertex,
          mc.v[y_min_index].vertex);

  while (current_vertex != y_min_index) {
    if (!Coord2LessThan(vertex_chains.v[mc.v[current_vertex].vertex].pt,
                        y_current)) {
      fprintf(stderr, "Polygon y_max_index: %d is not a monotone mountain\n",
              y_max_index);
    }
    //      ASSERT(Coord2LessThan(vertex_chains.v[mc.v[current_vertex].vertex].pt,
    //                            y_current),
    //             "Polygon is not a monotone mountain");
    // ASSERT(Coord2GreaterThanEqualTo(
    //           vertex_chains.v[mc.v[current_vertex].vertex].pt, y_min),
    //       "Polygon vertex %d smaller than y_min",
    //       mc.v[current_vertex].vertex);
    y_current = vertex_chains.v[mc.v[current_vertex].vertex].pt;
    if (side == BASE_RIGHT) {
      current_vertex = mc.v[current_vertex].next;
    } else {
      current_vertex = mc.v[current_vertex].prev;
    }
  }
}
