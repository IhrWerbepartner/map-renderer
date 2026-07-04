#pragma once
#include "arena.c"
#include "base.h"
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <time.h>

// TODO: implement tessalation

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
  LEFT,
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

#define TRIANG_INFINITY (1 << 30)
#define C_EPS 1.11e-16
#define FP_EQUAL(s, t) (fabs(s - t) <= C_EPS)
#define CROSS(v0, v1, v2)                                                      \
  (((v1).x - (v0).x) * ((v2).y - (v0).y) -                                     \
   ((v1).y - (v0).y) * ((v2).x - (v0).x))

#define DOT(v0, v1) ((v0).x * (v1).x + (v0).y * (v1).y)

static void GeneratePermutation(S32Array *permutation, S32 n);
static S32 RandomSegment(S32Array *permutation);
static S32 InitQueryStructure(QueryNodeArray *query_structure,
                              Coord2Slice vertices, TrapezoidArray *trapezoids,
                              SegmentArray *segments, S32 initial_segment);
static void ConstructTrapezoidation(QueryNodeArray *query_structure,
                                    Coord2Slice veritces,
                                    SegmentArray *segments,
                                    TrapezoidArray *trapezoids,
                                    S32Array *permutation);
static void AddSegment(QueryNodeArray *query_structure, Coord2Slice veritces,
                       SegmentSlice segments, TrapezoidArray *trapezoids,
                       S32 segment);
static void MergeTrapezoids(QueryNodeSlice query_structure,
                            Coord2Slice veritces, SegmentArray *segments,
                            TrapezoidArray *trapezoids, S32 segment,
                            S32 first_trapezoid, S32 last_trapezoid,
                            MergeSide side);
static S32 TrapezoidIndexFromVertex(QueryNodeSlice query_structure,
                                    SegmentSlice segments, Coord2Slice vertices,
                                    S32 vertex, S32 opposite_endpoint,
                                    S32 root);

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
/*
typedef struct triangulation_state {
  QueryNode *query_structure;
  QueryIndex query_size;
  Trapezoid *trapezoids;
  S32 trapezoid_size;
  Segment *segments;
  MonotoneChain *monotone_chain;
  S32 monotone_chain_size;
  VertexChain *vertex_chain;
  S32 vertex_chain_size;
  MonotoneChainPos *monotone_chain_position;
  bool *visited;
} TriangState;
*/

// computes the triangulation of the coordinates which are partitioned by
// countour_sizes. Coords[0] is NOT USED. so coords[1..=cs[0]] polygons are
// the first ones, coords[cs[0]..cs[1]] the next etc. returns the triangles
// as indices into the coords array.
TriangleArray tessalate_polygon(Arena *arena, Coord2Array coords,
                                S32Slice contour_sizes) {
  // filled and returned
  TriangleArray triangles = TriangleArrayNew(arena, coords.len);

  // scratch datastructure
  Arena *scratch = GetScratchConflict(&arena, 1).arena;
  QueryNodeArray query_structure = QueryNodeArrayNew(scratch, 8 * coords.len);
  SegmentArray segments = SegmentArrayNew(scratch, coords.len);
  TrapezoidArray trapezoids = TrapezoidArrayNew(scratch, 4 * coords.len);
  S32Array permutated_segments = S32ArrayNew(
      scratch, coords.len + 1); // TODO: figure out if this is necessary

  S32 contour = 0;
  S32 i = 1;
  while (contour < contour_sizes.count) {
    S32 contour_point_count = contour_sizes.v[contour];
    S32 first = i;
    S32 last = first + contour_point_count - 1;

    for (S32 j = 0; j < contour_point_count; j++, i++) {
      segments.data[i].v0 = i;
      segments.data[i].is_inserted = false;
      if (i == last) {
        segments.data[i].next = first;
        segments.data[i].prev = i - 1;
        segments.data[i - 1].v1 = segments.data[i].v0;
      } else if (i == first) {
        segments.data[i].next = i + 1;
        segments.data[i].prev = last;
        segments.data[last].v1 = segments.data[i].v0;
      } else {
        segments.data[i].prev = i - 1;
        segments.data[i].next = i + 1;
        segments.data[i - 1].v1 = segments.data[i].v0;
      }
    }
    contour += 1;
  }

  GeneratePermutation(&permutated_segments, i - 1);
  ConstructTrapezoidation(&query_structure, Coord2SliceFromArray(&coords),
                          &segments, &trapezoids, &permutated_segments);
  return triangles;
}

static void ConstructTrapezoidation(QueryNodeArray *query_structure,
                                    Coord2Slice vertices,
                                    SegmentArray *segments,
                                    TrapezoidArray *trapezoids,
                                    S32Array *permutation) {
  // Add the first segment and get the query structure and trapezoid list
  // initialised
  S32 query_root = InitQueryStructure(query_structure, vertices, trapezoids,
                                      segments, RandomSegment(permutation));
  for (S32 i = 1; i <= segments->len; i++) {
    segments->data[i].root0 = segments->data[i].root1 = query_root;
  }

  for (S32 h = 1; h <= MathLogStar(segments->len); h++) {
    for (S32 i = MathN(segments->len, h - 1) + 1; i <= MathN(segments->len, h);
         i++) {
      AddSegment(query_structure, vertices, SegmentSliceFromArray(segments),
                 trapezoids, RandomSegment(permutation));
    }

    /* Find a new root for each of the segment endpoints */
    for (S32 i = 1; i <= segments->len; i++) {
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

  for (S32 i = MathN(segments->len, MathLogStar(segments->len)) + 1;
       i <= segments->len; i++) {
    AddSegment(query_structure, vertices, SegmentSliceFromArray(segments),
               trapezoids, RandomSegment(permutation));
  }
}

// locates vertex in the query structure, starting the search at root.
// returns the trapezoid index the node is currently located in.
static S32 TrapezoidIndexFromVertex(QueryNodeSlice query_structure,
                                    SegmentSlice segments, Coord2Slice vertices,
                                    S32 vertex, S32 segment_endpoint,
                                    S32 root) {
  QueryNode root_node = query_structure.v[root];

  // TODO: make this iterative, no need for recursion
  switch (root_node.node_type) {
  case SINK:
    return root_node.trapezoid;

  case Y:
    if (Coord2GreaterThan(vertices.v[vertex], root_node.yval)) {
      // above
      return TrapezoidIndexFromVertex(query_structure, segments, vertices,
                                      vertex, segment_endpoint,
                                      root_node.right);
    }
    if (Coord2EqualTo(vertices.v[vertex], root_node.yval)) {
      // the point is already inserted
      if (Coord2GreaterThan(vertices.v[segment_endpoint], root_node.yval)) {
        // above
        return TrapezoidIndexFromVertex(query_structure, segments, vertices,
                                        vertex, segment_endpoint,
                                        root_node.right);
      } else {
        // below
        return TrapezoidIndexFromVertex(query_structure, segments, vertices,
                                        vertex, segment_endpoint,
                                        root_node.left);
      }
    } else {
      // below
      return TrapezoidIndexFromVertex(query_structure, segments, vertices,
                                      vertex, segment_endpoint, root_node.left);
    }

  case X:
    if (Coord2EqualTo(vertices.v[vertex],
                      vertices.v[segments.v[root_node.segment].v0]) ||
        Coord2EqualTo(vertices.v[vertex],
                      vertices.v[segments.v[root_node.segment].v1])) {
      if (FP_EQUAL(vertices.v[vertex].y,
                   vertices.v[segment_endpoint].y)) /* horizontal segment */ {
        if (vertices.v[segment_endpoint].x < vertices.v[vertex].x) {
          return TrapezoidIndexFromVertex(query_structure, segments, vertices,
                                          vertex, segment_endpoint,
                                          root_node.left); /* left */
        } else {
          return TrapezoidIndexFromVertex(query_structure, segments, vertices,
                                          vertex, segment_endpoint,
                                          root_node.right); /* right */
        }
      }

      if (VertexLeftOfSegment(vertices, segments, root_node.segment,
                              segment_endpoint)) {
        return TrapezoidIndexFromVertex(query_structure, segments, vertices,
                                        vertex, segment_endpoint,
                                        root_node.left); /* left */
      } else {
        return TrapezoidIndexFromVertex(query_structure, segments, vertices,
                                        vertex, segment_endpoint,
                                        root_node.right); /* right */
      }
    }
    if (VertexLeftOfSegment(vertices, segments, root_node.segment, vertex)) {
      return TrapezoidIndexFromVertex(query_structure, segments, vertices,
                                      vertex, segment_endpoint,
                                      root_node.left); /* left */
    } else {
      return TrapezoidIndexFromVertex(query_structure, segments, vertices,
                                      vertex, segment_endpoint,
                                      root_node.right); /* right */
    }

  default:
    ERROR_MSG("Node has no type and can not be part of location query!\n")
  }
}

// adds a vertex to the query structure and updates the trapezoidation.
// returns the first index of the affected trapezoid
// returns the lower trapezoid if this is the first vertex inserted
// the upper trapezoid if it was the second (see first_vertex)
static S32 AddVertex(QueryNodeArray *query_structure, Coord2Slice veritces,
                     SegmentSlice segments, TrapezoidArray *trapezoids,
                     S32 segment, S32 v0, S32 v1, S32 v0_root,
                     bool first_vertex) {

  S32 tu = TrapezoidIndexFromVertex(QueryNodeSliceFromArray(query_structure),
                                    segments, veritces, v0, v1, v0_root);
  S32 tl = TrapezoidArrayPush(
      trapezoids, trapezoids->data[tu]); /* tl is the new lower trapezoid */
  trapezoids->data[tl].is_valid = true;
  trapezoids->data[tu].min_y = trapezoids->data[tl].max_y = veritces.v[v0];
  trapezoids->data[tu].down0 = tl;
  trapezoids->data[tu].down1 = 0;
  trapezoids->data[tl].up0 = tu;
  trapezoids->data[tl].up1 = 0;

  // update neigbouring information of nearby trapezoids (if existing)
  S32 neigbour = trapezoids->data[tl].down0;
  if (neigbour) {
    if (trapezoids->data[neigbour].up0 == tu) {
      trapezoids->data[neigbour].up0 = tl;
    }
    if (trapezoids->data[neigbour].up1 == tu) {
      trapezoids->data[neigbour].up1 = tl;
    }
  }

  neigbour = trapezoids->data[tl].down1;
  if (neigbour) {
    if (trapezoids->data[neigbour].up0 == tu) {
      trapezoids->data[neigbour].up0 = tl;
    }
    if (trapezoids->data[neigbour].up1 == tu) {
      trapezoids->data[neigbour].up1 = tl;
    }
  }

  // Now update the query structure and obtain the sinks for the  two trapezoids
  // Upper trapezoid sink
  S32 i1 = QueryNodeArrayPush(query_structure, (QueryNode){0});
  // Lower trapezoid sink
  S32 i2 = QueryNodeArrayPush(query_structure, (QueryNode){0});
  S32 sk = trapezoids->data[tu].sink_node;

  // update the existing node (old sink)
  query_structure->data[sk].node_type = Y;
  query_structure->data[sk].yval = veritces.v[v0];
  query_structure->data[sk].segment = segment;
  query_structure->data[sk].left = i2;
  query_structure->data[sk].right = i1;

  query_structure->data[i1] =
      (QueryNode){.node_type = SINK, .trapezoid = tu, .parent = sk};
  query_structure->data[i2] =
      (QueryNode){.node_type = SINK, .trapezoid = tl, .parent = sk};

  trapezoids->data[tu].sink_node = i1;
  trapezoids->data[tl].sink_node = i2;
  return first_vertex ? tl : tu;
}

// adds a segment to the query structure and updates the trapezoidation.
static void AddSegment(QueryNodeArray *query_structure, Coord2Slice veritces,
                       SegmentSlice segments, TrapezoidArray *trapezoids,
                       S32 segment) {
  bool tribot = false;
  bool is_swapped = false;
  Segment s = segments.v[segment];
  bool v0_inserted = segments.v[segments.v[segment].prev].is_inserted;
  bool v1_inserted = segments.v[segments.v[segment].next].is_inserted;

  // swap v0, v1 such that v0 is higher than v1
  if (Coord2GreaterThan(veritces.v[s.v1], veritces.v[s.v0])) {
    S32 tmp = s.v0;
    s.v0 = s.v1;
    s.v1 = tmp;
    tmp = s.root0;
    s.root0 = s.root1;
    s.root1 = tmp;
    is_swapped = true;
  }

  // defines the top and bottom most trapezoids affected by threading this
  // segment through the trapezoidation structure
  S32 top_trapezoid, bottom_trapezoid;
  // insert v0 if not already inserted
  if (!v0_inserted) {
    top_trapezoid = AddVertex(query_structure, veritces, segments, trapezoids,
                              segment, s.v0, s.v1, s.root0, true);
  } else {
    // Get the topmost intersecting trapezoid
    top_trapezoid =
        TrapezoidIndexFromVertex(QueryNodeSliceFromArray(query_structure),
                                 segments, veritces, s.v0, s.v1, s.root0);
  }
  //
  // insert v1 if not already inserted
  if (!v1_inserted) {
    bottom_trapezoid =
        AddVertex(query_structure, veritces, segments, trapezoids, segment,
                  s.v1, s.v0, s.root1, false);
  } else {
    // Get the topmost intersecting trapezoid
    bottom_trapezoid =
        TrapezoidIndexFromVertex(QueryNodeSliceFromArray(query_structure),
                                 segments, veritces, s.v1, s.v0, s.root1);
    tribot = true;
  }

  /* Thread the segment into the query tree creating a new X-node */
  /* First, split all the trapezoids which are intersected by s into */
  /* two */

  S32 t = top_trapezoid; /* topmost trapezoid */
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

    S32 tfirstr, tlastr;
    if (t == top_trapezoid)
      tfirstr = new_trapezoid;
    if (Coord2EqualTo(trapezoids->data[t].max_y,
                      trapezoids->data[bottom_trapezoid].min_y))
      tlastr = new_trapezoid;

    trapezoids->data[new_trapezoid] = trapezoids->data[t];
    trapezoids->data[t].sink_node = left_sink_node;
    trapezoids->data[new_trapezoid].sink_node = right_sink_node;
    S32 t_sav = t;
    S32 tn_sav = new_trapezoid;

    // error case: no trapezoid below, cannot arise
    if (!(trapezoids->data[t].down0 || trapezoids->data[t].down1)) {
      ERROR_MSG("AddSegment: error, no lower trapezoid exists\n");
    }

    // only one trapezoid below. partition t into two and make the two resulting
    // trapezoids t and tn as the upper neighbours of the sole lower trapezoid
    if (trapezoids->data[t].down0 && (!trapezoids->data[t].down1)) {
      // continuation of a chain from above
      if (trapezoids->data[t].up0 && trapezoids->data[t].up1) {
        // three upper neighbours
        if (trapezoids->data[t].usave) {
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
        S32 tmp_u = trapezoids->data[t].up0;
        S32 td0 = trapezoids->data[tmp_u].down0;
        S32 td1 = trapezoids->data[tmp_u].down1;
        if (td0 && td1) {
          //      upward cusp
          //          t.u0
          // ---v----------v^----
          //         / \  new
          //        / t \
          // --------------------
          if (trapezoids->data[td0].right_segment &&
              !VertexLeftOfSegment(veritces, segments,
                                   trapezoids->data[td0].right_segment,
                                   segments.v[segment].v1)) {
            trapezoids->data[t].up0 = trapezoids->data[t].up1 =
                trapezoids->data[new_trapezoid].up1 = 0;
            trapezoids->data[trapezoids->data[new_trapezoid].up0].down1 =
                new_trapezoid;
          } else {
            // TODO: idk if this is right
            //    cusp going leftwards
            //        t.u0
            // ---------------v----
            //      /   \
            //     / new \ t
            // --------------------
            trapezoids->data[new_trapezoid].up0 =
                trapezoids->data[new_trapezoid].up1 = trapezoids->data[t].up1 =
                    0;
            trapezoids->data[trapezoids->data[t].up0].down0 = t;
          }
        } else {
          //  fresh segment
          //       t.u0
          //     0        1
          //-----|--------| -----
          //     v   \    v
          //     t    \  new
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
            VertexLeftOfSegment(veritces, segments, tmptriseg, s.v0)) {
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
        S32 down0_of_t = trapezoids->data[t].down0;
        if (trapezoids->data[down0_of_t].up0 &&
            trapezoids->data[down0_of_t].up1) {
          // passes thru LHS
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
    }

    if ((trapezoids[t].d0 <= 0) &&
        (trapezoids[t].d1 > 0)) { /* Only one trapezoid below */
      if ((trapezoids[t].u0 > 0) &&
          (trapezoids[t].u1 > 0)) {  /* continuation of a chain from abv. */
        if (trapezoids[t].usave > 0) /* three upper neighbours */
        {
          if (trapezoids[t].uside == S_LEFT) {
            trapezoids[tn].u0 = trapezoids[t].u1;
            trapezoids[t].u1 = -1;
            trapezoids[tn].u1 = trapezoids[t].usave;

            trapezoids[trapezoids[t].u0].d0 = t;
            trapezoids[trapezoids[tn].u0].d0 = tn;
            trapezoids[trapezoids[tn].u1].d0 = tn;
          } else /* intersects in the right */
          {
            trapezoids[tn].u1 = -1;
            trapezoids[tn].u0 = trapezoids[t].u1;
            trapezoids[t].u1 = trapezoids[t].u0;
            trapezoids[t].u0 = trapezoids[t].usave;

            trapezoids[trapezoids[t].u0].d0 = t;
            trapezoids[trapezoids[t].u1].d0 = t;
            trapezoids[trapezoids[tn].u0].d0 = tn;
          }

          trapezoids[t].usave = trapezoids[tn].usave = 0;
        } else /* No usave.... simple case */
        {
          trapezoids[tn].u0 = trapezoids[t].u1;
          trapezoids[t].u1 = trapezoids[tn].u1 = -1;
          trapezoids[trapezoids[tn].u0].d0 = tn;
        }
      } else { /* fresh seg. or upward cusp */
        int tmp_u = trapezoids[t].u0;
        int td0, td1;
        if (((td0 = trapezoids[tmp_u].d0) > 0) &&
            ((td1 = trapezoids[tmp_u].d1) > 0)) { /* upward cusp */
          if ((trapezoids[td0].right_segment > 0) &&
              !is_left_of(trapezoids[td0].right_segment, &s.v1)) {
            trapezoids[t].u0 = trapezoids[t].u1 = trapezoids[tn].u1 = -1;
            trapezoids[trapezoids[tn].u0].d1 = tn;
          } else {
            trapezoids[tn].u0 = trapezoids[tn].u1 = trapezoids[t].u1 = -1;
            trapezoids[trapezoids[t].u0].d0 = t;
          }
        } else /* fresh segment */
        {
          trapezoids[trapezoids[t].u0].d0 = t;
          trapezoids[trapezoids[t].u0].d1 = tn;
        }
      }

      if (FP_EQUAL(trapezoids[t].lo.y, trapezoids[bottom_trapezoid].lo.y) &&
          FP_EQUAL(trapezoids[t].lo.x, trapezoids[bottom_trapezoid].lo.x) &&
          tribot) { /* bottom forms a triangle */
        int tmpseg = 0;

        if (is_swapped)
          tmptriseg = segments[segnum].prev;
        else
          tmptriseg = segments[segnum].next;

        if ((tmpseg > 0) && is_left_of(tmpseg, &s.v0)) {
          /* L-R downward cusp */
          trapezoids[trapezoids[t].d1].u0 = t;
          trapezoids[tn].d0 = trapezoids[tn].d1 = -1;
        } else {
          /* R-L downward cusp */
          trapezoids[trapezoids[tn].d1].u1 = tn;
          trapezoids[t].d0 = trapezoids[t].d1 = -1;
        }
      } else {
        if ((trapezoids[trapezoids[t].d1].u0 > 0) &&
            (trapezoids[trapezoids[t].d1].u1 > 0)) {
          if (trapezoids[trapezoids[t].d1].u0 == t) /* passes thru LHS */
          {
            trapezoids[trapezoids[t].d1].usave =
                trapezoids[trapezoids[t].d1].u1;
            trapezoids[trapezoids[t].d1].uside = S_LEFT;
          } else {
            trapezoids[trapezoids[t].d1].usave =
                trapezoids[trapezoids[t].d1].u0;
            trapezoids[trapezoids[t].d1].uside = S_RIGHT;
          }
        }
        trapezoids[trapezoids[t].d1].u0 = t;
        trapezoids[trapezoids[t].d1].u1 = tn;
      }

      t = trapezoids[t].d1;
    }

    /* two trapezoids below. Find out which one is intersected by */
    /* this segment and proceed down that one */

    else {
      F64 y0, yt;
      Coord2 tmppt;
      int tnext, i_d0, i_d1;

      i_d0 = i_d1 = FALSE;
      if (FP_EQUAL(trapezoids[t].lo.y, s.v0.y)) {
        if (trapezoids[t].lo.x > s.v0.x)
          i_d0 = TRUE;
        else
          i_d1 = TRUE;
      } else {
        tmppt.y = y0 = trapezoids[t].lo.y;
        yt = (y0 - s.v0.y) / (s.v1.y - s.v0.y);
        tmppt.x = s.v0.x + yt * (s.v1.x - s.v0.x);

        if (_less_than(&tmppt, &trapezoids[t].lo))
          i_d0 = TRUE;
        else
          i_d1 = TRUE;
      }

      /* check continuity from the top so that the lower-neighbour */
      /* values are properly filled for the upper trapezoid */

      if ((trapezoids[t].u0 > 0) &&
          (trapezoids[t].u1 > 0)) {  /* continuation of a chain from abv. */
        if (trapezoids[t].usave > 0) /* three upper neighbours */
        {
          if (trapezoids[t].uside == S_LEFT) {
            trapezoids[tn].u0 = trapezoids[t].u1;
            trapezoids[t].u1 = -1;
            trapezoids[tn].u1 = trapezoids[t].usave;

            trapezoids[trapezoids[t].u0].d0 = t;
            trapezoids[trapezoids[tn].u0].d0 = tn;
            trapezoids[trapezoids[tn].u1].d0 = tn;
          } else /* intersects in the right */
          {
            trapezoids[tn].u1 = -1;
            trapezoids[tn].u0 = trapezoids[t].u1;
            trapezoids[t].u1 = trapezoids[t].u0;
            trapezoids[t].u0 = trapezoids[t].usave;

            trapezoids[trapezoids[t].u0].d0 = t;
            trapezoids[trapezoids[t].u1].d0 = t;
            trapezoids[trapezoids[tn].u0].d0 = tn;
          }

          trapezoids[t].usave = trapezoids[tn].usave = 0;
        } else /* No usave.... simple case */
        {
          trapezoids[tn].u0 = trapezoids[t].u1;
          trapezoids[tn].u1 = -1;
          trapezoids[t].u1 = -1;
          trapezoids[trapezoids[tn].u0].d0 = tn;
        }
      } else { /* fresh seg. or upward cusp */
        int tmp_u = trapezoids[t].u0;
        int td0, td1;
        if (((td0 = trapezoids[tmp_u].d0) > 0) &&
            ((td1 = trapezoids[tmp_u].d1) > 0)) { /* upward cusp */
          if ((trapezoids[td0].right_segment > 0) &&
              !is_left_of(trapezoids[td0].right_segment, &s.v1)) {
            trapezoids[t].u0 = trapezoids[t].u1 = trapezoids[tn].u1 = -1;
            trapezoids[trapezoids[tn].u0].d1 = tn;
          } else {
            trapezoids[tn].u0 = trapezoids[tn].u1 = trapezoids[t].u1 = -1;
            trapezoids[trapezoids[t].u0].d0 = t;
          }
        } else /* fresh segment */
        {
          trapezoids[trapezoids[t].u0].d0 = t;
          trapezoids[trapezoids[t].u0].d1 = tn;
        }
      }

      if (FP_EQUAL(trapezoids[t].lo.y, trapezoids[bottom_trapezoid].lo.y) &&
          FP_EQUAL(trapezoids[t].lo.x, trapezoids[bottom_trapezoid].lo.x) &&
          tribot) {
        /* this case arises only at the lowest trapezoid.. i.e.
           tlast, if the lower endpoint of the segment is
           already inserted in the structure */

        trapezoids[trapezoids[t].d0].u0 = t;
        trapezoids[trapezoids[t].d0].u1 = -1;
        trapezoids[trapezoids[t].d1].u0 = tn;
        trapezoids[trapezoids[t].d1].u1 = -1;

        trapezoids[tn].d0 = trapezoids[t].d1;
        trapezoids[t].d1 = trapezoids[tn].d1 = -1;

        tnext = trapezoids[t].d1;
      } else if (i_d0)
      /* intersecting d0 */
      {
        trapezoids[trapezoids[t].d0].u0 = t;
        trapezoids[trapezoids[t].d0].u1 = tn;
        trapezoids[trapezoids[t].d1].u0 = tn;
        trapezoids[trapezoids[t].d1].u1 = -1;

        /* new code to determine the bottom neighbours of the */
        /* newly partitioned trapezoid */

        trapezoids[t].d1 = -1;

        tnext = trapezoids[t].d0;
      } else /* intersecting d1 */
      {
        trapezoids[trapezoids[t].d0].u0 = t;
        trapezoids[trapezoids[t].d0].u1 = -1;
        trapezoids[trapezoids[t].d1].u0 = t;
        trapezoids[trapezoids[t].d1].u1 = tn;

        /* new code to determine the bottom neighbours of the */
        /* newly partitioned trapezoid */

        trapezoids[tn].d0 = trapezoids[t].d1;
        trapezoids[tn].d1 = -1;

        tnext = trapezoids[t].d1;
      }

      t = tnext;
    }

    trapezoids[t_sav].right_segment = trapezoids[tn_sav].left_segment = segnum;
  } /* end-while */

  /* Now combine those trapezoids which share common segments. We can */
  /* use the pointers to the parent to connect these together. This */
  /* works only because all these new trapezoids have been formed */
  /* due to splitting by the segment, and hence have only one parent */

  tfirstl = tfirst;
  tlastl = tlast;
  merge_trapezoids(segnum, tfirstl, tlastl, S_LEFT);
  merge_trapezoids(segnum, tfirstr, tlastr, S_RIGHT);

  segments[segnum].is_inserted = TRUE;
  return 0;
}

/* Initilialise the query structure (Q) and the trapezoid table (T)
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
  SegmentArrayPush(segments, (Segment){0});
  TrapezoidArrayPush(trapezoids, (Trapezoid){0});
  QueryNode *qs = query_structure->data;

  S32 root = QueryNodeArrayPush(query_structure, (QueryNode){0});
  S32 i2 = QueryNodeArrayPush(query_structure, (QueryNode){0});
  S32 i3 = QueryNodeArrayPush(query_structure, (QueryNode){0});
  S32 i4 = QueryNodeArrayPush(query_structure, (QueryNode){0});
  S32 i5 = QueryNodeArrayPush(query_structure, (QueryNode){0});
  S32 i6 = QueryNodeArrayPush(query_structure, (QueryNode){0});
  S32 i7 = QueryNodeArrayPush(query_structure, (QueryNode){0});

  S32 t1 = TrapezoidArrayPush(trapezoids, (Trapezoid){0}); // middle left
  S32 t2 = TrapezoidArrayPush(trapezoids, (Trapezoid){0}); // middle right
  S32 t3 = TrapezoidArrayPush(trapezoids, (Trapezoid){0}); // bottom most
  S32 t4 = TrapezoidArrayPush(trapezoids, (Trapezoid){0}); // top most

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
                       .min_y = (Coord2){(F64)-1 * (TRIANG_INFINITY),
                                         -1 * (F64)(TRIANG_INFINITY)},
                       .up0 = t1,
                       .up1 = t2,
                       .sink_node = i4,
                       .is_valid = true};
  ts[t4] = (Trapezoid){
      .max_y = (Coord2){(F64)(TRIANG_INFINITY), (F64)(TRIANG_INFINITY)},
      .min_y = qs[root].yval,
      .down0 = t1,
      .down1 = t2,
      .sink_node = i2,
      .is_valid = true};

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

  s->is_inserted = true;
  return root;
}

// generates a random permutaiton from 1..=n inside the permutaiton array
// keeps permuation[0] = 0.
static void GeneratePermutation(S32Array *permutation, S32 n) {
  // srand(time(NULL)); TODO: figure out where to seed prng
  for (S32 i = 0; i <= n; i++) {
    permutation->data[i] = i;
  }
  for (S32 i = n; i > 1; i--) {
    S32 j = (rand() % (i + 1)) + 1;
    S32 tmp = permutation->data[j];
    permutation->data[j] = permutation->data[i];
    permutation->data[i] = tmp;
  }
};

// selects the next segment from the permutaiton array
static S32 RandomSegment(S32Array *permutation) {
  S32 segment_index = S32ArrayPop(permutation);
#ifdef DEBUG
  fprintf(stderr, "choose_segment: %d\n", segment_index);
#endif
  return segment_index;
}

// computes log^*(n) (iterative logaritm)
static S32 MathLogStar(S32 n) {
  S32 i;
  F64 v = (F64)n;
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
  } else if (FP_EQUAL(v0.y, v1.y)) {
    if (v0.x > v1.x + C_EPS) {
      return v0;
    } else {
      return v1;
    }
  } else {
    return v1;
  }
}

static bool Coord2GreaterThan(Coord2 v0, Coord2 v1) {
  if (v0.y > v1.y + C_EPS)
    return true;
  else if (v0.y < v1.y - C_EPS)
    return false;
  else
    return (v0.x > v1.x);
}

static bool Coord2EqualTo(Coord2 v0, Coord2 v1) {
  return (FP_EQUAL(v0.y, v1.y) && FP_EQUAL(v0.x, v1.x));
}

static bool Coord2GreaterThanEqualTo(Coord2 v0, Coord2 v1) {
  if (v0.y > v1.y + C_EPS)
    return true;
  else if (v0.y < v1.y - C_EPS)
    return false;
  else
    return (v0.x >= v1.x);
}

static bool Coord2LessThan(Coord2 v0, Coord2 v1) {
  if (v0.y < v1.y - C_EPS)
    return true;
  else if (v0.y > v1.y + C_EPS)
    return false;
  else
    return (v0.x < v1.x);
}

/* Retun TRUE if the vertex v is to the left of line segment no.
 * segnum. Takes care of the degenerate cases when both the vertices
 * have the same y--cood, etc.
 */
static bool VertexLeftOfSegment(Coord2Slice vertices, SegmentSlice segments,
                                S32 segment, S32 vertex) {
  F64 area;
  Coord2 segment_v0 = veritces.v[segments.v[segment].v0];
  Coord2 segment_v1 = veritces.v[segments.v[segment].v1];
  Coord2 v = veritces.v[vertex];
  if (Coord2GreaterThan(segment_v1, segment_v0)) /* seg. going upwards */
  {
    if (FP_EQUAL(segment_v1.y, v.y)) {
      if (v.x < segment_v1.x)
        area = 1.0;
      else
        area = -1.0;
    } else if (FP_EQUAL(segment_v0.y, v.y)) {
      if (v.x < segment_v0.x)
        area = 1.0;
      else
        area = -1.0;
    } else
      area = CROSS(segment_v0, segment_v1, v);
  } else /* v0 > v1 */
  {
    if (FP_EQUAL(segment_v1.y, v.y)) {
      if (v.x < segment_v1.x)
        area = 1.0;
      else
        area = -1.0;
    } else if (FP_EQUAL(segment_v0.y, v.y)) {
      if (v.x < segment_v0.x)
        area = 1.0;
      else
        area = -1.0;
    } else
      area = CROSS(segment_v1, segment_v0, v);
  }
  return (area > 0.0);
}
