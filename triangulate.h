#pragma once

#include "base.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>

/* Segment attributes */
typedef struct {
  Coord2 v0, v1;    /* two endpoints */
  int is_inserted;  /* inserted in trapezoidation yet ? */
  int root0, root1; /* root nodes in Q */
  int next;         /* Next logical segment */
  int prev;         /* Previous segment */
} Segment;

/* Trapezoid attributes */

typedef struct {
  int left_segment, right_segment; /* two adjoining segments */
  Coord2 hi, lo;  /* max/min y-values */
  int u0, u1;
  int d0, d1;
  int sink;         /* pointer to corresponding in Q */
  int usave, uside; /* I forgot what this means */
  int state;
} Trapezoid;

/* Node attributes for every node in the query structure */

/* Node types */

typedef enum NodeType {
  X = 1,
  Y = 2,
  SINK = 3,
} NodeType;

typedef struct {
  NodeType nodetype; /* Y-node or S-node */
  int segnum;
  Coord2 yval;
  int trnum;
  int parent;      /* doubly linked DAG */
  int left, right; /* children */
} QueryNode;

typedef struct {
  int vnum;
  int next;   /* Circularly linked list  */
  int prev;   /* describing the monotone */
  int marked; /* polygon */
} MonotoneChain;

typedef struct {
  Coord2 pt;
  int vnext[4]; /* next vertices for the 4 chains */
  int vpos[4];  /* position of v in the 4 chains */
  int nextfree;
} VertexChain;

#define MAX_SEGMENTS 22050 /* max# of segments. Determines how */
                          /* many points can be specified as */
                          /* input. If your datasets have large */
                          /* number of points, increase this */
                          /* value accordingly. */

#define QSIZE 8 * MAX_SEGMENTS  /* maximum table sizes */
#define TRSIZE 4 * MAX_SEGMENTS /* max# trapezoids */

#define TRUE 1
#define FALSE 0

#define FIRSTPT 1 /* checking whether pt. is inserted */
#define LASTPT 2

#define TRIANG_INFINITY 1 << 30
#define C_EPS 1.11e-16 /* tolerance value: Used for making */
                      /* all decisions about collinearity or */
                      /* left/right of segment. Decrease */
                      /* this value if the input points are */
                      /* spaced very close together */

#define S_LEFT 1 /* for merge-direction */
#define S_RIGHT 2

#define ST_VALID 1 /* for trapezium state */
#define ST_INVALID 2

#define SP_SIMPLE_LRUP 1 /* for splitting trapezoids */
#define SP_SIMPLE_LRDN 2
#define SP_2UP_2DN 3
#define SP_2UP_LEFT 4
#define SP_2UP_RIGHT 5
#define SP_2DN_LEFT 6
#define SP_2DN_RIGHT 7
#define SP_NOSPLIT -1

#define TR_FROM_UP 1 /* for traverse-direction */
#define TR_FROM_DN 2

#define TRI_LHS 1
#define TRI_RHS 2

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

#define CROSS(v0, v1, v2)                                                      \
  (((v1).x - (v0).x) * ((v2).y - (v0).y) -                                     \
   ((v1).y - (v0).y) * ((v2).x - (v0).x))

#define DOT(v0, v1) ((v0).x * (v1).x + (v0).y * (v1).y)

#define FP_EQUAL(s, t) (fabs(s - t) <= C_EPS)

/* Global variables */

static QueryNode query_structure[QSIZE]; /* Query structure */
static Trapezoid trapezoids[TRSIZE];     /* Trapezoid structure */
static Segment segments[MAX_SEGMENTS];   /* Segment table */

#define CROSS_SINE(v0, v1) ((v0).x * (v1).y - (v1).x * (v0).y)
#define LENGTH(v0) (sqrt((v0).x * (v0).x + (v0).y * (v0).y))

static MonotoneChain
    monotone_chain[TRSIZE]; /* Table to hold all the monotone */
                            /* polygons . Each monotone polygon */
                            /* is a circularly linked list */

static VertexChain
    vertex_chain[MAX_SEGMENTS]; /* chain init. information. This */
                                /* is used to decide which */
                                /* monotone polygon to split if */
                                /* there are several other */
                                /* polygons touching at the same */
                                /* vertex  */

static int mon[MAX_SEGMENTS]; /* contains position of any vertex in */
                              /* the monotone chain for the polygon */
static int visited[TRSIZE];
static int chain_idx, mon_idx;

static int triangulate_single_polygon(int, int, int, TriangleArray *);
static int traverse_polygon(int, int, int, int);

/* Return the maximum of the two points into the yval structure */
static int _max(Coord2 *yval, Coord2 *v0, Coord2 *v1) {
  if (v0->y > v1->y + C_EPS)
    *yval = *v0;
  else if (FP_EQUAL(v0->y, v1->y)) {
    if (v0->x > v1->x + C_EPS)
      *yval = *v0;
    else
      *yval = *v1;
  } else
    *yval = *v1;

  return 0;
}

/* Return the minimum of the two points into the yval structure */
static int _min(Coord2 *yval, Coord2 *v0, Coord2 *v1) {
  if (v0->y < v1->y - C_EPS)
    *yval = *v0;
  else if (FP_EQUAL(v0->y, v1->y)) {
    if (v0->x < v1->x)
      *yval = *v0;
    else
      *yval = *v1;
  } else
    *yval = *v1;

  return 0;
}

static int _greater_than(Coord2 *v0, Coord2 *v1) {
  if (v0->y > v1->y + C_EPS)
    return TRUE;
  else if (v0->y < v1->y - C_EPS)
    return FALSE;
  else
    return (v0->x > v1->x);
}

static int _equal_to(Coord2 *v0, Coord2 *v1) {
  return (FP_EQUAL(v0->y, v1->y) && FP_EQUAL(v0->x, v1->x));
}

static int _greater_than_equal_to(Coord2 *v0, Coord2 *v1) {
  if (v0->y > v1->y + C_EPS)
    return TRUE;
  else if (v0->y < v1->y - C_EPS)
    return FALSE;
  else
    return (v0->x >= v1->x);
}

static int _less_than(Coord2 *v0, Coord2 *v1) {
  if (v0->y < v1->y - C_EPS)
    return TRUE;
  else if (v0->y > v1->y + C_EPS)
    return FALSE;
  else
    return (v0->x < v1->x);
}

/* Function returns TRUE if the trapezoid lies inside the polygon */
static int inside_polygon(Trapezoid *t) {
  int rseg = t->right_segment;

  if (t->state == ST_INVALID)
    return 0;

  if ((t->left_segment <= 0) || (t->right_segment <= 0))
    return 0;

  if (((t->u0 <= 0) && (t->u1 <= 0)) ||
      ((t->d0 <= 0) && (t->d1 <= 0))) /* triangle */
    return (_greater_than(&segments[rseg].v1, &segments[rseg].v0));

  return 0;
}

/* return a new mon structure from the table */
static int newmon(void) { return ++mon_idx; }

/* return a new chain element from the table */
static int new_chain_element(void) { return ++chain_idx; }

static F64 get_angle(Coord2 *vp0, Coord2 *vpnext, Coord2 *vp1) {
  Coord2 v0, v1;

  v0.x = vpnext->x - vp0->x;
  v0.y = vpnext->y - vp0->y;

  v1.x = vp1->x - vp0->x;
  v1.y = vp1->y - vp0->y;

  if (CROSS_SINE(v0, v1) >= 0) /* sine is positive */
    return DOT(v0, v1) / LENGTH(v0) / LENGTH(v1);
  else
    return (-1.0 * DOT(v0, v1) / LENGTH(v0) / LENGTH(v1) - 2);
}

/* (v0, v1) is the new diagonal to be added to the polygon. Find which */
/* chain to use and return the positions of v0 and v1 in p and q */
static int get_vertex_positions(int v0, int v1, int *ip, int *iq) {
  VertexChain *vp0, *vp1;
  int i;
  F64 angle, temp;
  int tp, tq;

  vp0 = &vertex_chain[v0];
  vp1 = &vertex_chain[v1];

  /* p is identified as follows. Scan from (v0, v1) rightwards till */
  /* you hit the first segment starting from v0. That chain is the */
  /* chain of our interest */

  angle = -4.0;
  for (i = 0; i < 4; i++) {
    if (vp0->vnext[i] <= 0)
      continue;
    if ((temp = get_angle(&vp0->pt, &(vertex_chain[vp0->vnext[i]].pt),
                          &vp1->pt)) > angle) {
      angle = temp;
      tp = i;
    }
  }

  *ip = tp;

  /* Do similar actions for q */

  angle = -4.0;
  for (i = 0; i < 4; i++) {
    if (vp1->vnext[i] <= 0)
      continue;
    if ((temp = get_angle(&vp1->pt, &(vertex_chain[vp1->vnext[i]].pt),
                          &vp0->pt)) > angle) {
      angle = temp;
      tq = i;
    }
  }

  *iq = tq;

  return 0;
}

/* v0 and v1 are specified in anti-clockwise order with respect to
 * the current monotone polygon mcur. Split the current polygon into
 * two polygons using the diagonal (v0, v1)
 */
static int make_new_monotone_poly(int mcur, int v0, int v1) {
  int p, q, ip, iq;
  int mnew = newmon();
  int i, j, nf0, nf1;
  VertexChain *vp0, *vp1;

  vp0 = &vertex_chain[v0];
  vp1 = &vertex_chain[v1];

  get_vertex_positions(v0, v1, &ip, &iq);

  p = vp0->vpos[ip];
  q = vp1->vpos[iq];

  /* At this stage, we have got the positions of v0 and v1 in the */
  /* desired chain. Now modify the linked lists */

  i = new_chain_element(); /* for the new list */
  j = new_chain_element();

  monotone_chain[i].vnum = v0;
  monotone_chain[j].vnum = v1;

  monotone_chain[i].next = monotone_chain[p].next;
  monotone_chain[monotone_chain[p].next].prev = i;
  monotone_chain[i].prev = j;
  monotone_chain[j].next = i;
  monotone_chain[j].prev = monotone_chain[q].prev;
  monotone_chain[monotone_chain[q].prev].next = j;

  monotone_chain[p].next = q;
  monotone_chain[q].prev = p;

  nf0 = vp0->nextfree;
  nf1 = vp1->nextfree;

  vp0->vnext[ip] = v1;

  vp0->vpos[nf0] = i;
  vp0->vnext[nf0] = monotone_chain[monotone_chain[i].next].vnum;
  vp1->vpos[nf1] = j;
  vp1->vnext[nf1] = v0;

  vp0->nextfree++;
  vp1->nextfree++;

#ifdef DEBUG
  fprintf(stderr, "make_poly: mcur = %d, (v0, v1) = (%d, %d)\n", mcur, v0, v1);
  fprintf(stderr, "next posns = (p, q) = (%d, %d)\n", p, q);
#endif

  mon[mcur] = p;
  mon[mnew] = i;
  return mnew;
}

/* Main routine to get monotone polygons from the trapezoidation of
 * the polygon.
 */

static int monotonate_trapezoids(int n) {
  int i;
  int tr_start;

  memset((void *)vertex_chain, 0, sizeof(vertex_chain));
  memset((void *)visited, 0, sizeof(visited));
  memset((void *)monotone_chain, 0, sizeof(monotone_chain));
  memset((void *)mon, 0, sizeof(mon));

  /* First locate a trapezoid which lies inside the polygon */
  /* and which is triangular */
  for (i = 0; i < TRSIZE; i++)
    if (inside_polygon(&trapezoids[i]))
      break;
  tr_start = i;

  /* Initialise the mon data-structure and start spanning all the */
  /* trapezoids within the polygon */

  for (i = 1; i <= n; i++) {
    monotone_chain[i].prev = segments[i].prev;
    monotone_chain[i].next = segments[i].next;
    monotone_chain[i].vnum = i;
    vertex_chain[i].pt = segments[i].v0;
    vertex_chain[i].vnext[0] = segments[i].next; /* next vertex */
    vertex_chain[i].vpos[0] = i;                 /* locn. of next vertex */
    vertex_chain[i].nextfree = 1;
  }

  chain_idx = n;
  mon_idx = 0;
  mon[0] = 1; /* position of any vertex in the first */
  /* chain  */

  /* traverse the polygon */
  if (trapezoids[tr_start].u0 > 0)
    traverse_polygon(0, tr_start, trapezoids[tr_start].u0, TR_FROM_UP);
  else if (trapezoids[tr_start].d0 > 0)
    traverse_polygon(0, tr_start, trapezoids[tr_start].d0, TR_FROM_DN);

  /* return the number of polygons created */
  return newmon();
}

/* recursively visit all the trapezoids */
static int traverse_polygon(int mcur, int trnum, int from, int dir) {
  Trapezoid *t = &trapezoids[trnum];
  int mnew;
  int v0, v1;
  int retval = 0;

  if ((trnum <= 0) || visited[trnum])
    return 0;

#ifdef DEBUG
  fprintf(stderr, "traversal %d -> %d\n", from, trnum);
#endif
  visited[trnum] = TRUE;

  /* We have much more information available here. */
  /* rseg: goes upwards   */
  /* lseg: goes downwards */

  /* Initially assume that dir = TR_FROM_DN (from the left) */
  /* Switch v0 and v1 if necessary afterwards */

  /* special cases for triangles with cusps at the opposite ends. */
  /* take care of this first */
  if ((t->u0 <= 0) && (t->u1 <= 0)) {
    if ((t->d0 > 0) && (t->d1 > 0)) /* downward opening triangle */
    {
      v0 = trapezoids[t->d1].left_segment;
      v1 = t->left_segment;
      if (from == t->d1) {
        mnew = make_new_monotone_poly(mcur, v1, v0);
        traverse_polygon(mcur, t->d1, trnum, TR_FROM_UP);
        traverse_polygon(mnew, t->d0, trnum, TR_FROM_UP);
      } else {
        mnew = make_new_monotone_poly(mcur, v0, v1);
        traverse_polygon(mcur, t->d0, trnum, TR_FROM_UP);
        traverse_polygon(mnew, t->d1, trnum, TR_FROM_UP);
      }
    } else {
      retval = SP_NOSPLIT; /* Just traverse all neighbours */
      traverse_polygon(mcur, t->u0, trnum, TR_FROM_DN);
      traverse_polygon(mcur, t->u1, trnum, TR_FROM_DN);
      traverse_polygon(mcur, t->d0, trnum, TR_FROM_UP);
      traverse_polygon(mcur, t->d1, trnum, TR_FROM_UP);
    }
  }

  else if ((t->d0 <= 0) && (t->d1 <= 0)) {
    if ((t->u0 > 0) && (t->u1 > 0)) /* upward opening triangle */
    {
      v0 = t->right_segment;
      v1 = trapezoids[t->u0].right_segment;
      if (from == t->u1) {
        mnew = make_new_monotone_poly(mcur, v1, v0);
        traverse_polygon(mcur, t->u1, trnum, TR_FROM_DN);
        traverse_polygon(mnew, t->u0, trnum, TR_FROM_DN);
      } else {
        mnew = make_new_monotone_poly(mcur, v0, v1);
        traverse_polygon(mcur, t->u0, trnum, TR_FROM_DN);
        traverse_polygon(mnew, t->u1, trnum, TR_FROM_DN);
      }
    } else {
      retval = SP_NOSPLIT; /* Just traverse all neighbours */
      traverse_polygon(mcur, t->u0, trnum, TR_FROM_DN);
      traverse_polygon(mcur, t->u1, trnum, TR_FROM_DN);
      traverse_polygon(mcur, t->d0, trnum, TR_FROM_UP);
      traverse_polygon(mcur, t->d1, trnum, TR_FROM_UP);
    }
  }

  else if ((t->u0 > 0) && (t->u1 > 0)) {
    if ((t->d0 > 0) && (t->d1 > 0)) /* downward + upward cusps */
    {
      v0 = trapezoids[t->d1].left_segment;
      v1 = trapezoids[t->u0].right_segment;
      retval = SP_2UP_2DN;
      if (((dir == TR_FROM_DN) && (t->d1 == from)) ||
          ((dir == TR_FROM_UP) && (t->u1 == from))) {
        mnew = make_new_monotone_poly(mcur, v1, v0);
        traverse_polygon(mcur, t->u1, trnum, TR_FROM_DN);
        traverse_polygon(mcur, t->d1, trnum, TR_FROM_UP);
        traverse_polygon(mnew, t->u0, trnum, TR_FROM_DN);
        traverse_polygon(mnew, t->d0, trnum, TR_FROM_UP);
      } else {
        mnew = make_new_monotone_poly(mcur, v0, v1);
        traverse_polygon(mcur, t->u0, trnum, TR_FROM_DN);
        traverse_polygon(mcur, t->d0, trnum, TR_FROM_UP);
        traverse_polygon(mnew, t->u1, trnum, TR_FROM_DN);
        traverse_polygon(mnew, t->d1, trnum, TR_FROM_UP);
      }
    } else /* only downward cusp */
    {
      if (_equal_to(&t->lo, &segments[t->left_segment].v1)) {
        v0 = trapezoids[t->u0].right_segment;
        v1 = segments[t->left_segment].next;

        retval = SP_2UP_LEFT;
        if ((dir == TR_FROM_UP) && (t->u0 == from)) {
          mnew = make_new_monotone_poly(mcur, v1, v0);
          traverse_polygon(mcur, t->u0, trnum, TR_FROM_DN);
          traverse_polygon(mnew, t->d0, trnum, TR_FROM_UP);
          traverse_polygon(mnew, t->u1, trnum, TR_FROM_DN);
          traverse_polygon(mnew, t->d1, trnum, TR_FROM_UP);
        } else {
          mnew = make_new_monotone_poly(mcur, v0, v1);
          traverse_polygon(mcur, t->u1, trnum, TR_FROM_DN);
          traverse_polygon(mcur, t->d0, trnum, TR_FROM_UP);
          traverse_polygon(mcur, t->d1, trnum, TR_FROM_UP);
          traverse_polygon(mnew, t->u0, trnum, TR_FROM_DN);
        }
      } else {
        v0 = t->right_segment;
        v1 = trapezoids[t->u0].right_segment;
        retval = SP_2UP_RIGHT;
        if ((dir == TR_FROM_UP) && (t->u1 == from)) {
          mnew = make_new_monotone_poly(mcur, v1, v0);
          traverse_polygon(mcur, t->u1, trnum, TR_FROM_DN);
          traverse_polygon(mnew, t->d1, trnum, TR_FROM_UP);
          traverse_polygon(mnew, t->d0, trnum, TR_FROM_UP);
          traverse_polygon(mnew, t->u0, trnum, TR_FROM_DN);
        } else {
          mnew = make_new_monotone_poly(mcur, v0, v1);
          traverse_polygon(mcur, t->u0, trnum, TR_FROM_DN);
          traverse_polygon(mcur, t->d0, trnum, TR_FROM_UP);
          traverse_polygon(mcur, t->d1, trnum, TR_FROM_UP);
          traverse_polygon(mnew, t->u1, trnum, TR_FROM_DN);
        }
      }
    }
  } else if ((t->u0 > 0) || (t->u1 > 0)) /* no downward cusp */
  {
    if ((t->d0 > 0) && (t->d1 > 0)) /* only upward cusp */
    {
      if (_equal_to(&t->hi, &segments[t->left_segment].v0)) {
        v0 = trapezoids[t->d1].left_segment;
        v1 = t->left_segment;
        retval = SP_2DN_LEFT;
        if (!((dir == TR_FROM_DN) && (t->d0 == from))) {
          mnew = make_new_monotone_poly(mcur, v1, v0);
          traverse_polygon(mcur, t->u1, trnum, TR_FROM_DN);
          traverse_polygon(mcur, t->d1, trnum, TR_FROM_UP);
          traverse_polygon(mcur, t->u0, trnum, TR_FROM_DN);
          traverse_polygon(mnew, t->d0, trnum, TR_FROM_UP);
        } else {
          mnew = make_new_monotone_poly(mcur, v0, v1);
          traverse_polygon(mcur, t->d0, trnum, TR_FROM_UP);
          traverse_polygon(mnew, t->u0, trnum, TR_FROM_DN);
          traverse_polygon(mnew, t->u1, trnum, TR_FROM_DN);
          traverse_polygon(mnew, t->d1, trnum, TR_FROM_UP);
        }
      } else {
        v0 = trapezoids[t->d1].left_segment;
        v1 = segments[t->right_segment].next;

        retval = SP_2DN_RIGHT;
        if ((dir == TR_FROM_DN) && (t->d1 == from)) {
          mnew = make_new_monotone_poly(mcur, v1, v0);
          traverse_polygon(mcur, t->d1, trnum, TR_FROM_UP);
          traverse_polygon(mnew, t->u1, trnum, TR_FROM_DN);
          traverse_polygon(mnew, t->u0, trnum, TR_FROM_DN);
          traverse_polygon(mnew, t->d0, trnum, TR_FROM_UP);
        } else {
          mnew = make_new_monotone_poly(mcur, v0, v1);
          traverse_polygon(mcur, t->u0, trnum, TR_FROM_DN);
          traverse_polygon(mcur, t->d0, trnum, TR_FROM_UP);
          traverse_polygon(mcur, t->u1, trnum, TR_FROM_DN);
          traverse_polygon(mnew, t->d1, trnum, TR_FROM_UP);
        }
      }
    } else /* no cusp */
    {
      if (_equal_to(&t->hi, &segments[t->left_segment].v0) &&
          _equal_to(&t->lo, &segments[t->right_segment].v0)) {
        v0 = t->right_segment;
        v1 = t->left_segment;
        retval = SP_SIMPLE_LRDN;
        if (dir == TR_FROM_UP) {
          mnew = make_new_monotone_poly(mcur, v1, v0);
          traverse_polygon(mcur, t->u0, trnum, TR_FROM_DN);
          traverse_polygon(mcur, t->u1, trnum, TR_FROM_DN);
          traverse_polygon(mnew, t->d1, trnum, TR_FROM_UP);
          traverse_polygon(mnew, t->d0, trnum, TR_FROM_UP);
        } else {
          mnew = make_new_monotone_poly(mcur, v0, v1);
          traverse_polygon(mcur, t->d1, trnum, TR_FROM_UP);
          traverse_polygon(mcur, t->d0, trnum, TR_FROM_UP);
          traverse_polygon(mnew, t->u0, trnum, TR_FROM_DN);
          traverse_polygon(mnew, t->u1, trnum, TR_FROM_DN);
        }
      } else if (_equal_to(&t->hi, &segments[t->right_segment].v1) &&
                 _equal_to(&t->lo, &segments[t->left_segment].v1)) {
        v0 = segments[t->right_segment].next;
        v1 = segments[t->left_segment].next;

        retval = SP_SIMPLE_LRUP;
        if (dir == TR_FROM_UP) {
          mnew = make_new_monotone_poly(mcur, v1, v0);
          traverse_polygon(mcur, t->u0, trnum, TR_FROM_DN);
          traverse_polygon(mcur, t->u1, trnum, TR_FROM_DN);
          traverse_polygon(mnew, t->d1, trnum, TR_FROM_UP);
          traverse_polygon(mnew, t->d0, trnum, TR_FROM_UP);
        } else {
          mnew = make_new_monotone_poly(mcur, v0, v1);
          traverse_polygon(mcur, t->d1, trnum, TR_FROM_UP);
          traverse_polygon(mcur, t->d0, trnum, TR_FROM_UP);
          traverse_polygon(mnew, t->u0, trnum, TR_FROM_DN);
          traverse_polygon(mnew, t->u1, trnum, TR_FROM_DN);
        }
      } else /* no split possible */
      {
        retval = SP_NOSPLIT;
        traverse_polygon(mcur, t->u0, trnum, TR_FROM_DN);
        traverse_polygon(mcur, t->d0, trnum, TR_FROM_UP);
        traverse_polygon(mcur, t->u1, trnum, TR_FROM_DN);
        traverse_polygon(mcur, t->d1, trnum, TR_FROM_UP);
      }
    }
  }

  return retval;
}

/* For each monotone polygon, find the ymax and ymin (to determine the */
/* two y-monotone chains) and pass on this monotone polygon for greedy */
/* triangulation. */
/* Take care not to triangulate duplicate monotone polygons */

static void triangulate_monotone_polygons(int nvert, int nmonpoly,
                                          TriangleArray *op) {
  int i;
  Coord2 ymax, ymin;
  int p, vfirst, posmax, posmin, v;
  int vcount, processed;

#ifdef DEBUG
  for (i = 0; i < nmonpoly; i++) {
    fprintf(stderr, "\n\nPolygon %d: ", i);
    vfirst = monotone_chain[mon[i]].vnum;
    p = monotone_chain[mon[i]].next;
    fprintf(stderr, "%d ", monotone_chain[mon[i]].vnum);
    while (monotone_chain[p].vnum != vfirst) {
      fprintf(stderr, "%d ", monotone_chain[p].vnum);
      p = monotone_chain[p].next;
    }
  }
  fprintf(stderr, "\n");
#endif

  for (i = 0; i < nmonpoly; i++) {
    vcount = 1;
    processed = FALSE;
    vfirst = monotone_chain[mon[i]].vnum;
    ymax = ymin = vertex_chain[vfirst].pt;
    posmax = posmin = mon[i];
    monotone_chain[mon[i]].marked = TRUE;
    p = monotone_chain[mon[i]].next;
    while ((v = monotone_chain[p].vnum) != vfirst) {
      if (monotone_chain[p].marked) {
        processed = TRUE;
        break; /* break from while */
      } else
        monotone_chain[p].marked = TRUE;

      if (_greater_than(&vertex_chain[v].pt, &ymax)) {
        ymax = vertex_chain[v].pt;
        posmax = p;
      }
      if (_less_than(&vertex_chain[v].pt, &ymin)) {
        ymin = vertex_chain[v].pt;
        posmin = p;
      }
      p = monotone_chain[p].next;
      vcount++;
    }

    if (processed) /* Go to next polygon */
      continue;

    if (vcount == 3) /* already a triangle */
    {
      TriangleArrayPush(op, (Triangle){
                                monotone_chain[p].vnum,
                                monotone_chain[monotone_chain[p].next].vnum,
                                monotone_chain[monotone_chain[p].prev].vnum,

                            });
    } else /* triangulate the polygon */
    {
      v = monotone_chain[monotone_chain[posmax].next].vnum;
      if (_equal_to(&vertex_chain[v].pt, &ymin)) { /* LHS is a single line */
        triangulate_single_polygon(nvert, posmax, TRI_LHS, op);
      } else
        triangulate_single_polygon(nvert, posmax, TRI_RHS, op);
    }
  }

#ifdef DEBUG
  {
    for (size_t i = 0; i < TriangleArrayLength(op); i++)
      fprintf(stderr, "tri #%ld: (%d, %d, %d)\n", i, op->data[i].a,
              op->data[i].b, op->data[i].c);
  }
#endif
}

/* A greedy corner-cutting algorithm to triangulate a y-monotone
 * polygon in O(n) time.
 * Joseph O-Rourke, Computational Geometry in C. Page 47
 */
static int triangulate_single_polygon(int nvert, int posmax, int side,
                                      TriangleArray *op) {
  int v;
  int rc[MAX_SEGMENTS], ri = 0; /* reflex chain */
  int endv, tmp, vpos;

  if (side == TRI_RHS) /* RHS segment is a single segment */
  {
    rc[0] = monotone_chain[posmax].vnum;
    tmp = monotone_chain[posmax].next;
    rc[1] = monotone_chain[tmp].vnum;
    ri = 1;

    vpos = monotone_chain[tmp].next;
    v = monotone_chain[vpos].vnum;

    if ((endv = monotone_chain[monotone_chain[posmax].prev].vnum) == 0)
      endv = nvert;
  } else /* LHS is a single segment */
  {
    tmp = monotone_chain[posmax].next;
    rc[0] = monotone_chain[tmp].vnum;
    tmp = monotone_chain[tmp].next;
    rc[1] = monotone_chain[tmp].vnum;
    ri = 1;

    vpos = monotone_chain[tmp].next;
    v = monotone_chain[vpos].vnum;

    endv = monotone_chain[posmax].vnum;
  }

  while ((v != endv) || (ri > 1)) {
    if (ri > 0) /* reflex chain is non-empty */
    {
      if (CROSS(vertex_chain[v].pt, vertex_chain[rc[ri - 1]].pt,
                vertex_chain[rc[ri]].pt) > 0) { /* convex corner: cut if off */
        TriangleArrayPush(op, (Triangle){
                                  rc[ri - 1],
                                  rc[ri],
                                  v,
                              });
        ri--;
      } else /* non-convex */
      {      /* add v to the chain */
        ri++;
        rc[ri] = v;
        vpos = monotone_chain[vpos].next;
        v = monotone_chain[vpos].vnum;
      }
    } else /* reflex-chain empty: add v to the */
    {      /* reflex chain and advance it  */
      rc[++ri] = v;
      vpos = monotone_chain[vpos].next;
      v = monotone_chain[vpos].vnum;
    }
  } /* end-while */

  /* reached the bottom vertex. Add in the triangle formed */
  TriangleArrayPush(op, (Triangle){
                            rc[ri - 1],
                            rc[ri],
                            v,
                        });
  ri--;

  return 0;
}

static int q_idx;
static int tr_idx;

/* Return a new node to be added into the query tree */
static int newnode(void) {
  if (q_idx < QSIZE)
    return q_idx++;
  else {
    fprintf(stderr, "newnode: Query-table overflow\n");
    return -1;
  }
}

/* Return a free trapezoid */
static int NewTrapezoid(void) {
  if (tr_idx < TRSIZE) {
    trapezoids[tr_idx].left_segment = -1;
    trapezoids[tr_idx].right_segment = -1;
    trapezoids[tr_idx].state = ST_VALID;
    return tr_idx++;
  } else {
    fprintf(stderr, "newtrap: Trapezoid-table overflow\n");
    return -1;
  }
}

/* Initilialise the query structure (Q) and the trapezoid table (T)
 * when the first segment is added to start the trapezoidation. The
 * query-tree starts out with 4 trapezoids, one S-node and 2 Y-nodes
 *
 *                4
 *   -----------------------------------
 *  		  \
 *  	1	   \        2
 *  		    \
 *   -----------------------------------
 *                3
 */

static int init_query_structure(int segnum) {
  int i1, i2, i3, i4, i5, i6, i7, root;
  int t1, t2, t3, t4;
  Segment *s = &segments[segnum];

  q_idx = tr_idx = 1;
  memset((void *)trapezoids, 0, sizeof(trapezoids));
  memset((void *)query_structure, 0, sizeof(query_structure));

  i1 = newnode();
  query_structure[i1].nodetype = Y;
  _max(&query_structure[i1].yval, &s->v0, &s->v1); /* root */
  root = i1;

  query_structure[i1].right = i2 = newnode();
  query_structure[i2].nodetype = SINK;
  query_structure[i2].parent = i1;

  query_structure[i1].left = i3 = newnode();
  query_structure[i3].nodetype = Y;
  _min(&query_structure[i3].yval, &s->v0, &s->v1); /* root */
  query_structure[i3].parent = i1;

  query_structure[i3].left = i4 = newnode();
  query_structure[i4].nodetype = SINK;
  query_structure[i4].parent = i3;

  query_structure[i3].right = i5 = newnode();
  query_structure[i5].nodetype = X;
  query_structure[i5].segnum = segnum;
  query_structure[i5].parent = i3;

  query_structure[i5].left = i6 = newnode();
  query_structure[i6].nodetype = SINK;
  query_structure[i6].parent = i5;

  query_structure[i5].right = i7 = newnode();
  query_structure[i7].nodetype = SINK;
  query_structure[i7].parent = i5;

  t1 = NewTrapezoid(); /* middle left */
  t2 = NewTrapezoid(); /* middle right */
  t3 = NewTrapezoid(); /* bottom-most */
  t4 = NewTrapezoid(); /* topmost */

  trapezoids[t1].hi = trapezoids[t2].hi = trapezoids[t4].lo =
      query_structure[i1].yval;
  trapezoids[t1].lo = trapezoids[t2].lo = trapezoids[t3].hi =
      query_structure[i3].yval;
  trapezoids[t4].hi.y = (F64)(TRIANG_INFINITY);
  trapezoids[t4].hi.x = (F64)(TRIANG_INFINITY);
  trapezoids[t3].lo.y = (F64)-1 * (TRIANG_INFINITY);
  trapezoids[t3].lo.x = (F64)-1 * (TRIANG_INFINITY);
  trapezoids[t1].right_segment = trapezoids[t2].left_segment = segnum;
  trapezoids[t1].u0 = trapezoids[t2].u0 = t4;
  trapezoids[t1].d0 = trapezoids[t2].d0 = t3;
  trapezoids[t4].d0 = trapezoids[t3].u0 = t1;
  trapezoids[t4].d1 = trapezoids[t3].u1 = t2;

  trapezoids[t1].sink = i6;
  trapezoids[t2].sink = i7;
  trapezoids[t3].sink = i4;
  trapezoids[t4].sink = i2;

  trapezoids[t1].state = trapezoids[t2].state = ST_VALID;
  trapezoids[t3].state = trapezoids[t4].state = ST_VALID;

  query_structure[i2].trnum = t4;
  query_structure[i4].trnum = t3;
  query_structure[i6].trnum = t1;
  query_structure[i7].trnum = t2;

  s->is_inserted = TRUE;
  return root;
}

/* Retun TRUE if the vertex v is to the left of line segment no.
 * segnum. Takes care of the degenerate cases when both the vertices
 * have the same y--cood, etc.
 */

static int is_left_of(int segnum, Coord2 *v) {
  Segment *s = &segments[segnum];
  F64 area;

  if (_greater_than(&s->v1, &s->v0)) /* seg. going upwards */
  {
    if (FP_EQUAL(s->v1.y, v->y)) {
      if (v->x < s->v1.x)
        area = 1.0;
      else
        area = -1.0;
    } else if (FP_EQUAL(s->v0.y, v->y)) {
      if (v->x < s->v0.x)
        area = 1.0;
      else
        area = -1.0;
    } else
      area = CROSS(s->v0, s->v1, (*v));
  } else /* v0 > v1 */
  {
    if (FP_EQUAL(s->v1.y, v->y)) {
      if (v->x < s->v1.x)
        area = 1.0;
      else
        area = -1.0;
    } else if (FP_EQUAL(s->v0.y, v->y)) {
      if (v->x < s->v0.x)
        area = 1.0;
      else
        area = -1.0;
    } else
      area = CROSS(s->v1, s->v0, (*v));
  }

  if (area > 0.0)
    return TRUE;
  else
    return FALSE;
}

/* Returns true if the corresponding endpoint of the given segment is */
/* already inserted into the segment tree. Use the simple test of */
/* whether the segment which shares this endpoint is already inserted */

static int inserted(int segnum, int whichpt) {
  if (whichpt == FIRSTPT)
    return segments[segments[segnum].prev].is_inserted;
  else
    return segments[segments[segnum].next].is_inserted;
}

/* This is query routine which determines which trapezoid does the
 * point v lie in. The return value is the trapezoid number.
 */

static int locate_endpoint(Coord2 *v, Coord2 *vo, int r) {
  QueryNode *rptr = &query_structure[r];

  switch (rptr->nodetype) {
  case SINK:
    return rptr->trnum;

  case Y:
    if (_greater_than(v, &rptr->yval)) /* above */
      return locate_endpoint(v, vo, rptr->right);
    else if (_equal_to(v, &rptr->yval))   /* the point is already */
    {                                     /* inserted. */
      if (_greater_than(vo, &rptr->yval)) /* above */
        return locate_endpoint(v, vo, rptr->right);
      else
        return locate_endpoint(v, vo, rptr->left); /* below */
    } else
      return locate_endpoint(v, vo, rptr->left); /* below */

  case X:
    if (_equal_to(v, &segments[rptr->segnum].v0) ||
        _equal_to(v, &segments[rptr->segnum].v1)) {
      if (FP_EQUAL(v->y, vo->y)) /* horizontal segment */
      {
        if (vo->x < v->x)
          return locate_endpoint(v, vo, rptr->left); /* left */
        else
          return locate_endpoint(v, vo, rptr->right); /* right */
      }

      else if (is_left_of(rptr->segnum, vo))
        return locate_endpoint(v, vo, rptr->left); /* left */
      else
        return locate_endpoint(v, vo, rptr->right); /* right */
    } else if (is_left_of(rptr->segnum, v))
      return locate_endpoint(v, vo, rptr->left); /* left */
    else
      return locate_endpoint(v, vo, rptr->right); /* right */

  default:
    ERROR_MSG("Haggu !!!!!\n")
  }
}

/* Thread in the segment into the existing trapezoidation. The
 * limiting trapezoids are given by tfirst and tlast (which are the
 * trapezoids containing the two endpoints of the segment. Merges all
 * possible trapezoids which flank this segment and have been recently
 * divided because of its insertion
 */

static int merge_trapezoids(int segnum, int tfirst, int tlast, int side) {
  int t, tnext, cond;
  int ptnext;

  /* First merge polys on the LHS */
  t = tfirst;
  while ((t > 0) &&
         _greater_than_equal_to(&trapezoids[t].lo, &trapezoids[tlast].lo)) {
    if (side == S_LEFT)
      cond = ((((tnext = trapezoids[t].d0) > 0) &&
               (trapezoids[tnext].right_segment == segnum)) ||
              (((tnext = trapezoids[t].d1) > 0) &&
               (trapezoids[tnext].right_segment == segnum)));
    else
      cond = ((((tnext = trapezoids[t].d0) > 0) &&
               (trapezoids[tnext].left_segment == segnum)) ||
              (((tnext = trapezoids[t].d1) > 0) &&
               (trapezoids[tnext].left_segment == segnum)));

    if (cond) {
      if ((trapezoids[t].left_segment == trapezoids[tnext].left_segment) &&
          (trapezoids[t].right_segment == trapezoids[tnext].right_segment)) /* good neighbours */
      {                                                   /* merge them */
        /* Use the upper node as the new node i.e. t */

        ptnext = query_structure[trapezoids[tnext].sink].parent;

        if (query_structure[ptnext].left == trapezoids[tnext].sink)
          query_structure[ptnext].left = trapezoids[t].sink;
        else
          query_structure[ptnext].right =
              trapezoids[t].sink; /* redirect parent */

        /* Change the upper neighbours of the lower trapezoids */

        if ((trapezoids[t].d0 = trapezoids[tnext].d0) > 0) {
          if (trapezoids[trapezoids[t].d0].u0 == tnext)
            trapezoids[trapezoids[t].d0].u0 = t;
          else {
            if (trapezoids[trapezoids[t].d0].u1 == tnext)
              trapezoids[trapezoids[t].d0].u1 = t;
          }
        }

        if ((trapezoids[t].d1 = trapezoids[tnext].d1) > 0) {
          if (trapezoids[trapezoids[t].d1].u0 == tnext)
            trapezoids[trapezoids[t].d1].u0 = t;
          else if (trapezoids[trapezoids[t].d1].u1 == tnext)
            trapezoids[trapezoids[t].d1].u1 = t;
        }

        trapezoids[t].lo = trapezoids[tnext].lo;
        trapezoids[tnext].state = ST_INVALID; /* invalidate the lower */
                                              /* trapezium */
      } else                                  /* not good neighbours */
        t = tnext;
    } else /* do not satisfy the outer if */
      t = tnext;

  } /* end-while */

  return 0;
}

/* Add in the new segment into the trapezoidation and update Q and T
 * structures. First locate the two endpoints of the segment in the
 * Q-structure. Then start from the topmost trapezoid and go down to
 * the  lower trapezoid dividing all the trapezoids in between .
 */

static int add_segment(int segnum) {
  Segment s;
  int tu, tl, sk, tfirst, tlast;
  int tfirstr, tlastr, tfirstl, tlastl;
  int i1, i2, t, tn;
  Coord2 tpt;
  int tribot = 0, is_swapped = 0;
  int tmptriseg;

  s = segments[segnum];
  if (_greater_than(&s.v1, &s.v0)) /* Get higher vertex in v0 */
  {
    int tmp;
    tpt = s.v0;
    s.v0 = s.v1;
    s.v1 = tpt;
    tmp = s.root0;
    s.root0 = s.root1;
    s.root1 = tmp;
    is_swapped = TRUE;
  }

  if ((is_swapped) ? !inserted(segnum, LASTPT)
                   : !inserted(segnum, FIRSTPT)) /* insert v0 in the tree */
  {
    int tmp_d;

    tu = locate_endpoint(&s.v0, &s.v1, s.root0);
    tl = NewTrapezoid(); /* tl is the new lower trapezoid */
    trapezoids[tl].state = ST_VALID;
    trapezoids[tl] = trapezoids[tu];
    trapezoids[tu].lo.y = trapezoids[tl].hi.y = s.v0.y;
    trapezoids[tu].lo.x = trapezoids[tl].hi.x = s.v0.x;
    trapezoids[tu].d0 = tl;
    trapezoids[tu].d1 = 0;
    trapezoids[tl].u0 = tu;
    trapezoids[tl].u1 = 0;

    if (((tmp_d = trapezoids[tl].d0) > 0) && (trapezoids[tmp_d].u0 == tu))
      trapezoids[tmp_d].u0 = tl;
    if (((tmp_d = trapezoids[tl].d0) > 0) && (trapezoids[tmp_d].u1 == tu))
      trapezoids[tmp_d].u1 = tl;

    if (((tmp_d = trapezoids[tl].d1) > 0) && (trapezoids[tmp_d].u0 == tu))
      trapezoids[tmp_d].u0 = tl;
    if (((tmp_d = trapezoids[tl].d1) > 0) && (trapezoids[tmp_d].u1 == tu))
      trapezoids[tmp_d].u1 = tl;

    /* Now update the query structure and obtain the sinks for the */
    /* two trapezoids */

    i1 = newnode(); /* Upper trapezoid sink */
    i2 = newnode(); /* Lower trapezoid sink */
    sk = trapezoids[tu].sink;

    query_structure[sk].nodetype = Y;
    query_structure[sk].yval = s.v0;
    query_structure[sk].segnum = segnum; /* not really reqd ... maybe later */
    query_structure[sk].left = i2;
    query_structure[sk].right = i1;

    query_structure[i1].nodetype = SINK;
    query_structure[i1].trnum = tu;
    query_structure[i1].parent = sk;

    query_structure[i2].nodetype = SINK;
    query_structure[i2].trnum = tl;
    query_structure[i2].parent = sk;

    trapezoids[tu].sink = i1;
    trapezoids[tl].sink = i2;
    tfirst = tl;
  } else /* v0 already present */
  {      /* Get the topmost intersecting trapezoid */
    tfirst = locate_endpoint(&s.v0, &s.v1, s.root0);
  }

  if ((is_swapped) ? !inserted(segnum, FIRSTPT)
                   : !inserted(segnum, LASTPT)) /* insert v1 in the tree */
  {
    int tmp_d;

    tu = locate_endpoint(&s.v1, &s.v0, s.root1);

    tl = NewTrapezoid(); /* tl is the new lower trapezoid */
    trapezoids[tl].state = ST_VALID;
    trapezoids[tl] = trapezoids[tu];
    trapezoids[tu].lo.y = trapezoids[tl].hi.y = s.v1.y;
    trapezoids[tu].lo.x = trapezoids[tl].hi.x = s.v1.x;
    trapezoids[tu].d0 = tl;
    trapezoids[tu].d1 = 0;
    trapezoids[tl].u0 = tu;
    trapezoids[tl].u1 = 0;

    if (((tmp_d = trapezoids[tl].d0) > 0) && (trapezoids[tmp_d].u0 == tu))
      trapezoids[tmp_d].u0 = tl;
    if (((tmp_d = trapezoids[tl].d0) > 0) && (trapezoids[tmp_d].u1 == tu))
      trapezoids[tmp_d].u1 = tl;

    if (((tmp_d = trapezoids[tl].d1) > 0) && (trapezoids[tmp_d].u0 == tu))
      trapezoids[tmp_d].u0 = tl;
    if (((tmp_d = trapezoids[tl].d1) > 0) && (trapezoids[tmp_d].u1 == tu))
      trapezoids[tmp_d].u1 = tl;

    /* Now update the query structure and obtain the sinks for the */
    /* two trapezoids */

    i1 = newnode(); /* Upper trapezoid sink */
    i2 = newnode(); /* Lower trapezoid sink */
    sk = trapezoids[tu].sink;

    query_structure[sk].nodetype = Y;
    query_structure[sk].yval = s.v1;
    query_structure[sk].segnum = segnum; /* not really reqd ... maybe later */
    query_structure[sk].left = i2;
    query_structure[sk].right = i1;

    query_structure[i1].nodetype = SINK;
    query_structure[i1].trnum = tu;
    query_structure[i1].parent = sk;

    query_structure[i2].nodetype = SINK;
    query_structure[i2].trnum = tl;
    query_structure[i2].parent = sk;

    trapezoids[tu].sink = i1;
    trapezoids[tl].sink = i2;
    tlast = tu;
  } else /* v1 already present */
  {      /* Get the lowermost intersecting trapezoid */
    tlast = locate_endpoint(&s.v1, &s.v0, s.root1);
    tribot = 1;
  }

  /* Thread the segment into the query tree creating a new X-node */
  /* First, split all the trapezoids which are intersected by s into */
  /* two */

  t = tfirst; /* topmost trapezoid */

  while ((t > 0) &&
         _greater_than_equal_to(&trapezoids[t].lo, &trapezoids[tlast].lo))
  /* traverse from top to bot */
  {
    int t_sav, tn_sav;
    sk = trapezoids[t].sink;
    i1 = newnode(); /* left trapezoid sink */
    i2 = newnode(); /* right trapezoid sink */

    query_structure[sk].nodetype = X;
    query_structure[sk].segnum = segnum;
    query_structure[sk].left = i1;
    query_structure[sk].right = i2;

    query_structure[i1].nodetype = SINK; /* left trapezoid (use existing one) */
    query_structure[i1].trnum = t;
    query_structure[i1].parent = sk;

    query_structure[i2].nodetype = SINK; /* right trapezoid (allocate new) */
    query_structure[i2].trnum = tn = NewTrapezoid();
    trapezoids[tn].state = ST_VALID;
    query_structure[i2].parent = sk;

    if (t == tfirst)
      tfirstr = tn;
    if (_equal_to(&trapezoids[t].lo, &trapezoids[tlast].lo))
      tlastr = tn;

    trapezoids[tn] = trapezoids[t];
    trapezoids[t].sink = i1;
    trapezoids[tn].sink = i2;
    t_sav = t;
    tn_sav = tn;

    /* error */

    if ((trapezoids[t].d0 <= 0) &&
        (trapezoids[t].d1 <= 0)) /* case cannot arise */
    {
      fprintf(stderr, "add_segment: error\n");
      break;
    }

    /* only one trapezoid below. partition t into two and make the */
    /* two resulting trapezoids t and tn as the upper neighbours of */
    /* the sole lower trapezoid */

    else if ((trapezoids[t].d0 > 0) &&
             (trapezoids[t].d1 <= 0)) { /* Only one trapezoid below */
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
          } else /* cusp going leftwards */
          {
            trapezoids[tn].u0 = trapezoids[tn].u1 = trapezoids[t].u1 = -1;
            trapezoids[trapezoids[t].u0].d0 = t;
          }
        } else /* fresh segment */
        {
          trapezoids[trapezoids[t].u0].d0 = t;
          trapezoids[trapezoids[t].u0].d1 = tn;
        }
      }

      if (FP_EQUAL(trapezoids[t].lo.y, trapezoids[tlast].lo.y) &&
          FP_EQUAL(trapezoids[t].lo.x, trapezoids[tlast].lo.x) &&
          tribot) { /* bottom forms a triangle */

        if (is_swapped)
          tmptriseg = segments[segnum].prev;
        else
          tmptriseg = segments[segnum].next;

        if ((tmptriseg > 0) && is_left_of(tmptriseg, &s.v0)) {
          /* L-R downward cusp */
          trapezoids[trapezoids[t].d0].u0 = t;
          trapezoids[tn].d0 = trapezoids[tn].d1 = -1;
        } else {
          /* R-L downward cusp */
          trapezoids[trapezoids[tn].d0].u1 = tn;
          trapezoids[t].d0 = trapezoids[t].d1 = -1;
        }
      } else {
        if ((trapezoids[trapezoids[t].d0].u0 > 0) &&
            (trapezoids[trapezoids[t].d0].u1 > 0)) {
          if (trapezoids[trapezoids[t].d0].u0 == t) /* passes thru LHS */
          {
            trapezoids[trapezoids[t].d0].usave =
                trapezoids[trapezoids[t].d0].u1;
            trapezoids[trapezoids[t].d0].uside = S_LEFT;
          } else {
            trapezoids[trapezoids[t].d0].usave =
                trapezoids[trapezoids[t].d0].u0;
            trapezoids[trapezoids[t].d0].uside = S_RIGHT;
          }
        }
        trapezoids[trapezoids[t].d0].u0 = t;
        trapezoids[trapezoids[t].d0].u1 = tn;
      }

      t = trapezoids[t].d0;
    }

    else if ((trapezoids[t].d0 <= 0) &&
             (trapezoids[t].d1 > 0)) { /* Only one trapezoid below */
      assert(false);
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

      if (FP_EQUAL(trapezoids[t].lo.y, trapezoids[tlast].lo.y) &&
          FP_EQUAL(trapezoids[t].lo.x, trapezoids[tlast].lo.x) &&
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

      if (FP_EQUAL(trapezoids[t].lo.y, trapezoids[tlast].lo.y) &&
          FP_EQUAL(trapezoids[t].lo.x, trapezoids[tlast].lo.x) && tribot) {
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
  if (tlastr < 1 || tlastr > MAX_SEGMENTS * 10) {
    fprintf(stderr, "tlastr is undefined");
    exit(1);
  }
  merge_trapezoids(segnum, tfirstl, tlastl, S_LEFT);
  merge_trapezoids(segnum, tfirstr, tlastr, S_RIGHT);

  segments[segnum].is_inserted = TRUE;
  return 0;
}

/* Update the roots stored for each of the endpoints of the segment.
 * This is done to speed up the location-query for the endpoint when
 * the segment is inserted into the trapezoidation subsequently
 */
static int find_new_roots(int segnum) {
  Segment *s = &segments[segnum];

  if (s->is_inserted)
    return 0;

  s->root0 = locate_endpoint(&s->v0, &s->v1, s->root0);
  s->root0 = trapezoids[s->root0].sink;

  s->root1 = locate_endpoint(&s->v1, &s->v0, s->root1);
  s->root1 = trapezoids[s->root1].sink;
  return 0;
}

static int choose_idx;
static int permute[MAX_SEGMENTS];

/* Generate a random permutation of the segments 1..n */
static int generate_random_ordering(int n) {
  srand(3); // TODO: figure out where to seed prng
  for (int i = 0; i <= n; i++) {
    permute[i] = i;
  }
  for (int i = n; i > 1; i--) {
    const int j = (rand() % i) + 1;
    const int tmp = permute[j];
    permute[j] = permute[i];
    permute[i] = tmp;
  }
  choose_idx = n;
#ifdef DEBUG
  int sum = 0;
  for (S32 i = 1; i <= n; i++) {
    sum += permute[i];
  }
  assert(sum == (n * (n + 1)) / 2);
#endif
  return 0;
}

/* Return the next segment in the generated random ordering of all the */
/* segments in S */
static int choose_segment(void) {

#ifdef DEBUG
  fprintf(stderr, "choose_segment: %d\n", permute[choose_idx]);
#endif
  return permute[choose_idx--];
}

/* Get log*n for given n */
static int math_logstar_n(int n) {
  int i;
  F64 v;

  for (i = 0, v = (F64)n; v >= 1; i++)
    v = log2(v);

  return (i - 1);
}

static int math_N(int n, int h) {
  int i;
  F64 v;

  for (i = 0, v = n; i < h; i++)
    v = log2(v);

  return (int)ceil((F64)n / v);
}

/* Main routine to perform trapezoidation */
static int construct_trapezoids(int segment_count) {
  int i;
  int root, h;

  /* Add the first segment and get the query structure and trapezoid */
  /* list initialised */

  root = init_query_structure(choose_segment());

  for (i = 1; i <= segment_count; i++)
    segments[i].root0 = segments[i].root1 = root;

  for (h = 1; h <= math_logstar_n(segment_count); h++) {
    for (i = math_N(segment_count, h - 1) + 1; i <= math_N(segment_count, h); i++)
      add_segment(choose_segment());

    /* Find a new root for each of the segment endpoints */
    for (i = 1; i <= segment_count; i++)
      find_new_roots(i);
  }

  for (i = math_N(segment_count, math_logstar_n(segment_count)) + 1; i <= segment_count; i++)
    add_segment(choose_segment());

  return 0;
}

static int initialise(int n) {
  int i;

  for (i = 1; i <= n; i++)
    segments[i].is_inserted = FALSE;

  generate_random_ordering(n);

  return 0;
}

/* Input specified as contours.
 * Outer contour must be anti-clockwise.
 * All inner contours must be clockwise.
 *
 * Every contour is specified by giving all its points in order. No
 * point shoud be repeated. i.e. if the outer contour is a square,
 * only the four distinct endpoints shopudl be specified in order.
 *
 * ncontours: #contours
 * cntr: An array describing the number of points in each
 *	 contour. Thus, cntr[i] = #points in the i'th contour.
 * vertices: Input array of vertices. Vertices for each contour
 *           immediately follow those for previous one. Array location
 *           vertices[0] must NOT be used (i.e. i/p starts from
 *           vertices[1] instead. The output triangles are
 *	     specified  w.r.t. the indices of these vertices.
 * triangles: Output array to hold triangles.
 *
 * Enough space must be allocated for all the arrays before calling
 * this routine
 */

int triangulate_polygon(int num_contours, int verts_per_contour[], Coord2 *vertices,
                        TriangleArray *triangles) {
  int i;
  int nmonpoly, contour, npoints;
  int n;

  memset((void *)segments, 0, sizeof(segments));
  contour = 0;
  i = 1;

  while (contour < num_contours) {
    int j;
    int first, last;

    npoints = verts_per_contour[contour];
    first = i;
    last = first + npoints - 1;
    for (j = 0; j < npoints; j++, i++) {
      segments[i].v0.x = vertices[i].x;
      segments[i].v0.y = vertices[i].y;

      if (i == last) {
        segments[i].next = first;
        segments[i].prev = i - 1;
        segments[i - 1].v1 = segments[i].v0;
      } else if (i == first) {
        segments[i].next = i + 1;
        segments[i].prev = last;
        segments[last].v1 = segments[i].v0;
      } else {
        segments[i].prev = i - 1;
        segments[i].next = i + 1;
        segments[i - 1].v1 = segments[i].v0;
      }

      segments[i].is_inserted = FALSE;
    }

    contour++;
  }

  n = i - 1;

  initialise(n);
  construct_trapezoids(n);
  nmonpoly = monotonate_trapezoids(n);
  triangulate_monotone_polygons(n, nmonpoly, triangles);

  return 0;
}
