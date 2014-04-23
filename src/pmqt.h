#ifndef PMQT_H
#define PMQT_H

#include <stdint.h>
#include <math.h>

// Basic configuration of the library, these can be changed to support
// different floating point types.
typedef double pmqt_float_t;

#define PMQT_POS_INFINITY (INFINITY)
#define PMQT_NEG_INFINITY (-INFINITY)

#define PMQT_EPS (1e-9)

#define pmqt_fmin(_x, _y) fmin((_x), (_y))
#define pmqt_fmax(_x, _y) fmax((_x), (_y))
#define pmqt_fabs(_x)     fabs((_x))

typedef struct pmqt_point {
	pmqt_float_t  x;
	pmqt_float_t  y;
} pmqt_point_t;

typedef struct pmqt_edge {
	pmqt_point_t  a;
	pmqt_point_t  b;
} pmqt_edge_t;

// A simple edge linked-list element.
typedef struct pmqt_edge_list {
	const pmqt_edge_t     *edge;
	struct pmqt_edge_list *next;
} pmqt_edge_list_t;

// A bounding box with precomputed width/height.
typedef struct pmqt_bounds {
	pmqt_point_t  nw;
	pmqt_point_t  se;
	pmqt_float_t  width;
	pmqt_float_t  height;
} pmqt_bounds_t;

// The type of node in the tree.
// White: Empty node with no children.
// Grey: A node with children, has no data associated with it.
// Black (Point): Contains an edge endpoint, number of connected edges that
//                intersect with the node.
// Black (Edge): Contains a single edge that intersects with the node.
typedef enum {
	PMQT_WHITE = 0,
	PMQT_GREY,
	PMQT_BLACK_POINT,
	PMQT_BLACK_EDGE
} pmqt_nodetype_t;

typedef struct pmqt_node {
	// children
	struct pmqt_node *ne;
	struct pmqt_node *nw;
	struct pmqt_node *se;
	struct pmqt_node *sw;
	union {
		struct {
			pmqt_point_t      point;
			pmqt_edge_list_t *edges;
		} p;
		const pmqt_edge_t *edge;
	} u;
	pmqt_bounds_t    bounds;
	pmqt_nodetype_t  type;
} pmqt_node_t;

typedef struct pmqt {
	pmqt_node_t *root;
} pmqt_t;

#define PMQT_CONTINUE 0

// Callback used when searching a tree.
typedef int (*pmqt_search_cb)(const pmqt_node_t *node, const pmqt_edge_t *edge, void *arg);

// Callback used when walking a tree.
typedef int (*pmqt_walk_cb)(const pmqt_node_t *node, void *arg);

// Allocates a new PM Quadtree with the given bounds.
pmqt_t * pmqt_new(pmqt_float_t min_x, pmqt_float_t min_y,
	pmqt_float_t max_x, pmqt_float_t max_y);

// Deallocates and frees a tree.
void pmqt_free(pmqt_t *tree);

// Return codes from pmqt_insert.
typedef enum {
	PMQT_OK          =  0,  // No error
	PMQT_ERR_NOMEM   = -1,  // malloc failed
	PMQT_ERR_INTRSCT = -2,  // An edge that would intersect an existing edge
	                        // was attempted to be inserted
	PMQT_ERR_OOB     = -3,  // An edge that was outside the bounds of the
	                        // tree was attempted to be inserted
	PMQT_ERR_OTHER   = -100 // Unknown error
} pmqt_err_t;

// Insert an edge into the tree. Note: the edge must exist as long as the
// tree does.
pmqt_err_t pmqt_insert(pmqt_t *tree, const pmqt_edge_t *edge);

// Walk all the children of the tree.
int pmqt_walk(const pmqt_node_t *root, pmqt_walk_cb descent,
	pmqt_walk_cb ascent, void *arg);

// Search all nodes of the tree that intersect with the given edge.
int pmqt_search(const pmqt_node_t *node, const pmqt_edge_t *edge,
	pmqt_search_cb process, void *arg);

#endif // PMQT_H

