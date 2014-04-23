#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>
#include <string.h>
#include "pmqt.h"

static pmqt_err_t pmqt_insert_edge(pmqt_t *tree, pmqt_node_t *root,
	const pmqt_edge_t *edge);

static inline void
pmqt_point_set(pmqt_point_t *point, pmqt_float_t x, pmqt_float_t y)
{
	point->x = x;
	point->y = y;
}

static inline bool
pmqt_point_equals(const pmqt_point_t *a, const pmqt_point_t *b)
{
	return a->x == b->x && a->y == b->y;
}

static void
pmqt_bounds_extend(pmqt_bounds_t *bounds, pmqt_float_t x, pmqt_float_t y)
{
	bounds->nw.x = pmqt_fmin(x, bounds->nw.x);
	bounds->nw.y = pmqt_fmax(y, bounds->nw.y);
	bounds->se.x = pmqt_fmax(x, bounds->se.x);
	bounds->se.y = pmqt_fmin(y, bounds->se.y);

	bounds->width  = pmqt_fabs(bounds->nw.x - bounds->se.x);
	bounds->height = pmqt_fabs(bounds->nw.y - bounds->se.y);
}

static pmqt_node_t *
pmqt_node_new(void)
{
	pmqt_node_t *node;

	if ((node = malloc(sizeof(*node))) == NULL) {
		return NULL;
	}

	node->ne = NULL;
	node->nw = NULL;
	node->se = NULL;
	node->sw = NULL;

	node->type = PMQT_WHITE;

	pmqt_point_set(&node->bounds.nw, PMQT_POS_INFINITY, PMQT_NEG_INFINITY);
	pmqt_point_set(&node->bounds.se, PMQT_NEG_INFINITY, PMQT_POS_INFINITY);

	return node;
}

static void
pmqt_node_free(pmqt_node_t *node)
{
	assert(node);

	switch (node->type) {
	case PMQT_GREY:
		pmqt_node_free(node->nw);
		pmqt_node_free(node->ne);
		pmqt_node_free(node->sw);
		pmqt_node_free(node->se);
		break;
	case PMQT_BLACK_POINT:
	{
		pmqt_edge_list_t *edges = node->u.p.edges;

		while (edges) {
			pmqt_edge_list_t *next = edges->next;
			free(edges);
			edges = next;
		}
	}
	default:
		break;
	}

	free(node);
}

static pmqt_node_t *
pmqt_node_new_with_bounds(pmqt_float_t min_x, pmqt_float_t min_y,
	pmqt_float_t max_x, pmqt_float_t max_y)
{
	pmqt_node_t *node;

	if ((node = pmqt_node_new()) == NULL) {
		return NULL;
	}

	pmqt_bounds_extend(&node->bounds, max_x, max_y);
	pmqt_bounds_extend(&node->bounds, min_x, min_y);

	return node;
}

static pmqt_err_t
pmqt_node_add_edge(pmqt_node_t *node, const pmqt_edge_t *edge)
{
	assert(node);
	assert(edge);

	pmqt_edge_list_t *new_el;

	if ((new_el = malloc(sizeof(*new_el))) == NULL) {
		return PMQT_ERR_NOMEM;
	}

	new_el->next = NULL;
	new_el->edge = edge;

	if (node->u.p.edges == NULL) {
		node->u.p.edges = new_el;
		return PMQT_OK;
	}

	pmqt_edge_list_t *el = node->u.p.edges;

	while (el->next) {
		el = el->next;
	}

	el->next = new_el;

	return PMQT_OK;
}

static bool
pmqt_edge_intersect_bounds(const pmqt_edge_t *edge, const pmqt_bounds_t *bounds)
{
	// find min and max X for the segment
	pmqt_float_t minX = edge->a.x;
	pmqt_float_t maxX = edge->b.x;

	if (edge->a.x > edge->b.x) {
		minX = edge->b.x;
		maxX = edge->a.x;
	}

	// find the intersection of the segment's and rectangle's x-projections
	if (maxX > bounds->se.x) { maxX = bounds->se.x; }
	if (minX < bounds->nw.x) { minX = bounds->nw.x; }

	if (minX > maxX) {
		// x-projections do not intersect
		return false;
	}

	// find corresponding min and max y for min and max x we found before
	pmqt_float_t minY = edge->a.y;
	pmqt_float_t maxY = edge->b.y;

	pmqt_float_t dx = edge->b.x - edge->a.x;

	if (pmqt_fabs(dx) > PMQT_EPS) {
		pmqt_float_t a = (edge->b.y - edge->a.y) / dx;
		pmqt_float_t b = edge->a.y - a * edge->a.x;
		minY = a * minX + b;
		maxY = a * maxX + b;
	}

	if (minY > maxY) {
		pmqt_float_t tmp = maxY;
		maxY = minY;
		minY = tmp;
	}

	// find the intersection of the segment's and rectangle's y-projections
	if (maxY > bounds->nw.y) { maxY = bounds->nw.y; }
	if (minY < bounds->se.y) { minY = bounds->se.y; }

	if (minY > maxY) {
		// y-projections do not intersect
		return false;
	}

	return true;
}


static bool
pmqt_point_in_bounds(const pmqt_point_t *point, const pmqt_bounds_t *bounds)
{
	return bounds->nw.x <= point->x &&
	       bounds->nw.y >= point->y &&
	       bounds->se.x >= point->x &&
	       bounds->se.y <= point->y;
}

static pmqt_err_t
pmqt_split_node(pmqt_t *tree, pmqt_node_t *node)
{
	if (node->type == PMQT_GREY) {
		// you can never split a grey node, this is a logic error
		return PMQT_ERR_OTHER;
	}

	pmqt_node_t *nw = NULL;
	pmqt_node_t *ne = NULL;
	pmqt_node_t *sw = NULL;
	pmqt_node_t *se = NULL;

	pmqt_float_t x  = node->bounds.nw.x;
	pmqt_float_t y  = node->bounds.nw.y;
	pmqt_float_t hw = node->bounds.width / 2;
	pmqt_float_t hh = node->bounds.height / 2;

	if ((nw = pmqt_node_new_with_bounds(x,      y - hh,     x + hw,     y)) == NULL) { goto nomem; }
	if ((ne = pmqt_node_new_with_bounds(x + hw, y - hh,     x + hw * 2, y)) == NULL) { goto nomem; }
	if ((sw = pmqt_node_new_with_bounds(x,      y - hh * 2, x + hw,     y - hh)) == NULL) { goto nomem; }
	if ((se = pmqt_node_new_with_bounds(x + hw, y - hh * 2, x + hw * 2, y - hh)) == NULL) { goto nomem; }

	node->nw = nw;
	node->ne = ne;
	node->sw = sw;
	node->se = se;

	pmqt_nodetype_t old_type = node->type;
	node->type = PMQT_GREY; // the node always becomes a grey node

	switch (old_type) {
	case PMQT_BLACK_EDGE:
		// was black (edge), so push it down to the children
		return pmqt_insert_edge(tree, node, (const pmqt_edge_t *)node->u.edge);
	case PMQT_BLACK_POINT:
	{
		// was black (point), that means that one child will become a
		// black (point) node, and up to three will become black (edge)
		// nodes
		pmqt_edge_list_t *edges = node->u.p.edges;
		pmqt_err_t ret = PMQT_OK;

		while (edges) {
			pmqt_edge_list_t *next = edges->next;
			ret = pmqt_insert_edge(tree, node, edges->edge);
			free(edges);
			if (ret) { break; }
			edges = next;
		}

		return ret;
	}
	case PMQT_WHITE:
		// was a white node, nothing special happens
		return PMQT_OK;
	default:
		// a fatal error
		return PMQT_ERR_OTHER;
	}

nomem:
	if (nw) { free(nw); }
	if (ne) { free(ne); }
	if (sw) { free(sw); }
	if (se) { free(se); }

	return PMQT_ERR_NOMEM;
}

static pmqt_err_t
pmqt_insert_edge(pmqt_t *tree, pmqt_node_t *node, const pmqt_edge_t *edge)
{
	switch (node->type) {
	case PMQT_WHITE:
	{
		bool has_a, has_b;
		has_a = pmqt_point_in_bounds(&edge->a, &node->bounds);
		has_b = pmqt_point_in_bounds(&edge->b, &node->bounds);

		if (has_a && has_b) {
			// both points are inside the area, so it needs to be split
			pmqt_split_node(tree, node);
			return pmqt_insert_edge(tree, node, edge);
		} else if (has_a) {
			node->type = PMQT_BLACK_POINT;
			node->u.p.point = edge->a;
			node->u.p.edges = NULL;
			return pmqt_node_add_edge(node, edge);
		} else if (has_b) {
			node->type = PMQT_BLACK_POINT;
			node->u.p.point = edge->b;
			node->u.p.edges = NULL;
			return pmqt_node_add_edge(node, edge);
		} else if (pmqt_edge_intersect_bounds(edge, &node->bounds)) {
			node->type = PMQT_BLACK_EDGE;
			node->u.edge = edge;
		}

		return PMQT_OK;
	}
	case PMQT_GREY:
	{
		int ret;

		if ((ret = pmqt_insert_edge(tree, node->nw, edge))) { return ret; }
		if ((ret = pmqt_insert_edge(tree, node->ne, edge))) { return ret; }
		if ((ret = pmqt_insert_edge(tree, node->sw, edge))) { return ret; }
		if ((ret = pmqt_insert_edge(tree, node->se, edge))) { return ret; }

		break;
	}
	case PMQT_BLACK_POINT:
		if (pmqt_point_equals(&node->u.p.point, &edge->a) ||
		    pmqt_point_equals(&node->u.p.point, &edge->b)) {
			// when the node is black (point), and the edge shares a point,
			// append it to the list of edges and stop.
			pmqt_node_add_edge(node, edge);
			break;
		}

		// otherwise, intentionally fall through here and treat it
		// like a black (edge) case.
	case PMQT_BLACK_EDGE:
	{
		bool has_a, has_b;
		has_a = pmqt_point_in_bounds(&edge->a, &node->bounds);
		has_b = pmqt_point_in_bounds(&edge->b, &node->bounds);

		if (has_a || has_b || pmqt_edge_intersect_bounds(edge, &node->bounds)) {
			if (node->bounds.width == 0 || node->bounds.height == 0) {
				// TODO: Check the intersection BEFORE node split
				return PMQT_ERR_INTRSCT;
			}
			pmqt_split_node(tree, node);
			return pmqt_insert_edge(tree, node, edge);
		}

		break;
	}
	};

	return PMQT_OK;
}

pmqt_t *
pmqt_new(pmqt_float_t min_x, pmqt_float_t min_y, pmqt_float_t max_x,
	pmqt_float_t max_y)
{
	pmqt_t *tree;

	if ((tree = malloc(sizeof(*tree))) == NULL) {
		return NULL;
	}

	tree->root = pmqt_node_new_with_bounds(min_x, min_y, max_x, max_y);

	if (tree->root == NULL) {
		free(tree);
		return NULL;
	}

	return tree;
}

void
pmqt_free(pmqt_t *tree)
{
	if (tree->root) {
		pmqt_node_free(tree->root);
	}

	free(tree);
}

pmqt_err_t
pmqt_insert(pmqt_t *tree, const pmqt_edge_t *edge)
{
	if (!pmqt_point_in_bounds(&edge->a, &tree->root->bounds) ||
	    !pmqt_point_in_bounds(&edge->b, &tree->root->bounds)) {
		return PMQT_ERR_OOB;
	}

	return pmqt_insert_edge(tree, tree->root, edge);
}

int
pmqt_walk(const pmqt_node_t *root, pmqt_walk_cb descent, pmqt_walk_cb ascent,
	void *arg)
{
	int ret = PMQT_CONTINUE;

	ret = (*descent)(root, arg);
	if (ret) { return ret; }

	if (root->type == PMQT_GREY) {
		ret = pmqt_walk(root->nw, descent, ascent, arg);
		if (ret) { return ret; }
		ret = pmqt_walk(root->ne, descent, ascent, arg);
		if (ret) { return ret; }
		ret = pmqt_walk(root->sw, descent, ascent, arg);
		if (ret) { return ret; }
		ret = pmqt_walk(root->se, descent, ascent, arg);
		if (ret) { return ret; }
	}

	ret = (*ascent)(root, arg);
	if (ret) { return ret; }

	return PMQT_CONTINUE;
}

int
pmqt_search(const pmqt_node_t *node, const pmqt_edge_t *edge,
	pmqt_search_cb process, void *arg)
{
	switch (node->type) {
	case PMQT_WHITE:
		return PMQT_CONTINUE;
	case PMQT_GREY:
	{
		if (pmqt_edge_intersect_bounds(edge, &node->nw->bounds)) {
			int ret = pmqt_search(node->nw, edge, process, arg);
			if (ret) { return ret; }
		}

		if (pmqt_edge_intersect_bounds(edge, &node->ne->bounds)) {
			int ret = pmqt_search(node->ne, edge, process, arg);
			if (ret) { return ret; }
		}

		if (pmqt_edge_intersect_bounds(edge, &node->sw->bounds)) {
			int ret = pmqt_search(node->sw, edge, process, arg);
			if (ret) { return ret; }
		}

		if (pmqt_edge_intersect_bounds(edge, &node->se->bounds)) {
			int ret = pmqt_search(node->se, edge, process, arg);
			if (ret) { return ret; }
		}

		return PMQT_CONTINUE;
	}
	case PMQT_BLACK_EDGE: // intentional fall through
	case PMQT_BLACK_POINT:
		if (process) {
			return (*process)(node, edge, arg);
		}
		break;
	}

	return PMQT_CONTINUE;
}

