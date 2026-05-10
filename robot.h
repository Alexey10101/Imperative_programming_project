#ifndef ROBOT_H
#define ROBOT_H

#include <stddef.h>

typedef struct KDNode {
    double *point;
    struct KDNode *left;
    struct KDNode *right;
} KDNode;

typedef struct {
    double *values;
    size_t n;
    size_t dim;
} PointCloud;

int load_csv(const char *path, PointCloud *pc);
void free_cloud(PointCloud *pc);

KDNode *kd_build(const PointCloud *pc);
KDNode *kd_insert(KDNode *r, const double *p, size_t dim, size_t depth, int *ok);
KDNode *kd_delete(KDNode *r, const double *p, size_t dim, size_t depth, int *del);
void kd_nearest(KDNode *r, const double *q, size_t dim, size_t depth, const double **best, double *best_d2);
void kd_free(KDNode *r);

int parse_point(const char *arg, size_t dim, double *out);
void print_point(const double *p, size_t dim);

int dbscan(const PointCloud *pc, double eps, int minPts, int *labels, int *clusters);
int fuzzy_cmeans(const PointCloud *pc, int c, double m, int iters, double tol, int *labels, double *centroids);

#endif
