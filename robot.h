#ifndef ROBOT_H
#define ROBOT_H

#include <stddef.h>

// узел k-d дерева; point хранит dim координат
typedef struct KDNode {
    double *point;
    struct KDNode *left;
    struct KDNode *right;
} KDNode;

// облако точек: плоский массив values[n * dim] (row-major)
typedef struct {
    double *values;
    size_t n;
    size_t dim;
} PointCloud;

// загрузить точки из CSV в PointCloud
int load_csv(const char *path, PointCloud *pc);
// освободить память PointCloud
void free_cloud(PointCloud *pc);

// построить k-d дерево из всех точек облака
KDNode *kd_build(const PointCloud *pc);
// вставить одну точку в k-d дерево
KDNode *kd_insert(KDNode *r, const double *p, size_t dim, size_t depth, int *ok);
// удалить одну точку из k-d дерева
KDNode *kd_delete(KDNode *r, const double *p, size_t dim, size_t depth, int *del);
// найти ближайшую к запросу точку в k-d дереве
void kd_nearest(KDNode *r, const double *q, size_t dim, size_t depth, const double **best, double *best_d2);
// освободить всё k-d дерево
void kd_free(KDNode *r);

// разобрать строку точки из CLI в числовые координаты
int parse_point(const char *arg, size_t dim, double *out);
// напечатать точку в формате CSV
void print_point(const double *p, size_t dim);

// кластеризация по плотности; возвращает метки и число кластеров
int dbscan(const PointCloud *pc, double eps, int minPts, int *labels, int *clusters);
// fuzzy C-means; возвращает hard-метки и центроиды
int fuzzy_cmeans(const PointCloud *pc, int c, double m, int iters, double tol, int *labels, double *centroids);

#endif
