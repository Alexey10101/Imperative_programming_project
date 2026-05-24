#include "robot.h"

#include <ctype.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define EPS 1e-9
#define P(pc, i) ((pc)->values + (i) * (pc)->dim)

typedef struct {
    int *a;
    size_t n;
    size_t cap;
} IntVec;

typedef struct KDRefNode {
    const double *point;
    int idx;
    struct KDRefNode *left;
    struct KDRefNode *right;
} KDRefNode;

static char *xstrdup(const char *s)
{
    size_t n = strlen(s) + 1;
    char *d = (char *)malloc(n);
    if (d) memcpy(d, s, n);
    return d;
}

static void trim(char *s)
{
    size_t i = 0, j = strlen(s);
    while (j && isspace((unsigned char)s[j - 1]))
        s[--j] = '\0';
    while (s[i] && isspace((unsigned char)s[i]))
        i++;
    if (i) memmove(s, s + i, j - i + 1);
}

static int parse_num(const char *t, double *v)
{
    char *e = NULL;
    *v = strtod(t, &e);
    while (e && isspace((unsigned char)*e)) e++;
    return e && e != t && *e == '\0';
}

static int vec_push(IntVec *v, int x)
{
    if (v->n == v->cap)
    {
        size_t nc = v->cap ? v->cap * 2 : 64;
        int *na = (int *)realloc(v->a, nc * sizeof(int));
        if (!na) return 0;
        v->a = na;
        v->cap = nc;
    }
    v->a[v->n++] = x;
    return 1;
}

static int parse_list(const char *line, double **vals, size_t *cnt)
{
    char *buf = xstrdup(line), *cur = NULL;
    size_t n = 0, cap = 8;
    double *a = (double *)malloc(cap * sizeof(double));

    if (!buf || !a)
    {
        free(buf);
        free(a);
        return 0;
    }

    cur = buf;
    while (1)
    {
        char *comma = strchr(cur, ',');
        double x = 0.0;

        if (comma) *comma = '\0';
        trim(cur);
        if (!cur[0] || !parse_num(cur, &x))
        {
            free(buf);
            free(a);
            return 0;
        }

        if (n == cap)
        {
            cap *= 2;
            a = (double *)realloc(a, cap * sizeof(double));
            if (!a)
            {
                free(buf);
                return 0;
            }
        }
        a[n++] = x;
        if (!comma) break;
        cur = comma + 1;
    }

    free(buf);
    *vals = a;
    *cnt = n;
    return n > 0;
}

int load_csv(const char *path, PointCloud *pc)
{
    FILE *f = fopen(path, "r");
    char line[8192];
    double *store = NULL;
    size_t cap = 0;
    size_t n = 0;
    size_t dim = 0;
    if (!f) return 0;

    while (fgets(line, sizeof(line), f))
    {
        double *row = NULL;
        size_t k = 0;

        trim(line);
        if (!line[0]) continue;

        if (!parse_list(line, &row, &k))
        {
            fclose(f);
            free(store);
            return 0;
        }

        if (!dim) dim = k;
        
        if (k != dim)
        {
            fclose(f);
            free(row);
            free(store);
            return 0;
        }

        if (n == cap)
        {
            cap = cap ? cap * 2 : 64;
            store = (double *)realloc(store, cap * dim * sizeof(double));
            if (!store)
            {
                fclose(f);
                free(row);
                return 0;
            }
        }

        memcpy(store + n * dim, row, dim * sizeof(double));
        n++;
        free(row);
    }

    fclose(f);

    if (!n || !dim)
    {
        free(store);
        return 0;
    }

    pc->values = store;
    pc->n = n;
    pc->dim = dim;
    return 1;
}

void free_cloud(PointCloud *pc)
{
    free(pc->values);
    pc->values = NULL;
    pc->n = 0;
    pc->dim = 0;
}

static double d2(const double *a, const double *b, size_t dim)
{
    double s = 0.0;
    for (size_t i = 0; i < dim; i++)
    {
        double t = a[i] - b[i];
        s += t * t;
    }
    return s;
}

static int peq(const double *a, const double *b, size_t dim)
{
    for (size_t i = 0; i < dim; i++)
        if (fabs(a[i] - b[i]) > EPS) return 0;
    return 1;
}

static KDNode *node_new(const double *p, size_t dim)
{
    KDNode *n = (KDNode *)calloc(1, sizeof(KDNode));
    if (!n) return NULL;

    n->point = (double *)malloc(dim * sizeof(double));
    if (!n->point)
    {
        free(n);
        return NULL;
    }

    memcpy(n->point, p, dim * sizeof(double));
    return n;
}

KDNode *kd_insert(KDNode *r, const double *p, size_t dim, size_t depth, int *ok)
{
    size_t cd = depth % dim;

    if (!r)
    {
        KDNode *n = node_new(p, dim);
        if (!n) *ok = 0;
        return n;
    }

    if (p[cd] < r->point[cd]) r->left = kd_insert(r->left, p, dim, depth + 1, ok);
    else r->right = kd_insert(r->right, p, dim, depth + 1, ok);
    return r;
}

static KDNode *kd_min(KDNode *r, size_t td, size_t dim, size_t depth)
{
    KDNode *a = NULL;
    KDNode *b = NULL;
    KDNode *m = r;
    size_t cd = depth % dim;

    if (!r) return NULL;
    if (cd == td) return r->left ? kd_min(r->left, td, dim, depth + 1) : r;

    a = kd_min(r->left, td, dim, depth + 1);
    b = kd_min(r->right, td, dim, depth + 1);
    if (a && a->point[td] < m->point[td]) m = a;
    if (b && b->point[td] < m->point[td]) m = b;
    return m;
}

KDNode *kd_delete(KDNode *r, const double *p, size_t dim, size_t depth, int *del)
{
    size_t cd = depth % dim;
    if (!r) return NULL;
    if (peq(r->point, p, dim))
    {
        *del = 1;
        if (r->right)
        {
            KDNode *m = kd_min(r->right, cd, dim, depth + 1);
            memcpy(r->point, m->point, dim * sizeof(double));
            r->right = kd_delete(r->right, m->point, dim, depth + 1, del);
            return r;
        }
        if (r->left)
        {
            KDNode *m = kd_min(r->left, cd, dim, depth + 1);
            memcpy(r->point, m->point, dim * sizeof(double));
            r->right = kd_delete(r->left, m->point, dim, depth + 1, del);
            r->left = NULL;
            return r;
        }
        free(r->point);
        free(r);
        return NULL;
    }
    if (p[cd] < r->point[cd]) r->left = kd_delete(r->left, p, dim, depth + 1, del);
    else r->right = kd_delete(r->right, p, dim, depth + 1, del);
    return r;
}

void kd_nearest(KDNode *r, const double *q, size_t dim, size_t depth, const double **best, double *best_d2)
{
    size_t cd = 0;
    KDNode *near = NULL;
    KDNode *far = NULL;
    double dd = 0.0, ad = 0.0;
    if (!r) return;
    dd = d2(r->point, q, dim);
    if (dd < *best_d2)
    {
        *best_d2 = dd;
        *best = r->point;
    }

    cd = depth % dim;
    near = (q[cd] < r->point[cd]) ? r->left : r->right;
    far = (q[cd] < r->point[cd]) ? r->right : r->left;

    kd_nearest(near, q, dim, depth + 1, best, best_d2);
    ad = q[cd] - r->point[cd];
    if (ad * ad < *best_d2) kd_nearest(far, q, dim, depth + 1, best, best_d2);
}

void kd_free(KDNode *r)
{
    if (!r) return;
    kd_free(r->left);
    kd_free(r->right);
    free(r->point);
    free(r);
}

KDNode *kd_build(const PointCloud *pc)
{
    KDNode *r = NULL;
    int ok = 1;

    for (size_t i = 0; i < pc->n; i++)
    {
        r = kd_insert(r, P(pc, i), pc->dim, 0, &ok);
        if (!ok)
        {
            kd_free(r);
            return NULL;
        }
    }
    return r;
}

static KDRefNode *kdref_new(const double *p, int idx)
{
    KDRefNode *n = (KDRefNode *)calloc(1, sizeof(KDRefNode));
    if (!n) return NULL;
    n->point = p;
    n->idx = idx;
    return n;
}

static KDRefNode *kdref_insert(KDRefNode *r, const double *p, int idx, size_t dim, size_t depth, int *ok)
{
    size_t cd = depth % dim;
    if (!r)
    {
        KDRefNode *n = kdref_new(p, idx);
        if (!n) *ok = 0;
        return n;
    }

    if (p[cd] < r->point[cd]) r->left = kdref_insert(r->left, p, idx, dim, depth + 1, ok);
    else r->right = kdref_insert(r->right, p, idx, dim, depth + 1, ok);
    return r;
}

static KDRefNode *kdref_build(const PointCloud *pc)
{
    KDRefNode *r = NULL;
    int ok = 1;
    for (size_t i = 0; i < pc->n; i++)
    {
        r = kdref_insert(r, P(pc, i), (int)i, pc->dim, 0, &ok);
        if (!ok) return r;
    }
    return r;
}

static void kdref_free(KDRefNode *r)
{
    if (!r) return;
    kdref_free(r->left);
    kdref_free(r->right);
    free(r);
}

static int kdref_radius(const KDRefNode *r, const double *q, double eps2, size_t dim, size_t depth, IntVec *out)
{
    size_t cd = depth % dim;
    double ad = 0.0;
    if (!r) return 1;
    if (d2(r->point, q, dim) <= eps2 + EPS)
    {
        if (!vec_push(out, r->idx)) return 0;
    }

    ad = q[cd] - r->point[cd];
    if (ad <= 0.0)
    {
        if (!kdref_radius(r->left, q, eps2, dim, depth + 1, out)) return 0;
        if (ad * ad <= eps2 && !kdref_radius(r->right, q, eps2, dim, depth + 1, out)) return 0;
    }
    else
    {
        if (!kdref_radius(r->right, q, eps2, dim, depth + 1, out)) return 0;
        if (ad * ad <= eps2 && !kdref_radius(r->left, q, eps2, dim, depth + 1, out)) return 0;
    }
    return 1;
}

int parse_point(const char *arg, size_t dim, double *out)
{
    double *vals = NULL;
    size_t cnt = 0;
    if (!parse_list(arg, &vals, &cnt)) return 0;
    if (cnt != dim)
    {
        free(vals);
        return 0;
    }
    memcpy(out, vals, dim * sizeof(double));
    free(vals);
    return 1;
}

void print_point(const double *p, size_t dim)
{
    for (size_t i = 0; i < dim; i++) printf("%s%.6f", i ? "," : "", p[i]);
}

int dbscan(const PointCloud *pc, double eps, int minPts, int *labels, int *clusters)
{
    size_t n = pc->n;
    double eps2 = eps * eps;
    int cid = 0;
    KDRefNode *tree = NULL;
    IntVec nbr = {0}, nbr2 = {0};
    int *queue = NULL, *inq_stamp = NULL;
    unsigned char *vis = NULL;
    int stamp = 1;
    int ok = 1;

    if (eps <= 0.0 || minPts <= 0) return 0;

    tree = kdref_build(pc);
    queue = (int *)malloc(n * sizeof(int));
    inq_stamp = (int *)calloc(n, sizeof(int));
    vis = (unsigned char *)calloc(n, 1);
    if (!tree || !queue || !inq_stamp || !vis)
    {
        kdref_free(tree);
        free(queue);
        free(inq_stamp);
        free(vis);
        free(nbr.a);
        free(nbr2.a);
        return 0;
    }

    for (size_t i = 0; i < n; i++) labels[i] = -2;

    for (size_t i = 0; i < n; i++)
    {
        size_t qh = 0;
        size_t qt = 0;
        if (vis[i]) continue;
        vis[i] = 1;

        nbr.n = 0;
        if (!kdref_radius(tree, P(pc, i), eps2, pc->dim, 0, &nbr))
        {
            ok = 0;
            break;
        }

        if ((int)nbr.n < minPts)
        {
            labels[i] = -1;
            continue;
        }

        if (stamp == 0x7fffffff)
        {
            memset(inq_stamp, 0, n * sizeof(int));
            stamp = 1;
        }
        else
            stamp++;

        labels[i] = cid;
        for (size_t k = 0; k < nbr.n; k++)
        {
            int v = nbr.a[k];
            if (inq_stamp[v] != stamp)
            {
                inq_stamp[v] = stamp;
                queue[qt++] = v;
            }
        }

        while (qh < qt)
        {
            int p = queue[qh++];
            if (!vis[p])
            {
                vis[p] = 1;
                nbr2.n = 0;
                if (!kdref_radius(tree, P(pc, (size_t)p), eps2, pc->dim, 0, &nbr2))
                {
                    ok = 0;
                    break;
                }
                if ((int)nbr2.n >= minPts)
                {
                    for (size_t t = 0; t < nbr2.n; t++)
                    {
                        int u = nbr2.a[t];
                        if (inq_stamp[u] != stamp)
                        {
                            inq_stamp[u] = stamp;
                            queue[qt++] = u;
                        }
                    }
                }
            }
            if (labels[p] < 0) labels[p] = cid;
        }
        cid++;
    }
    *clusters = cid;
    kdref_free(tree);
    free(queue);
    free(inq_stamp);
    free(vis);
    free(nbr.a);
    free(nbr2.a);
    return ok;
}

int fuzzy_cmeans(const PointCloud *pc, int c, double m, int iters, double tol, int *labels, double *cent)
{
    size_t n = pc->n;
    size_t d = pc->dim;
    size_t C = (size_t)c;
    double *u = NULL;
    double maxc = 0.0;

    if (c <= 0 || C > n || m <= 1.0) return 0;

    u = (double *)malloc(n * C * sizeof(double));
    if (!u) return 0;

    srand(42);
    for (size_t i = 0; i < n; i++)
    {
        double s = 0.0;
        for (size_t j = 0; j < C; j++)
        {
            u[i * C + j] = (double)(rand() % 1000 + 1);
            s += u[i * C + j];
        }
        for (size_t j = 0; j < C; j++)
            u[i * C + j] /= s;
    }

    for (int it = 0; it < iters; it++)
    {
        maxc = 0.0;
        for (size_t j = 0; j < C; j++)
        {
            double den = 0.0;
            for (size_t t = 0; t < d; t++)
                cent[j * d + t] = 0.0;

            for (size_t i = 0; i < n; i++)
            {
                double w = pow(u[i * C + j], m);
                den += w;
                for (size_t t = 0; t < d; t++)
                    cent[j * d + t] += w * P(pc, i)[t];
            }

            if (den < EPS)
            {
                free(u);
                return 0;
            }
            for (size_t t = 0; t < d; t++)
                cent[j * d + t] /= den;
        }

        for (size_t i = 0; i < n; i++)
        {
            int z = -1;
            for (size_t j = 0; j < C; j++)
            {
                if (d2(P(pc, i), cent + j * d, d) < EPS)
                {
                    z = (int)j;
                    break;
                }
            }
            for (size_t j = 0; j < C; j++)
            {
                double nu = 0.0;
                if (z >= 0)
                    nu = ((int)j == z) ? 1.0 : 0.0;
                else
                {
                    double dij = sqrt(d2(P(pc, i), cent + j * d, d));
                    double s = 0.0;
                    for (size_t k = 0; k < C; k++)
                    {
                        double dik = sqrt(d2(P(pc, i), cent + k * d, d));
                        s += pow(dij / dik, 2.0 / (m - 1.0));
                    }
                    nu = 1.0 / s;
                }

                if (fabs(nu - u[i * C + j]) > maxc)
                    maxc = fabs(nu - u[i * C + j]);
                u[i * C + j] = nu;
            }
        }
        if (maxc < tol) break;
    }

    for (size_t i = 0; i < n; i++)
    {
        size_t bj = 0;
        for (size_t j = 1; j < C; j++)
        {
            if (u[i * C + j] > u[i * C + bj]) bj = j;
        }
        labels[i] = (int)bj;
    }

    free(u);
    return 1;
}

static double cmeans_membership(const double *x, const double *cent, size_t dim, int c, int j, double m)
{
    double dij2 = d2(x, cent + (size_t)j * dim, dim);
    if (dij2 < EPS) return 1.0;

    double dij = sqrt(dij2);
    double sum = 0.0;
    for (int k = 0; k < c; k++)
    {
        double dik2 = d2(x, cent + (size_t)k * dim, dim);
        if (dik2 < EPS) return 0.0;
        sum += pow(dij / sqrt(dik2), 2.0 / (m - 1.0));
    }
    return 1.0 / sum;
}

static int parse_dbscan_arg(const char *s, double *eps, int *minPts)
{
    char tail = '\0';
    if (!strchr(s, ',')) return 0;
    if (sscanf(s, " %lf , %d %c", eps, minPts, &tail) != 2) return 0;
    return *eps > 0.0 && *minPts > 0;
}

static void usage(const char *p)
{
    fprintf(stderr, "Usage:\n");
    fprintf(stderr, "  %s <csv> -kd_insert  x1,x2[,x3,...,xk]\n", p);
    fprintf(stderr, "  %s <csv> -kd_delete  x1,x2[,x3,...,xk]\n", p);
    fprintf(stderr, "  %s <csv> -kd_nearest x1,x2[,x3,...,xk]\n", p);
    fprintf(stderr, "  %s <csv> -cmeans <clusters> [m] [max_iter] [tol]\n", p);
    fprintf(stderr, "  %s <csv> -dbscan <eps,minPts>  OR  <eps> <minPts>\n", p);
}

int main(int argc, char **argv)
{
    PointCloud pc = {0};

    if (argc < 4 || !load_csv(argv[1], &pc))
    {
        usage(argv[0]);
        return 1;
    }

    if (!strcmp(argv[2], "-kd_insert") || !strcmp(argv[2], "-kd_delete") || !strcmp(argv[2], "-kd_nearest"))
    {
        KDNode *root = kd_build(&pc);
        double *q = (double *)malloc(pc.dim * sizeof(double));

        if (!root || !q || !parse_point(argv[3], pc.dim, q))
        {
            kd_free(root);
            free(q);
            free_cloud(&pc);
            return 1;
        }

        if (!strcmp(argv[2], "-kd_insert"))
        {
            int ok = 1;
            root = kd_insert(root, q, pc.dim, 0, &ok);
            if (ok)
            {
                printf("Inserted point: ");
                print_point(q, pc.dim);
                puts("");
            }
        }
        else if (!strcmp(argv[2], "-kd_delete"))
        {
            int del = 0;
            root = kd_delete(root, q, pc.dim, 0, &del);
            printf("%s point: ", del ? "Deleted" : "Point not found");
            print_point(q, pc.dim);
            puts("");
        }
        else
        {
            const double *best = NULL;
            double best_d2 = DBL_MAX;
            kd_nearest(root, q, pc.dim, 0, &best, &best_d2);
            printf("Query point: ");
            print_point(q, pc.dim);
            printf("\nNearest point: ");
            print_point(best, pc.dim);
            printf("\nDistance: %.6f\n", sqrt(best_d2));
        }
        kd_free(root);
        free(q);
    }
    else if (!strcmp(argv[2], "-cmeans"))
    {
        int c = atoi(argv[3]);
        int it = (argc > 5) ? atoi(argv[5]) : 100;
        double m = (argc > 4) ? atof(argv[4]) : 2.0;
        double tol = (argc > 6) ? atof(argv[6]) : 1e-4;
        int *labels = (int *)malloc(pc.n * sizeof(int));
        double *cent = (double *)malloc((size_t)c * pc.dim * sizeof(double));

        if (!labels || !cent || !fuzzy_cmeans(&pc, c, m, it, tol, labels, cent))
        {
            free(labels);
            free(cent);
            free_cloud(&pc);
            return 1;
        }

        puts("C-means completed.\nCentroids:");
        for (int j = 0; j < c; j++)
        {
            printf("  C%d: ", j);
            print_point(cent + (size_t)j * pc.dim, pc.dim);
            puts("");
        }

        puts("Point assignments:");
        for (size_t i = 0; i < pc.n; i++)
            printf("  Point %zu -> C%d\n", i, labels[i]);

        puts("Membership degrees:");
        for (size_t i = 0; i < pc.n; i++)
        {
            int z = -1;
            printf("  Point %zu: ", i);
            for (int j = 0; j < c; j++)
            {
                if (d2(P(&pc, i), cent + (size_t)j * pc.dim, pc.dim) < EPS)
                {
                    z = j;
                    break;
                }
            }

            for (int j = 0; j < c; j++)
            {
                double u = 0.0;
                if (z >= 0) u = (j == z) ? 1.0 : 0.0;
                else u = cmeans_membership(P(&pc, i), cent, pc.dim, c, j, m);
                printf("C%d=%.6f%s", j, u, (j + 1 == c) ? "" : ", ");
            }
            puts("");
        }

        free(labels);
        free(cent);
    }
    else if (!strcmp(argv[2], "-dbscan"))
    {
        double eps = 0.0;
        int minPts = 0, k = 0;
        int *labels = (int *)malloc(pc.n * sizeof(int));

        if (!labels)
        {
            free_cloud(&pc);
            return 1;
        }

        if (!(parse_dbscan_arg(argv[3], &eps, &minPts) ||
              (argc > 4 && (eps = atof(argv[3])) > 0.0 && (minPts = atoi(argv[4])) > 0)))
        {
            free(labels);
            free_cloud(&pc);
            return 1;
        }

        if (!dbscan(&pc, eps, minPts, labels, &k))
        {
            free(labels);
            free_cloud(&pc);
            return 1;
        }

        printf("DBSCAN completed.\nClusters found: %d\nLabels (-1 means noise):\n", k);
        for (size_t i = 0; i < pc.n; i++)
            printf("  Point %zu -> %d\n", i, labels[i]);
        free(labels);
    }
    else
    {
        usage(argv[0]);
        free_cloud(&pc);
        return 1;
    }
    free_cloud(&pc);
    return 0;
}
