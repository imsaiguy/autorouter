/* Visible Autorouter - compiled A* search (the router's inner loop).
 *
 * A drop-in for Router._astar in router.py, for nets without a wider net class:
 * same states (cell * 10 + arrival direction), same costs, same rules.  Plain C with
 * no Python headers, loaded with ctypes, so one library per operating system works
 * with every Python version.
 *
 * Build:  gcc -O2 -shared -fPIC -o libastar_linux_x86_64.so astar.c
 *         x86_64-w64-mingw32-gcc -O2 -shared -o astar_win64.dll astar.c
 */
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif

#define FREE (-1)
#define NSTATE 10
#define VIA_DIR 8
#define START_DIR 9

static const int DX[8] = {1, -1, 0, 0, 1, -1, 1, -1};
static const int DY[8] = {0, 0, 1, -1, 1, 1, -1, -1};
static const double SQ2 = 1.4142135623730951;
static double DLEN[8];
static unsigned char TURN_OK[NSTATE][8], TURN_OK90[NSTATE][8];
static int REV[8];

typedef struct { double f, g; int st; } HeapItem;

typedef struct {
    int nx, ny, nl, plane, size;
    const int *owner, *via_owner, *occ;
    const double *hist;
    double dir_mult[64 * 8];
    int nfp, *fp_dx, *fp_dy;
    /* per-cell scratch, cleared by stamps (no memset per call) */
    int stamp;
    int *mark_src, *mark_tgt, *mark_own, *mark_seen, *mark_retry, *mark_bm, *mark_esc, *mark_escs;
    unsigned char *bm_val, *esc_badin, *escs_badout;
    double *esc_extra;
    /* open-addressing hash of states: g and predecessor */
    int hcap, hcount, *hkey, *hcame, *hstamp;
    double *hg;
    /* binary heap */
    HeapItem *heap;
    int hn, hcapacity;
} Ctx;

static void tables(void) {
    static int done = 0;
    if (done) return;
    done = 1;
    for (int d = 0; d < 8; d++) {
        DLEN[d] = (DX[d] && DY[d]) ? SQ2 : 1.0;
        for (int e = 0; e < 8; e++)
            if (DX[e] == -DX[d] && DY[e] == -DY[d]) REV[d] = e;
    }
    for (int a = 0; a < NSTATE; a++)
        for (int b = 0; b < 8; b++) {
            if (a >= 8) { TURN_OK[a][b] = TURN_OK90[a][b] = 1; continue; }
            double c = (DX[a] * DX[b] + DY[a] * DY[b]) / (DLEN[a] * DLEN[b]);
            TURN_OK[a][b] = c > 0.70;
            TURN_OK90[a][b] = c > -0.01;
        }
}

EXPORT void *astar_new(int nx, int ny, int nl, const int *owner, const int *via_owner, const int *occ,
                       const double *hist, const double *dir_mult, int nfp, const int *fp_dx, const int *fp_dy) {
    tables();
    Ctx *c = (Ctx *)calloc(1, sizeof(Ctx));
    if (!c || nl > 64) return NULL;
    c->nx = nx; c->ny = ny; c->nl = nl; c->plane = nx * ny; c->size = c->plane * nl;
    c->owner = owner; c->via_owner = via_owner; c->occ = occ; c->hist = hist;
    memcpy(c->dir_mult, dir_mult, sizeof(double) * 8 * nl);
    c->nfp = nfp;
    c->fp_dx = (int *)malloc(sizeof(int) * (nfp ? nfp : 1));
    c->fp_dy = (int *)malloc(sizeof(int) * (nfp ? nfp : 1));
    memcpy(c->fp_dx, fp_dx, sizeof(int) * nfp);
    memcpy(c->fp_dy, fp_dy, sizeof(int) * nfp);
    int n = c->size;
    c->mark_src = (int *)calloc(n, sizeof(int));
    c->mark_tgt = (int *)calloc(n, sizeof(int));
    c->mark_own = (int *)calloc(n, sizeof(int));
    c->mark_seen = (int *)calloc(n, sizeof(int));
    c->mark_retry = (int *)calloc(n, sizeof(int));
    c->mark_bm = (int *)calloc(n, sizeof(int));
    c->mark_esc = (int *)calloc(n, sizeof(int));
    c->mark_escs = (int *)calloc(n, sizeof(int));
    c->bm_val = (unsigned char *)calloc(n, 1);
    c->esc_badin = (unsigned char *)calloc(n, 1);
    c->escs_badout = (unsigned char *)calloc(n, 1);
    c->esc_extra = (double *)calloc(n, sizeof(double));
    c->hcap = 1 << 16;
    c->hkey = (int *)malloc(sizeof(int) * c->hcap);
    c->hcame = (int *)malloc(sizeof(int) * c->hcap);
    c->hstamp = (int *)calloc(c->hcap, sizeof(int));
    c->hg = (double *)malloc(sizeof(double) * c->hcap);
    c->hcapacity = 1 << 14;
    c->heap = (HeapItem *)malloc(sizeof(HeapItem) * c->hcapacity);
    if (!c->mark_src || !c->mark_tgt || !c->mark_own || !c->mark_seen || !c->mark_retry || !c->mark_bm ||
        !c->mark_esc || !c->mark_escs || !c->bm_val || !c->esc_badin || !c->escs_badout || !c->esc_extra ||
        !c->hkey || !c->hcame || !c->hstamp || !c->hg || !c->heap)
        return NULL;
    return c;
}

EXPORT void astar_free(void *p) {
    Ctx *c = (Ctx *)p;
    if (!c) return;
    free(c->fp_dx); free(c->fp_dy);
    free(c->mark_src); free(c->mark_tgt); free(c->mark_own); free(c->mark_seen); free(c->mark_retry);
    free(c->mark_bm); free(c->mark_esc); free(c->mark_escs); free(c->bm_val); free(c->esc_badin);
    free(c->escs_badout); free(c->esc_extra);
    free(c->hkey); free(c->hcame); free(c->hstamp); free(c->hg); free(c->heap);
    free(c);
}

/* ---- state hash ---------------------------------------------------------------- */
static inline unsigned hslot(int key, int cap) { return ((unsigned)key * 2654435761u) & (unsigned)(cap - 1); }

static int hfind(Ctx *c, int key) {          /* slot of key, or -1 */
    unsigned i = hslot(key, c->hcap);
    for (;;) {
        if (c->hstamp[i] != c->stamp) return -1;
        if (c->hkey[i] == key) return (int)i;
        i = (i + 1) & (unsigned)(c->hcap - 1);
    }
}

static int hgrow(Ctx *c);

static int hput(Ctx *c, int key) {           /* slot for key (inserted if new), -1 on OOM */
    if (c->hcount * 2 >= c->hcap && !hgrow(c)) return -1;
    unsigned i = hslot(key, c->hcap);
    for (;;) {
        if (c->hstamp[i] != c->stamp) {
            c->hstamp[i] = c->stamp; c->hkey[i] = key; c->hg[i] = 1e300; c->hcame[i] = -1;
            c->hcount++;
            return (int)i;
        }
        if (c->hkey[i] == key) return (int)i;
        i = (i + 1) & (unsigned)(c->hcap - 1);
    }
}

static int hgrow(Ctx *c) {
    int oc = c->hcap, nc = oc * 2;
    int *ok = c->hkey, *ocame = c->hcame, *ost = c->hstamp;
    double *og = c->hg;
    int *nk = (int *)malloc(sizeof(int) * nc), *ncm = (int *)malloc(sizeof(int) * nc);
    int *nst = (int *)calloc(nc, sizeof(int));
    double *ng = (double *)malloc(sizeof(double) * nc);
    if (!nk || !ncm || !nst || !ng) { free(nk); free(ncm); free(nst); free(ng); return 0; }
    c->hkey = nk; c->hcame = ncm; c->hstamp = nst; c->hg = ng; c->hcap = nc; c->hcount = 0;
    for (int i = 0; i < oc; i++)
        if (ost[i] == c->stamp) {
            int s = hput(c, ok[i]);
            c->hg[s] = og[i]; c->hcame[s] = ocame[i];
        }
    free(ok); free(ocame); free(ost); free(og);
    return 1;
}

/* ---- heap ---------------------------------------------------------------------- */
static inline int hless(const HeapItem *a, const HeapItem *b) {
    if (a->f != b->f) return a->f < b->f;
    if (a->g != b->g) return a->g < b->g;
    return a->st < b->st;
}

static int hpush(Ctx *c, double f, double g, int st) {
    if (c->hn >= c->hcapacity) {
        int nc = c->hcapacity * 2;
        HeapItem *nh = (HeapItem *)realloc(c->heap, sizeof(HeapItem) * nc);
        if (!nh) return 0;
        c->heap = nh; c->hcapacity = nc;
    }
    int i = c->hn++;
    HeapItem it = {f, g, st};
    while (i > 0) {
        int p = (i - 1) / 2;
        if (!hless(&it, &c->heap[p])) break;
        c->heap[i] = c->heap[p];
        i = p;
    }
    c->heap[i] = it;
    return 1;
}

static HeapItem hpop(Ctx *c) {
    HeapItem top = c->heap[0];
    HeapItem last = c->heap[--c->hn];
    int i = 0, n = c->hn;
    for (;;) {
        int l = 2 * i + 1, r = l + 1, m = i;
        const HeapItem *mv = &last;
        if (l < n && hless(&c->heap[l], mv)) { m = l; mv = &c->heap[l]; }
        if (r < n && hless(&c->heap[r], mv)) { m = r; mv = &c->heap[r]; }
        if (m == i) break;
        c->heap[i] = c->heap[m];
        i = m;
    }
    if (n > 0) c->heap[i] = last;
    return top;
}

/* ---- the search ---------------------------------------------------------------- */
static inline double cell_cost(const Ctx *c, int i, double pf) {
    return (1.0 + c->hist[i]) * (1.0 + pf * c->occ[i]);
}

/* Returns path length (>0), 0 = no path, -1 = out of memory, -2 = path longer than cap. */
EXPORT int astar_run(void *p, int nid,
                     const int *sources, const int *sdirs, const double *sg0, int nsrc,
                     const int *targets, int ntgt,
                     const unsigned char *goal, int layer_only, int tlayer_mask,
                     int bx0, int by0, int bx1, int by1,
                     const unsigned char *allowed,
                     const int *own, int nown,
                     const int *bm_cells, const unsigned char *bm_vals, int nbm,
                     const int *esc_cells, const double *esc_extra, const unsigned char *esc_badin, int nesc,
                     const int *escs_cells, const unsigned char *escs_badout, int nescs,
                     double pf, double bend, double via_cost, double h_weight,
                     int right_angles, double right_cost, double sharp_cost,
                     int has_box, int gx0, int gy0, int gx1, int gy1,
                     int *out_path, int out_cap, int *out_expl, int expl_cap, int *n_expl) {
    Ctx *c = (Ctx *)p;
    const int nx = c->nx, plane = c->plane, nl = c->nl;
    const int *owner = c->owner, *occ = c->occ;
    const double *hist = c->hist;
    int stamp = ++c->stamp;
    if (stamp == 0x7fffffff) {                       /* wrap: clear every stamp array */
        int n = c->size;
        memset(c->mark_src, 0, sizeof(int) * n); memset(c->mark_tgt, 0, sizeof(int) * n);
        memset(c->mark_own, 0, sizeof(int) * n); memset(c->mark_seen, 0, sizeof(int) * n);
        memset(c->mark_retry, 0, sizeof(int) * n); memset(c->mark_bm, 0, sizeof(int) * n);
        memset(c->mark_esc, 0, sizeof(int) * n); memset(c->mark_escs, 0, sizeof(int) * n);
        memset(c->hstamp, 0, sizeof(int) * c->hcap);
        stamp = c->stamp = 1;
    }
    c->hcount = 0; c->hn = 0;
    if (n_expl) *n_expl = 0;
    for (int i = 0; i < nsrc; i++) c->mark_src[sources[i]] = stamp;
    for (int i = 0; i < ntgt; i++) c->mark_tgt[targets[i]] = stamp;
    for (int i = 0; i < nown; i++) c->mark_own[own[i]] = stamp;
    for (int i = 0; i < nbm; i++) { c->mark_bm[bm_cells[i]] = stamp; c->bm_val[bm_cells[i]] = bm_vals[i]; }
    for (int i = 0; i < nesc; i++) {
        c->mark_esc[esc_cells[i]] = stamp;
        c->esc_extra[esc_cells[i]] = esc_extra[i];
        c->esc_badin[esc_cells[i]] = esc_badin[i];
    }
    for (int i = 0; i < nescs; i++) { c->mark_escs[escs_cells[i]] = stamp; c->escs_badout[escs_cells[i]] = escs_badout[i]; }

    /* heuristic: octile distance to the target box, plus a via if not on a target layer */
    int tx0, ty0, tx1, ty1;
    int ntl = 0;
    for (int l = 0; l < nl; l++) if (tlayer_mask >> l & 1) ntl++;
    double layer_h = ntl < nl ? via_cost : 0.0;
    if (layer_only) {
        tx0 = gx0; ty0 = gy0; tx1 = gx1; ty1 = gy1;
    } else {
        tx0 = ty0 = 1 << 30; tx1 = ty1 = -1;
        for (int i = 0; i < ntgt; i++) {
            int r = targets[i] % plane, x = r % nx, y = r / nx;
            if (x < tx0) tx0 = x;
            if (x > tx1) tx1 = x;
            if (y < ty0) ty0 = y;
            if (y > ty1) ty1 = y;
        }
    }
#define HEUR(x, y, l, out) do { \
        if (layer_only && !has_box) { (out) = ((tlayer_mask >> (l)) & 1) ? 0.0 : layer_h; } \
        else { \
            int dx_ = (x) < tx0 ? tx0 - (x) : ((x) > tx1 ? (x) - tx1 : 0); \
            int dy_ = (y) < ty0 ? ty0 - (y) : ((y) > ty1 ? (y) - ty1 : 0); \
            if (dx_ < dy_) { int t_ = dx_; dx_ = dy_; dy_ = t_; } \
            double v_ = dx_ + (SQ2 - 1.0) * dy_; \
            if (!layer_only && layer_h != 0.0 && !((tlayer_mask >> (l)) & 1)) v_ += layer_h; \
            (out) = v_ * h_weight; \
        } } while (0)

    for (int i = 0; i < nsrc; i++) {
        int cell = sources[i], d0 = sdirs ? sdirs[i] : START_DIR;
        int st = cell * NSTATE + d0;
        int s = hput(c, st);
        if (s < 0) return -1;
        double g0 = sg0 ? sg0[i] : 0.0;
        if (g0 >= c->hg[s]) continue;
        c->hg[s] = g0; c->hcame[s] = -1;
        int r = cell % plane, l = cell / plane;
        double hv;
        HEUR(r % nx, r / nx, l, hv);
        if (!hpush(c, g0 + hv, g0, st)) return -1;
    }
    int hit = -1;
    int ne = 0;
    while (c->hn > 0) {
        HeapItem it = hpop(c);
        int st = it.st;
        double gc = it.g;
        int slot = hfind(c, st);
        if (slot >= 0 && gc > c->hg[slot]) continue;
        int cell = st / NSTATE, ad = st % NSTATE;
        int is_goal = layer_only ? (goal[cell] && c->mark_src[cell] != stamp) : (c->mark_tgt[cell] == stamp);
        if (is_goal) {
            int blocked = 0;
            if (ad < 8 && c->mark_bm[cell] == stamp && !((c->bm_val[cell] >> REV[ad]) & 1)) blocked = 1;
            if (!blocked) { hit = st; break; }
        }
        int first = c->mark_seen[cell] != stamp;
        if (first) {
            c->mark_seen[cell] = stamp;
            if (out_expl && ne < expl_cap) out_expl[ne++] = cell;
        }
        int l = cell / plane, r = cell % plane, y = r / nx, x = r % nx;
        const double *dm = &c->dir_mult[l * 8];
        const unsigned char *okt = right_angles ? TURN_OK90[ad] : TURN_OK[ad];
        int bmask = (ad == START_DIR && c->mark_bm[cell] == stamp) ? c->bm_val[cell] : 255;
        int src_esc = (ad == START_DIR && c->mark_escs[cell] == stamp);
        for (int d = 0; d < 8; d++) {
            if (!okt[d] || !((bmask >> d) & 1)) continue;
            int dx = DX[d], dy = DY[d];
            int x2 = x + dx, y2 = y + dy;
            if (x2 < bx0 || x2 > bx1 || y2 < by0 || y2 > by1) continue;
            if (allowed && !allowed[y2 * nx + x2]) continue;
            int n2 = cell + dy * nx + dx;
            int o = owner[n2];
            if (o != FREE && o != nid) continue;
            double ln = DLEN[d];
            double cost = (c->mark_own[n2] == stamp) ? ln : ln * (1.0 + hist[n2]) * (1.0 + pf * occ[n2]);
            cost *= dm[d];
            if (dx && dy) {
                int s1 = cell + dx, s2 = cell + dy * nx;
                int o1 = owner[s1], o2 = owner[s2];
                if ((o1 != FREE && o1 != nid) || (o2 != FREE && o2 != nid)) continue;
                if (c->mark_own[s1] != stamp) cost += cell_cost(c, s1, pf) - 1.0;
                if (c->mark_own[s2] != stamp) cost += cell_cost(c, s2, pf) - 1.0;
            }
            if (ad < 8 && ad != d) {
                cost += bend;
                if (right_angles && !TURN_OK[ad][d]) cost += right_cost;
            }
            if (c->mark_esc[n2] == stamp) {           /* arriving on an escape stub */
                cost += c->esc_extra[n2];
                if ((c->esc_badin[n2] >> d) & 1) cost += sharp_cost;
            }
            if (src_esc && ((c->escs_badout[cell] >> d) & 1)) cost += sharp_cost;
            double ng = gc + cost;
            int n2s = n2 * NSTATE + d;
            int s = hput(c, n2s);
            if (s < 0) return -1;
            if (ng < c->hg[s]) {
                c->hg[s] = ng; c->hcame[s] = st;
                double hv;
                HEUR(x2, y2, l, hv);
                if (!hpush(c, ng + hv, ng, n2s)) return -1;
            }
        }
        /* vias: once per cell (on the first visit, or the next one if that came by via) */
        int try_via = 0;
        if (nl > 1) {
            if (ad == VIA_DIR) {
                if (first) c->mark_retry[cell] = stamp;
            } else if (first || c->mark_retry[cell] == stamp) {
                c->mark_retry[cell] = 0;
                try_via = 1;
            }
        }
        if (try_via) {
            int vo = c->via_owner[r];
            int ok = (vo == FREE || vo == nid);
            for (int k = 0; ok && k < c->nfp; k++) {
                int xx = x + c->fp_dx[k], yy = y + c->fp_dy[k];
                if (xx < 0 || xx >= nx || yy < 0 || yy >= c->ny) ok = 0;
            }
            if (ok) {
                double extra = 0.0;
                for (int l2 = 0; l2 < nl; l2++) {
                    int base = l2 * plane + r;
                    for (int k = 0; k < c->nfp; k++) {
                        int q = base + c->fp_dy[k] * nx + c->fp_dx[k];
                        if (c->mark_own[q] == stamp) continue;
                        if (occ[q] || hist[q] != 0.0) extra += (1.0 + hist[q]) * (1.0 + pf * occ[q]) - 1.0;
                    }
                }
                double vc = via_cost + extra;
                for (int l2 = 0; l2 < nl; l2++) {
                    if (l2 == l) continue;
                    int n2 = l2 * plane + r;
                    int o = owner[n2];
                    if (o != FREE && o != nid) continue;
                    double ng = gc + vc;
                    int n2s = n2 * NSTATE + VIA_DIR;
                    int s = hput(c, n2s);
                    if (s < 0) return -1;
                    if (ng < c->hg[s]) {
                        c->hg[s] = ng; c->hcame[s] = st;
                        double hv;
                        HEUR(x, y, l2, hv);
                        if (!hpush(c, ng + hv, ng, n2s)) return -1;
                    }
                }
            }
        }
    }
    if (n_expl) *n_expl = ne;
    if (hit < 0) return 0;
    /* walk back, then reverse */
    int n = 0, st = hit;
    while (st != -1) {
        if (n >= out_cap) return -2;
        out_path[n++] = st / NSTATE;
        int s = hfind(c, st);
        st = (s >= 0) ? c->hcame[s] : -1;
    }
    for (int i = 0; i < n / 2; i++) { int t = out_path[i]; out_path[i] = out_path[n - 1 - i]; out_path[n - 1 - i] = t; }
    return n;
}
