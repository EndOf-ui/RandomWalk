#include "sim.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static inline int idx(const Sim *s, int r, int c) { return r * s->WorldWidth + c; }

static bool alloc_arrays(Sim *s) {
    size_t n = (size_t)s->WorldHeight * (size_t)s->WorldWidth;
    s->obstacle  = (bool*)calloc(n, sizeof(bool));
    s->steps_sum = (uint64_t*)calloc(n, sizeof(uint64_t));
    s->hits_sum  = (uint64_t*)calloc(n, sizeof(uint64_t));
    return s->obstacle && s->steps_sum && s->hits_sum;
}

bool sim_init_empty(Sim *s, int h, int w, bool worldType) {
    if (!s || h <= 0 || w <= 0) return false;
    memset(s, 0, sizeof(*s));
    s->WorldHeight = h;
    s->WorldWidth = w;
    s->WorldType = worldType;
    s->MaxReps = 0;
    s->ActRep = 0;
    s->K = 100;
    s->MoveProbs[0] = 0.25; s->MoveProbs[1] = 0.25; s->MoveProbs[2] = 0.25; s->MoveProbs[3] = 0.25;
    return alloc_arrays(s);
}

void sim_free(Sim *s) {
    if (!s) return;
    free(s->obstacle);  s->obstacle = NULL;
    free(s->steps_sum); s->steps_sum = NULL;
    free(s->hits_sum);  s->hits_sum = NULL;
}

static bool probs_ok(const double p[4]) {
    double sum = p[0] + p[1] + p[2] + p[3];
    if (sum < 0.999999 || sum > 1.000001) return false;
    for (int i = 0; i < 4; i++) if (p[i] < 0.0) return false;
    return true;
}

static int sample_dir(const double p[4]) {
    double r = (double)rand() / (double)RAND_MAX;
    double c = 0.0;
    for (int i = 0; i < 4; i++) {
        c += p[i];
        if (r <= c) return i;
    }
    return 3;
}

// torus wrap
static inline int wrap(int x, int m) {
    x %= m;
    if (x < 0) x += m;
    return x;
}

static void step_try(const Sim *s, int *r, int *c, int dir) {
    int nr = *r, nc = *c;
    if (dir == 0) nr--;        // U
    else if (dir == 1) nr++;   // D
    else if (dir == 2) nc--;   // L
    else nc++;                 // R

    nr = wrap(nr, s->WorldHeight);
    nc = wrap(nc, s->WorldWidth);

    // ak sú prekážky a cieľ je prekážka, ostaneme na mieste
    if (s->WorldType && s->obstacle[idx(s, nr, nc)]) return;

    *r = nr; *c = nc;
}

static bool bfs_connected_from_center(const Sim *s) {
    int H = s->WorldHeight, W = s->WorldWidth;
    int cr = H/2, cc = W/2;
    if (s->obstacle[idx(s, cr, cc)]) return false;

    size_t n = (size_t)H * (size_t)W;
    bool *vis = (bool*)calloc(n, sizeof(bool));
    int *q = (int*)malloc((int)n * (int)sizeof(int));
    if (!vis || !q) { free(vis); free(q); return false; }

    int qh = 0, qt = 0;
    vis[idx(s, cr, cc)] = true;
    q[qt++] = idx(s, cr, cc);

    while (qh < qt) {
        int v = q[qh++];
        int r = v / W, c = v % W;

        const int dr[4] = {-1, +1, 0, 0};
        const int dc[4] = {0, 0, -1, +1};

        for (int i = 0; i < 4; i++) {
            int nr = wrap(r + dr[i], H);
            int nc = wrap(c + dc[i], W);
            int ni = idx(s, nr, nc);
            if (s->WorldType && s->obstacle[ni]) continue;
            if (!vis[ni]) { vis[ni] = true; q[qt++] = ni; }
        }
    }

    // každý non-obstacle musí byť reachable
    for (int i = 0; i < (int)n; i++) {
        if (s->WorldType && s->obstacle[i]) continue;
        if (!vis[i]) { free(vis); free(q); return false; }
    }

    free(vis);
    free(q);
    return true;
}

bool sim_generate_obstacles_connected(Sim *s, double obstacleDensity, uint64_t seed) {
    if (!s || !s->obstacle) return false;
    if (!s->WorldType) return true; // bez prekážok netreba
    if (obstacleDensity < 0.0) obstacleDensity = 0.0;
    if (obstacleDensity > 0.80) obstacleDensity = 0.80; // aby bolo realistické nájsť connected

    srand((unsigned)(seed ? seed : (uint64_t)time(NULL)));

    int H = s->WorldHeight, W = s->WorldWidth;
    int cr = H/2, cc = W/2;

    size_t n = (size_t)H * (size_t)W;

    // skúšaj generovať, kým je svet connected
    for (int attempt = 0; attempt < 5000; attempt++) {
        memset(s->obstacle, 0, n * sizeof(bool));

        for (int r = 0; r < H; r++) {
            for (int c = 0; c < W; c++) {
                if (r == cr && c == cc) continue; // [0,0] nesmie byť prekážka
                double u = (double)rand() / (double)RAND_MAX;
                if (u < obstacleDensity) s->obstacle[idx(s, r, c)] = true;
            }
        }

        if (bfs_connected_from_center(s)) return true;
    }
    return false;
}

static uint32_t walk_until_center(const Sim *s, int sr, int sc, int *hitWithinK) {
    int H = s->WorldHeight, W = s->WorldWidth;
    int cr = H/2, cc = W/2;

    int r = sr, c = sc;
    uint32_t steps = 0;
    *hitWithinK = 0;

    while (!(r == cr && c == cc)) {
        int dir = sample_dir(s->MoveProbs);
        step_try(s, &r, &c, dir);
        steps++;
        if (steps == (uint32_t)s->K && (r == cr && c == cc)) *hitWithinK = 1;
        if (steps < (uint32_t)s->K && (r == cr && c == cc)) *hitWithinK = 1;
        // Na konečnom grafe (connected) je zásah takmer iste, takže netreba hard limit.
    }
    return steps;
}

bool sim_run(Sim *s, int addReps, uint64_t seed) {
    if (!s || addReps <= 0) return false;
    if (!probs_ok(s->MoveProbs)) return false;

    srand((unsigned)(seed ? seed : (uint64_t)time(NULL)));

    int H = s->WorldHeight, W = s->WorldWidth;
    int cr = H/2, cc = W/2;

    for (int rep = 0; rep < addReps; rep++) {
        for (int r = 0; r < H; r++) {
            for (int c = 0; c < W; c++) {
                int i = idx(s, r, c);
                if (s->WorldType && s->obstacle[i]) continue;

                if (r == cr && c == cc) {
                    s->steps_sum[i] += 0;
                    s->hits_sum[i] += 1; // do K krokov je to pravda (0 krokov)
                    continue;
                }

                int hitK = 0;
                uint32_t steps = walk_until_center(s, r, c, &hitK);
                s->steps_sum[i] += (uint64_t)steps;
                s->hits_sum[i] += (uint64_t)hitK;
            }
        }
        s->ActRep++;
        if (s->MaxReps < s->ActRep) s->MaxReps = s->ActRep;
        if (s->SimEnd) break;
    }
    return true;
}

bool sim_save_state(const Sim *s, const char *path) {
    if (!s || !path) return false;
    FILE *f = fopen(path, "w");
    if (!f) return false;

    fprintf(f, "DWALK1\n");
    fprintf(f, "%d %d\n", s->WorldHeight, s->WorldWidth);
    fprintf(f, "%d\n", (int)s->WorldType);
    fprintf(f, "%d\n", s->K);
    fprintf(f, "%.17g %.17g %.17g %.17g\n", s->MoveProbs[0], s->MoveProbs[1], s->MoveProbs[2], s->MoveProbs[3]);
    fprintf(f, "%d %d\n", s->MaxReps, s->ActRep);

    int H = s->WorldHeight, W = s->WorldWidth;
    for (int r = 0; r < H; r++) {
        for (int c = 0; c < W; c++) fprintf(f, "%d", s->obstacle[idx(s,r,c)] ? 1 : 0);
        fprintf(f, "\n");
    }

    for (int r = 0; r < H; r++) {
        for (int c = 0; c < W; c++) fprintf(f, "%llu ", (unsigned long long)s->steps_sum[idx(s,r,c)]);
        fprintf(f, "\n");
    }

    for (int r = 0; r < H; r++) {
        for (int c = 0; c < W; c++) fprintf(f, "%llu ", (unsigned long long)s->hits_sum[idx(s,r,c)]);
        fprintf(f, "\n");
    }

    fclose(f);
    return true;
}

bool sim_load_state(Sim *s, const char *path) {
    if (!s || !path) return false;
    FILE *f = fopen(path, "r");
    if (!f) return false;

    char magic[32] = {0};
    if (!fgets(magic, (int)sizeof(magic), f)) { fclose(f); return false; }
    if (strncmp(magic, "DWALK1", 5) != 0) { fclose(f); return false; }

    int H=0, W=0, wt=0, K=0, maxReps=0, actRep=0;
    double p0,p1,p2,p3;

    if (fscanf(f, "%d %d\n", &H, &W) != 2) { fclose(f); return false; }
    if (fscanf(f, "%d\n", &wt) != 1) { fclose(f); return false; }
    if (fscanf(f, "%d\n", &K) != 1) { fclose(f); return false; }
    if (fscanf(f, "%lf %lf %lf %lf\n", &p0,&p1,&p2,&p3) != 4) { fclose(f); return false; }
    if (fscanf(f, "%d %d\n", &maxReps, &actRep) != 2) { fclose(f); return false; }

    sim_free(s);
    if (!sim_init_empty(s, H, W, (bool)wt)) { fclose(f); return false; }

    s->K = K;
    s->MoveProbs[0]=p0; s->MoveProbs[1]=p1; s->MoveProbs[2]=p2; s->MoveProbs[3]=p3;
    s->MaxReps = maxReps;
    s->ActRep = actRep;

    // obstacles
    for (int r = 0; r < H; r++) {
        char line[8192];
        if (!fgets(line, (int)sizeof(line), f)) { fclose(f); return false; }
        // môže to byť prázdny riadok po fscanf -> preskoč, ak treba
        if ((int)strlen(line) < W) { r--; continue; }
        for (int c = 0; c < W; c++) s->obstacle[idx(s,r,c)] = (line[c] == '1');
    }

    for (int r = 0; r < H; r++) {
        for (int c = 0; c < W; c++) {
            unsigned long long v=0;
            if (fscanf(f, "%llu", &v) != 1) { fclose(f); return false; }
            s->steps_sum[idx(s,r,c)] = (uint64_t)v;
        }
    }

    for (int r = 0; r < H; r++) {
        for (int c = 0; c < W; c++) {
            unsigned long long v=0;
            if (fscanf(f, "%llu", &v) != 1) { fclose(f); return false; }
            s->hits_sum[idx(s,r,c)] = (uint64_t)v;
        }
    }

    fclose(f);
    return true;
}

void sim_print_summary_avg_steps(const Sim *s) {
    int H = s->WorldHeight, W = s->WorldWidth;
    printf("Average steps to center (ActRep=%d):\n", s->ActRep);
    for (int r = 0; r < H; r++) {
        for (int c = 0; c < W; c++) {
            int i = idx(s,r,c);
            if (s->WorldType && s->obstacle[i]) { printf("  X   "); continue; }
            double avg = (s->ActRep > 0) ? (double)s->steps_sum[i] / (double)s->ActRep : 0.0;
            printf("%5.1f ", avg);
        }
        printf("\n");
    }
}

void sim_print_summary_prob_k(const Sim *s) {
    int H = s->WorldHeight, W = s->WorldWidth;
    printf("P(reach center within K=%d) (ActRep=%d):\n", s->K, s->ActRep);
    for (int r = 0; r < H; r++) {
        for (int c = 0; c < W; c++) {
            int i = idx(s,r,c);
            if (s->WorldType && s->obstacle[i]) { printf("  X   "); continue; }
            double pr = (s->ActRep > 0) ? (double)s->hits_sum[i] / (double)s->ActRep : 0.0;
            printf("%5.2f ", pr);
        }
        printf("\n");
    }
}