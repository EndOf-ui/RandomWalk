#ifndef SIM_H
#define SIM_H

#include <stdbool.h>
#include <stdint.h>

#ifndef PATH_MAX
#define PATH_MAX 4096
#endif

// client-side (neskôr; zatiaľ nepoužité)
typedef struct ClientData {
    bool SimMode; // false=sumárny, true=interaktívny (neskôr)
    bool SimEnd;  // žiadosť ukončiť (neskôr)
} ClientData;

// server-side (tu používané aj v single-process režime)
typedef struct Sim {
    char WorldFilePath[PATH_MAX];   // ak sa načítava svet z externého súboru (voliteľné)
    bool WorldType;                 // 0=bez prekážok, 1=s prekážkami
    int WorldHeight;
    int WorldWidth;

    int MaxReps;
    int ActRep;                     // aktuálna (koľko je už hotových)

    double MoveProbs[4];            // U, D, L, R (súčet = 1)
    int K;                          // max krokov pre pravdepodobnosť

    char ResultFilePath[PATH_MAX];  // súbor na uloženie stavu

    int DrunkCoords[2];             // (row, col) pre interaktívny mód (neskôr)
    bool SimEnd;

    // --- interné polia pre sumár (per-cell) ---
    bool *obstacle;                 // H*W
    uint64_t *steps_sum;            // H*W
    uint64_t *hits_sum;             // H*W (počet zásahov do K)
} Sim;

bool sim_init_empty(Sim *s, int h, int w, bool worldType);
void sim_free(Sim *s);

bool sim_generate_obstacles_connected(Sim *s, double obstacleDensity, uint64_t seed);

bool sim_run(Sim *s, int addReps, uint64_t seed);
bool sim_save_state(const Sim *s, const char *path);
bool sim_load_state(Sim *s, const char *path);

void sim_print_summary_avg_steps(const Sim *s);
void sim_print_summary_prob_k(const Sim *s);

#endif
