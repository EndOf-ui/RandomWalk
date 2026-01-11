// server_main.c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>

#include "sim.h"
#include "socket.h"

#define DEFAULT_PORT 5555
#define BUF_SIZE 4096

static Sim g_sim;
static bool g_sim_initialized = false;
static bool g_sim_running = false;
static int  g_mode_interactive = 0;

static pthread_mutex_t g_sim_mutex = PTHREAD_MUTEX_INITIALIZER;

static void trim_newline(char *s) {
    if (!s) return;
    size_t n = strlen(s);
    while (n > 0 && (s[n-1] == '\n' || s[n-1] == '\r')) {
        s[n-1] = '\0';
        n--;
    }
}

static int send_all(int sock, const char *data) {
    size_t len = strlen(data);
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = send(sock, data + sent, len - sent, 0);
        if (n <= 0) return -1;
        sent += (size_t)n;
    }
    return 0;
}

static void cmd_new_sim(int sock, char *args) {
    int H, W, wt, reps, K;
    double pU, pD, pL, pR;
    char out[256] = {0};

    int n = sscanf(args, "%d %d %d %lf %lf %lf %lf %d %d %255s",
                   &H, &W, &wt,
                   &pU, &pD, &pL, &pR,
                   &K, &reps, out);
    if (n != 10) {
        send_all(sock, "ERR Bad NEW_SIM params\n");
        return;
    }

    pthread_mutex_lock(&g_sim_mutex);

    sim_free(&g_sim);
    memset(&g_sim, 0, sizeof(g_sim));
    if (!sim_init_empty(&g_sim, H, W, (bool)wt)) {
        pthread_mutex_unlock(&g_sim_mutex);
        send_all(sock, "ERR sim_init_empty\n");
        return;
    }

    g_sim.K = K;
    g_sim.MoveProbs[0] = pU;
    g_sim.MoveProbs[1] = pD;
    g_sim.MoveProbs[2] = pL;
    g_sim.MoveProbs[3] = pR;
    strncpy(g_sim.ResultFilePath, out, sizeof(g_sim.ResultFilePath)-1);

    if (g_sim.WorldType) {
        double dens = 0.2;
        if (!sim_generate_obstacles_connected(&g_sim, dens, (uint64_t)time(NULL))) {
            pthread_mutex_unlock(&g_sim_mutex);
            send_all(sock, "ERR generate_obstacles\n");
            return;
        }
    }

    if (!sim_run(&g_sim, reps, (uint64_t)time(NULL))) {
        pthread_mutex_unlock(&g_sim_mutex);
        send_all(sock, "ERR sim_run\n");
        return;
    }

    g_sim_initialized = true;
    g_sim_running = true;

    char resp[128];
    snprintf(resp, sizeof(resp),
             "OK NEW_SIM ActRep=%d\n", g_sim.ActRep);
    pthread_mutex_unlock(&g_sim_mutex);

    send_all(sock, resp);
}

static void cmd_resume_sim(int sock, char *args) {
    char inFile[256] = {0};
    int reps;
    char outFile[256] = {0};

    int n = sscanf(args, "%255s %d %255s", inFile, &reps, outFile);
    if (n != 3) {
        send_all(sock, "ERR Bad RESUME_SIM params\n");
        return;
    }

    pthread_mutex_lock(&g_sim_mutex);

    sim_free(&g_sim);
    memset(&g_sim, 0, sizeof(g_sim));
    if (!sim_load_state(&g_sim, inFile)) {
        pthread_mutex_unlock(&g_sim_mutex);
        send_all(sock, "ERR sim_load_state\n");
        return;
    }
    strncpy(g_sim.ResultFilePath, outFile, sizeof(g_sim.ResultFilePath)-1);

    if (!sim_run(&g_sim, reps, (uint64_t)time(NULL))) {
        pthread_mutex_unlock(&g_sim_mutex);
        send_all(sock, "ERR sim_run\n");
        return;
    }

    g_sim_initialized = true;
    g_sim_running = true;

    char resp[128];
    snprintf(resp, sizeof(resp),
             "OK RESUME_SIM ActRep=%d\n", g_sim.ActRep);
    pthread_mutex_unlock(&g_sim_mutex);

    send_all(sock, resp);
}

static void cmd_run_more(int sock, char *args) {
    int reps;
    if (sscanf(args, "%d", &reps) != 1 || reps <= 0) {
        send_all(sock, "ERR Bad RUN_MORE params\n");
        return;
    }

    pthread_mutex_lock(&g_sim_mutex);
    if (!g_sim_initialized) {
        pthread_mutex_unlock(&g_sim_mutex);
        send_all(sock, "ERR No simulation\n");
        return;
    }
    if (!sim_run(&g_sim, reps, (uint64_t)time(NULL))) {
        pthread_mutex_unlock(&g_sim_mutex);
        send_all(sock, "ERR sim_run\n");
        return;
    }

    char resp[128];
    snprintf(resp, sizeof(resp),
             "OK RUN_MORE ActRep=%d\n", g_sim.ActRep);
    pthread_mutex_unlock(&g_sim_mutex);

    send_all(sock, resp);
}

static void cmd_set_mode(int sock, char *args) {
    int m;
    if (sscanf(args, "%d", &m) != 1 || (m != 0 && m != 1)) {
        send_all(sock, "ERR Bad SET_MODE\n");
        return;
    }

    pthread_mutex_lock(&g_sim_mutex);
    g_mode_interactive = m;
    pthread_mutex_unlock(&g_sim_mutex);

    send_all(sock, "OK SET_MODE\n");
}

static void cmd_get_summary_avg(int sock) {
    pthread_mutex_lock(&g_sim_mutex);
    if (!g_sim_initialized) {
        pthread_mutex_unlock(&g_sim_mutex);
        send_all(sock, "ERR No simulation\n");
        return;
    }

    int H = g_sim.WorldHeight;
    int W = g_sim.WorldWidth;

    char line[1024];
    snprintf(line, sizeof(line),
             "OK SUMMARY_AVG H=%d W=%d ActRep=%d\n",
             H, W, g_sim.ActRep);
    send_all(sock, line);

    for (int r = 0; r < H; r++) {
        line[0] = '\0';
        for (int c = 0; c < W; c++) {
            int i = r * W + c;
            if (g_sim.WorldType && g_sim.obstacle[i]) {
                strcat(line, "X ");
            } else {
                double avg = (g_sim.ActRep > 0)
                             ? (double)g_sim.steps_sum[i] / (double)g_sim.ActRep
                             : 0.0;
                char buf[32];
                snprintf(buf, sizeof(buf), "%.1f ", avg);
                strcat(line, buf);
            }
        }
        strcat(line, "\n");
        send_all(sock, line);
    }

    pthread_mutex_unlock(&g_sim_mutex);
}

static void cmd_get_summary_prob(int sock) {
    pthread_mutex_lock(&g_sim_mutex);
    if (!g_sim_initialized) {
        pthread_mutex_unlock(&g_sim_mutex);
        send_all(sock, "ERR No simulation\n");
        return;
    }

    int H = g_sim.WorldHeight;
    int W = g_sim.WorldWidth;

    char line[1024];
    snprintf(line, sizeof(line),
             "OK SUMMARY_PROB H=%d W=%d K=%d ActRep=%d\n",
             H, W, g_sim.K, g_sim.ActRep);
    send_all(sock, line);

    for (int r = 0; r < H; r++) {
        line[0] = '\0';
        for (int c = 0; c < W; c++) {
            int i = r * W + c;
            if (g_sim.WorldType && g_sim.obstacle[i]) {
                strcat(line, "X ");
            } else {
                double pr = (g_sim.ActRep > 0)
                            ? (double)g_sim.hits_sum[i] / (double)g_sim.ActRep
                            : 0.0;
                char buf[32];
                snprintf(buf, sizeof(buf), "%.2f ", pr);
                strcat(line, buf);
            }
        }
        strcat(line, "\n");
        send_all(sock, line);
    }

    pthread_mutex_unlock(&g_sim_mutex);
}

static void cmd_end_sim(int sock) {
    pthread_mutex_lock(&g_sim_mutex);
    if (g_sim_initialized) {
        sim_save_state(&g_sim, g_sim.ResultFilePath[0] ? g_sim.ResultFilePath : "result.txt");
        sim_free(&g_sim);
        memset(&g_sim, 0, sizeof(g_sim));
        g_sim_initialized = false;
        g_sim_running = false;
    }
    pthread_mutex_unlock(&g_sim_mutex);

    send_all(sock, "OK END_SIM\n");
}

static void *client_thread(void *arg) {
    int sock = *(int*)arg;
    free(arg);

    char buf[BUF_SIZE];

    send_all(sock, "HELLO RandomWalkServer\n");

    while (1) {
        ssize_t n = recv(sock, buf, sizeof(buf)-1, 0);
        if (n <= 0) break;
        buf[n] = '\0';
        trim_newline(buf);
        if (strlen(buf) == 0) continue;

        char cmd[64] = {0};
        char *args = NULL;

        char *space = strchr(buf, ' ');
        if (space) {
            size_t len = (size_t)(space - buf);
            if (len >= sizeof(cmd)) len = sizeof(cmd)-1;
            memcpy(cmd, buf, len);
            cmd[len] = '\0';
            args = space + 1;
        } else {
            strncpy(cmd, buf, sizeof(cmd)-1);
            args = buf + strlen(buf);
        }

        if (strcmp(cmd, "NEW_SIM") == 0) {
            cmd_new_sim(sock, args);
        } else if (strcmp(cmd, "RESUME_SIM") == 0) {
            cmd_resume_sim(sock, args);
        } else if (strcmp(cmd, "RUN_MORE") == 0) {
            cmd_run_more(sock, args);
        } else if (strcmp(cmd, "SET_MODE") == 0) {
            cmd_set_mode(sock, args);
        } else if (strcmp(cmd, "GET_SUMMARY_AVG") == 0) {
            cmd_get_summary_avg(sock);
        } else if (strcmp(cmd, "GET_SUMMARY_PROB") == 0) {
            cmd_get_summary_prob(sock);
        } else if (strcmp(cmd, "END_SIM") == 0) {
            cmd_end_sim(sock);
        } else if (strcmp(cmd, "QUIT") == 0) {
            send_all(sock, "OK BYE\n");
            break;
        } else {
            send_all(sock, "ERR Unknown command\n");
        }
    }

    close(sock);
    return NULL;
}

int main(int argc, char *argv[]) {
    int port = DEFAULT_PORT;
    if (argc >= 2) {
        port = atoi(argv[1]);
        if (port <= 0) port = DEFAULT_PORT;
    }

    int passive = passive_socket_init(port);
    if (passive < 0) {
        fprintf(stderr, "Failed to init server socket\n");
        return 1;
    }

    printf("Server listening on port %d\n", port);

    for (;;) {
        int *pSock = malloc(sizeof(int));
        if (!pSock) continue;

        *pSock = passive_socket_wait_for_client(passive);
        if (*pSock < 0) {
            free(pSock);
            continue;
        }

        pthread_t tid;
        pthread_create(&tid, NULL, client_thread, pSock);
        pthread_detach(tid);
    }

    passive_socket_destroy(passive);
    return 0;
}

