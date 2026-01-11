// client_main.c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>

#include "socket.h"

#define BUF_SIZE 4096

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

static ssize_t recv_line(int sock, char *buf, size_t maxlen) {
    size_t pos = 0;
    while (pos + 1 < maxlen) {
        char c;
        ssize_t n = recv(sock, &c, 1, 0);
        if (n <= 0) {
            if (pos == 0) return n;
            break;
        }
        if (c == '\n') {
            buf[pos] = '\0';
            return (ssize_t)pos;
        }
        buf[pos++] = c;
    }
    buf[pos] = '\0';
    return (ssize_t)pos;
}

static void menu() {
    printf("\n--- Random Walk CLIENT ---\n");
    printf("1) Nova simulacia\n");
    printf("2) Obnovit simulaciu zo suboru\n");
    printf("3) Spustit dalsie replikacie\n");
    printf("4) Zobrazit priemer krokov\n");
    printf("5) Zobrazit pravdepodobnost do K\n");
    printf("6) Nastavit mod (0=sumar,1=interaktivny)\n");
    printf("0) Koniec (QUIT)\n");
    printf("Volba: ");
    fflush(stdout);
}

int main(int argc, char *argv[]) {
    const char *serverName = "127.0.0.1";
    int port = 5555;

    if (argc >= 2) serverName = argv[1];
    if (argc >= 3) {
        port = atoi(argv[2]);
        if (port <= 0) port = 5555;
    }

    int sock = connect_to_server(serverName, port);
    if (sock < 0) {
        fprintf(stderr, "Neviem sa pripojit na server %s:%d\n", serverName, port);
        return 1;
    }

    char buf[BUF_SIZE];

    ssize_t n = recv_line(sock, buf, sizeof(buf));
    if (n > 0) {
        printf("%s\n", buf);
    }

    for (;;) {
        menu();
        int choice = -1;
        if (scanf("%d", &choice) != 1) break;

        int ch;
        while ((ch = getchar()) != '\n' && ch != EOF) {}

        if (choice == 0) {
            send_all(sock, "QUIT\n");
            n = recv_line(sock, buf, sizeof(buf));
            if (n > 0) printf("%s\n", buf);
            break;
        } else if (choice == 1) {
            int H,W,wt,reps,K;
            double pU,pD,pL,pR;
            char out[256];

            printf("WorldHeight: "); scanf("%d", &H);
            printf("WorldWidth: "); scanf("%d", &W);
            printf("WorldType (0=bez,1=prekazky): "); scanf("%d", &wt);
            printf("MoveProbs U D L R (sum=1): "); scanf("%lf %lf %lf %lf", &pU,&pD,&pL,&pR);
            printf("K (max krokov): "); scanf("%d", &K);
            printf("Pocet replikacii: "); scanf("%d", &reps);
            int c;
            while ((c = getchar()) != '\n' && c != EOF) {}
            printf("Subor pre ulozenie stavu: ");
            if (!fgets(out, sizeof(out), stdin)) continue;
            trim_newline(out);

            char cmd[BUF_SIZE];
            snprintf(cmd, sizeof(cmd),
                     "NEW_SIM %d %d %d %f %f %f %f %d %d %s\n",
                     H, W, wt, pU, pD, pL, pR, K, reps, out);
            send_all(sock, cmd);

            n = recv_line(sock, buf, sizeof(buf));
            if (n > 0) printf("%s\n", buf);

        } else if (choice == 2) {
            char inFile[256], outFile[256];
            int reps;

            printf("Subor s ulozenou simulaciou: ");
            if (!fgets(inFile, sizeof(inFile), stdin)) continue;
            trim_newline(inFile);
            printf("Pocet replikacii navyse: ");
            scanf("%d", &reps);
            int c;
            while ((c = getchar()) != '\n' && c != EOF) {}
            printf("Subor pre ulozenie vysledku: ");
            if (!fgets(outFile, sizeof(outFile), stdin)) continue;
            trim_newline(outFile);

            char cmd[BUF_SIZE];
            snprintf(cmd, sizeof(cmd),
                     "RESUME_SIM %s %d %s\n",
                     inFile, reps, outFile);
            send_all(sock, cmd);

            n = recv_line(sock, buf, sizeof(buf));
            if (n > 0) printf("%s\n", buf);

        } else if (choice == 3) {
            int reps;
            printf("Kolko dalsich replikacii: ");
            scanf("%d", &reps);
            int c;
            while ((c = getchar()) != '\n' && c != EOF) {}

            char cmd[128];
            snprintf(cmd, sizeof(cmd),
                     "RUN_MORE %d\n", reps);
            send_all(sock, cmd);

            n = recv_line(sock, buf, sizeof(buf));
            if (n > 0) printf("%s\n", buf);

        } else if (choice == 4) {
            send_all(sock, "GET_SUMMARY_AVG\n");

            n = recv_line(sock, buf, sizeof(buf));
            if (n <= 0) {
                printf("Chyba odpovede\n");
                continue;
            }
            printf("%s\n", buf);

            int H=0,W=0;
            sscanf(buf, "OK SUMMARY_AVG H=%d W=%d", &H, &W);

            for (int r = 0; r < H; r++) {
                n = recv_line(sock, buf, sizeof(buf));
                if (n <= 0) break;
                printf("%s\n", buf);
            }

        } else if (choice == 5) {
            send_all(sock, "GET_SUMMARY_PROB\n");

            n = recv_line(sock, buf, sizeof(buf));
            if (n <= 0) {
                printf("Chyba odpovede\n");
                continue;
            }
            printf("%s\n", buf);

            int H=0,W=0;
            sscanf(buf, "OK SUMMARY_PROB H=%d W=%d", &H, &W);

            for (int r = 0; r < H; r++) {
                n = recv_line(sock, buf, sizeof(buf));
                if (n <= 0) break;
                printf("%s\n", buf);
            }

        } else if (choice == 6) {
            int m;
            printf("Zadaj mod (0=sumar,1=interaktivny): ");
            scanf("%d", &m);
            int c;
            while ((c = getchar()) != '\n' && c != EOF) {}

            char cmd[64];
            snprintf(cmd, sizeof(cmd),
                     "SET_MODE %d\n", m);
            send_all(sock, cmd);

            n = recv_line(sock, buf, sizeof(buf));
            if (n > 0) printf("%s\n", buf);
        } else {
            printf("Neznama volba.\n");
        }
    }

    close(sock);
    return 0;
}
