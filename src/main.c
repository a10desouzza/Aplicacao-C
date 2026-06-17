#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <dirent.h>
#include <time.h>
#include <math.h>
#include <termios.h>
#include <sys/select.h>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include "elm_driver.h"

/* ===== Caminhos dos parametros da rede (ajuste se necessario) ===== */
#define PATH_WEIGHTS "../data/w_in_q.bin"
#define PATH_BETA    "../data/beta_q.bin"
#define PATH_BIAS    "../data/bias_q.bin"

/* Caminhos-base dos parametros - padroes acima; sobrescritos pela linha de
 * comando (-w/-b/-s) e, em tempo de execucao, pelos comandos de envio. */
static char g_path_w [256] = PATH_WEIGHTS;
static char g_path_bt[256] = PATH_BETA;
static char g_path_bs[256] = PATH_BIAS;

/* ===================== Geometria do VGA ===================== */
#define VGA_W   320
#define VGA_H   240
#define GRID    28                 /* imagem 28x28            */
#define CELL    8                  /* cada pixel -> bloco 8x8 */
#define CURSOR_CELLS 1             /* cursor 8x8 = 1 celula */
#define CANVAS  (GRID * CELL)      /* 224                     */
#define OFF_X   ((VGA_W - CANVAS) / 2)   /* 48 */
#define OFF_Y   ((VGA_H - CANVAS) / 2)   /* 8  */

/* ===================== PIOs do VGA na ponte lightweight ===================== */
#define LW_BRIDGE_BASE   0xFF200000
#define LW_BRIDGE_SPAN   0x00001000
#define VGA_STATUS_OFFSET  0x30
#define VGA_SIGNALS_OFFSET 0x40
#define VGA_DATA_OFFSET    0x50

/* pio_vga_signals: bit0 = enable, bit1 = reset */
#define VGA_SIG_ENABLE (1u << 0)
#define VGA_SIG_RESET  (1u << 1)

static volatile uint32_t *g_data;     /* pixel  (0x50) */
static volatile uint32_t *g_signals;  /* ctrl   (0x40) */
static volatile uint32_t *g_status;   /* done   (0x30) */
static void *g_base = MAP_FAILED;
static int   g_fd   = -1;

/* ===================== Camada VGA ===================== */

/* Empacotamento conforme o ghrd_top.v. */
static uint32_t pack_pixel(int x, int y, int r, int g, int b) {
    return ((uint32_t)(y & 0x3FF) << 19)
         | ((uint32_t)(x & 0x3FF) << 9)
         | ((uint32_t)(r & 0x7) << 6)        
         | ((uint32_t)(g & 0x7) << 3)        
         | ((uint32_t)(b & 0x7));            
}

static int vga_init(void) {
    g_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (g_fd < 0) { perror("open /dev/mem"); return -1; }
    g_base = mmap(NULL, LW_BRIDGE_SPAN, PROT_READ | PROT_WRITE,
                  MAP_SHARED, g_fd, LW_BRIDGE_BASE);
    if (g_base == MAP_FAILED) { perror("mmap"); close(g_fd); g_fd = -1; return -2; }
    g_status  = (volatile uint32_t *)((char *)g_base + VGA_STATUS_OFFSET);
    g_signals = (volatile uint32_t *)((char *)g_base + VGA_SIGNALS_OFFSET);
    g_data    = (volatile uint32_t *)((char *)g_base + VGA_DATA_OFFSET);

    *g_signals = VGA_SIG_RESET; usleep(1000);
    *g_signals = 0;             usleep(1000);
    return 0;
}

static void vga_close(void) {
    if (g_base != MAP_FAILED) munmap(g_base, LW_BRIDGE_SPAN);
    if (g_fd >= 0) close(g_fd);
    g_base = MAP_FAILED; g_fd = -1;
}

static void vga_set_pixel(int x, int y, int r, int g, int b) {
    uint32_t word;
    if (x < 0 || x >= VGA_W || y < 0 || y >= VGA_H) return;
    word = pack_pixel(x, y, r, g, b);
    *g_data    = word;                 
    *g_signals = VGA_SIG_ENABLE;       
    while ((*g_status & 0x1) == 0) { } 
    *g_signals = 0;                    
}

static void vga_fill_rect(int x0, int y0, int w, int h, int r, int g, int b) {
    int x, y;
    for (y = y0; y < y0 + h; y++)
        for (x = x0; x < x0 + w; x++)
            vga_set_pixel(x, y, r, g, b);
}

static void vga_clear(int r, int g, int b) {
    vga_fill_rect(0, 0, VGA_W, VGA_H, r, g, b);
}

static void vga_show_image28(const uint8_t img[GRID * GRID]) {
    int gx, gy;
    vga_clear(0, 0, 0);
    for (gy = 0; gy < GRID; gy++) {
        for (gx = 0; gx < GRID; gx++) {
            int v = img[gy * GRID + gx] >> 5;   
            vga_fill_rect(OFF_X + gx * CELL, OFF_Y + gy * CELL,
                          CELL, CELL, v, v, v);
        }
    }
}

static void put_cell_rgb(int gx, int gy, int r, int g, int b) {
    if (gx < 0 || gx >= GRID || gy < 0 || gy >= GRID) return;
    vga_fill_rect(OFF_X + gx * CELL, OFF_Y + gy * CELL, CELL, CELL, r, g, b);
}

static void put_cell(int gx, int gy, int on) {
    int v = on ? 7 : 0;
    put_cell_rgb(gx, gy, v, v, v);
}

static void draw_cursor(int gx, int gy, int r, int g, int b) {
    int dx, dy;
    for (dy = 0; dy < CURSOR_CELLS; dy++)
        for (dx = 0; dx < CURSOR_CELLS; dx++)
            put_cell_rgb(gx + dx, gy + dy, r, g, b);
}

static void restore_block(uint8_t st[GRID][GRID], int gx, int gy) {
    int dx, dy;
    for (dy = 0; dy < CURSOR_CELLS; dy++)
        for (dx = 0; dx < CURSOR_CELLS; dx++) {
            int cgx = gx + dx, cgy = gy + dy;
            if (cgx < GRID && cgy < GRID)
                put_cell(cgx, cgy, st[cgy][cgx] ? 1 : 0);
        }
}

static void paint_block(uint8_t st[GRID][GRID], int gx, int gy) {
    int dx, dy;
    for (dy = 0; dy < CURSOR_CELLS; dy++)
        for (dx = 0; dx < CURSOR_CELLS; dx++) {
            int cgx = gx + dx, cgy = gy + dy;
            if (cgx < GRID && cgy < GRID && !st[cgy][cgx]) {
                st[cgy][cgx] = 255;
                put_cell(cgx, cgy, 1); 
            }
        }
}

static void flush_mouse(int fd) {
    int fl = fcntl(fd, F_GETFL, 0);
    unsigned char t[64];
    fcntl(fd, F_SETFL, fl | O_NONBLOCK);
    while (read(fd, t, sizeof(t)) > 0) { }
    fcntl(fd, F_SETFL, fl);
}

/* ===================== Modo de desenho ===================== */
static int draw_mode(uint8_t out[GRID * GRID]) {
    static uint8_t state[GRID][GRID];
    unsigned char p[3];
    int fd, cx, cy, gx, gy, ogx, ogy, i, j;
    int was_left = 0;
    const int gmax = GRID - CURSOR_CELLS;

    /* Configura o terminal para leitura instantanea do teclado (tecla Enter) */
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    fd = open("/dev/input/mice", O_RDONLY);
    if (fd < 0) { 
        perror("open /dev/input/mice"); 
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt); /* restaura terminal se der erro */
        return -1; 
    }

    flush_mouse(fd);

    memset(state, 0, sizeof(state));
    vga_clear(0, 0, 0);                 

    cx = CANVAS / 2; cy = CANVAS / 2;
    gx = cx / CELL; gy = cy / CELL;
    if (gx > gmax) gx = gmax;
    if (gy > gmax) gy = gmax;
    ogx = gx; ogy = gy;
    draw_cursor(gx, gy, 7, 7, 7);       

    printf("\n[MODO DESENHO]\n");
    printf("  ESQUERDO: pinta 8x8 | MEIO: apagar tudo | ENTER (teclado): confirmar e inferir\n");

    while (1) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd, &fds);            /* Escuta o mouse */
        FD_SET(STDIN_FILENO, &fds);  /* Escuta o teclado simultaneamente */

        /* Usa o select para bloquear sem gastar CPU ate que algo aconteca em um dos dois */
        if (select(fd + 1, &fds, NULL, NULL, NULL) < 0) break;

        /* Verifica se a interacao veio do teclado */
        if (FD_ISSET(STDIN_FILENO, &fds)) {
            char ch;
            if (read(STDIN_FILENO, &ch, 1) == 1) {
                if (ch == '\n' || ch == '\r') {
                    printf("\n"); /* quebra a linha visualmente no console */
                    break;        /* Sai do loop e confirma a inferencia */
                }
            }
        }

        /* Verifica se a interacao veio do mouse */
        if (FD_ISSET(fd, &fds)) {
            if (read(fd, p, 3) != 3) continue;

            if (!(p[0] & 0x08)) { 
                unsigned char discard;
                read(fd, &discard, 1); 
                continue; 
            }

            int dx    = (signed char)p[1];
            int dy    = (signed char)p[2];
            int left  = p[0] & 0x1;
            int mid   = p[0] & 0x4;

            cx += dx; cy -= dy;
            if (cx < 0) cx = 0;
            if (cx >= CANVAS) cx = CANVAS - 1;
            if (cy < 0) cy = 0;
            if (cy >= CANVAS) cy = CANVAS - 1;
            gx = cx / CELL; gy = cy / CELL;
            if (gx > gmax) gx = gmax;
            if (gy > gmax) gy = gmax;

            if (mid) {
                memset(state, 0, sizeof(state));
                vga_fill_rect(OFF_X, OFF_Y, CANVAS, CANVAS, 0, 0, 0);
                ogx = gx; ogy = gy;
                draw_cursor(gx, gy, 7, 7, 7);
                continue;
            }

            if (left) {
                if (!was_left) {
                    paint_block(state, gx, gy);
                } else {
                    int x0 = ogx, y0 = ogy;
                    int x1 = gx, y1 = gy;
                    int dx_line = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
                    int dy_line = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
                    int err = dx_line + dy_line, e2;

                    while (1) {
                        paint_block(state, x0, y0);
                        if (x0 == x1 && y0 == y1) break;
                        e2 = 2 * err;
                        if (e2 >= dy_line) { err += dy_line; x0 += sx; }
                        if (e2 <= dx_line) { err += dx_line; y0 += sy; }
                    }
                }
            }
            was_left = left;

            if (gx != ogx || gy != ogy) {
                restore_block(state, ogx, ogy);
                ogx = gx; ogy = gy;
            }

            draw_cursor(gx, gy, 7, 7, 7);
        }
    }
    close(fd);
    
    /* Restaura as definicoes originais do terminal antes de sair */
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    for (j = 0; j < GRID; j++)
        for (i = 0; i < GRID; i++)
            out[j * GRID + i] = state[j][i];

    return 0;
}

/* ===================== Integracao com o coprocessador ===================== */

static long ler_bin(const char *caminho, void *dst, long max_bytes) {
    FILE *f = fopen(caminho, "rb");
    long n;
    if (!f) { perror(caminho); return -1; }
    n = (long)fread(dst, 1, max_bytes, f);
    fclose(f);
    return n;
}

static int carregar_modelo(void) {
    static uint16_t w[ELM_N_WEIGHTS];
    static uint16_t bt[ELM_N_BETA];
    static uint16_t bs[ELM_N_BIAS];

    if (ler_bin(g_path_w,  w,  sizeof(w))  != (long)sizeof(w))  return -1;
    if (ler_bin(g_path_bt, bt, sizeof(bt)) != (long)sizeof(bt)) return -2;
    if (ler_bin(g_path_bs, bs, sizeof(bs)) != (long)sizeof(bs)) return -3;

    carregar_pesos(w);
    carregar_beta(bt);
    carregar_bias(bs);
    return 0;
}

static int carregar_png_28(const char *caminho, uint8_t out[GRID * GRID]) {
    int w, h, n, x, y;
    unsigned char *data = stbi_load(caminho, &w, &h, &n, 1);
    if (!data) {
        fprintf(stderr, "[-] Erro ao abrir PNG '%s': %s\n",
                caminho, stbi_failure_reason());
        return -1;
    }
    if (w == GRID && h == GRID) {
        memcpy(out, data, GRID * GRID);
    } else {
        for (y = 0; y < GRID; y++)
            for (x = 0; x < GRID; x++) {
                int sx = x * w / GRID;
                int sy = y * h / GRID;
                out[y * GRID + x] = data[sy * w + sx];
            }
    }
    stbi_image_free(data);
    return 0;
}

static int inferir(const uint8_t img[ELM_N_IMG]) {
    int r;
    if (carregar_imagem(img) != ELM_OK) {
        printf("[-] Falha ao enviar a imagem ao coprocessador.\n");
        return -1;
    }
    iniciar_inferencia();
    r = obter_resultado();
    reiniciar_fpga();
    return r;
}

/* ===================== Interface de texto ===================== */

static void read_line(char *dst, size_t n) {
    size_t L;
    if (!fgets(dst, (int)n, stdin)) { dst[0] = '\0'; return; }
    L = strlen(dst);
    while (L && (dst[L-1] == '\n' || dst[L-1] == '\r')) dst[--L] = '\0';
}

static int inferir_arquivo(const char *caminho) {
    uint8_t img[ELM_N_IMG];
    if (carregar_png_28(caminho, img) != 0) return -1;
    vga_show_image28(img); 
    printf(">> DIGITO PREDITO: %d\n", inferir(img));
    return 0;
}

/* ---- Comandos do menu ---- */

static void enviar_imagem_arquivo(void) {
    char caminho[256];
    printf("Caminho da imagem PNG: ");
    read_line(caminho, sizeof(caminho));
    if (!caminho[0]) { printf("[-] Caminho vazio.\n"); return; }
    inferir_arquivo(caminho);
}

static void pedir_caminho_base(const char *rotulo, char *base, size_t n) {
    char linha[256];
    printf("Caminho do %s (.bin) [%s]: ", rotulo, base);
    read_line(linha, sizeof(linha));
    if (linha[0]) { strncpy(base, linha, n - 1); base[n-1] = '\0'; }
}

static void enviar_pesos(void) {
    static uint16_t w[ELM_N_WEIGHTS];
    pedir_caminho_base("pesos", g_path_w, sizeof(g_path_w));
    if (ler_bin(g_path_w, w, sizeof(w)) != (long)sizeof(w)) {
        printf("[-] Tamanho invalido (esperado %lu bytes).\n", (unsigned long)sizeof(w));
        return;
    }
    carregar_pesos(w);
    printf("[+] Pesos enviados (%s).\n", g_path_w);
}

static void enviar_bias(void) {
    static uint16_t bs[ELM_N_BIAS];
    pedir_caminho_base("bias", g_path_bs, sizeof(g_path_bs));
    if (ler_bin(g_path_bs, bs, sizeof(bs)) != (long)sizeof(bs)) {
        printf("[-] Tamanho invalido (esperado %lu bytes).\n", (unsigned long)sizeof(bs));
        return;
    }
    carregar_bias(bs);
    printf("[+] Bias enviado (%s).\n", g_path_bs);
}

static void enviar_beta(void) {
    static uint16_t bt[ELM_N_BETA];
    pedir_caminho_base("beta", g_path_bt, sizeof(g_path_bt));
    if (ler_bin(g_path_bt, bt, sizeof(bt)) != (long)sizeof(bt)) {
        printf("[-] Tamanho invalido (esperado %lu bytes).\n", (unsigned long)sizeof(bt));
        return;
    }
    carregar_beta(bt);
    printf("[+] Beta enviado (%s).\n", g_path_bt);
}

static void cmd_reset(void) {
    reiniciar_fpga();
    printf("[+] FPGA resetada.\n");
}

static void salvar_desenho_png(const uint8_t img[GRID * GRID]) {
    static int seq = 0;
    char caminho[300];
    mkdir("minhas_imagens", 0777);
    snprintf(caminho, sizeof(caminho), "minhas_imagens/desenho_%ld_%d.png",
             (long)time(NULL), seq++);
    if (stbi_write_png(caminho, GRID, GRID, 1, img, GRID))
        printf("[+] Desenho salvo em %s\n", caminho);
    else
        printf("[-] Falha ao salvar o PNG do desenho.\n");
}

static void suavizar_desenho(uint8_t img[GRID * GRID]) {
    uint8_t tmp[GRID * GRID];
    int x, y, dx, dy, pico = 0;
    for (y = 0; y < GRID; y++)
        for (x = 0; x < GRID; x++) {
            int soma = 0, n = 0;
            for (dy = -1; dy <= 1; dy++)
                for (dx = -1; dx <= 1; dx++) {
                    int nx = x + dx, ny = y + dy;
                    if (nx >= 0 && nx < GRID && ny >= 0 && ny < GRID) {
                        soma += img[ny * GRID + nx];
                        n++;
                    }
                }
            tmp[y * GRID + x] = (uint8_t)(soma / n);
            if (tmp[y * GRID + x] > pico) pico = tmp[y * GRID + x];
        }
    if (pico > 0)
        for (x = 0; x < GRID * GRID; x++) {
            int v = tmp[x] * 255 / pico;
            img[x] = (uint8_t)(v > 255 ? 255 : v);
        }
}

static void modo_desenho(void) {
    uint8_t img[ELM_N_IMG];
    if (draw_mode(img) != 0) {
        printf("[-] Mouse indisponivel (/dev/input/mice). Rode como root.\n");
        return;
    }
    
    salvar_desenho_png(img);           
    
    /* * O desenho seco e fluido fica exatamente do jeito que foi feito no monitor VGA. 
     * Agora o blur é executado e processado APENAS DENTRO da memoria RAM, 
     * sem nunca ser enviado ao monitor novamente. 
     */
    suavizar_desenho(img);             
    
    /* A variavel tratada é despachada diretamente aos buffers da FPGA */
    printf(">> DIGITO PREDITO: %d\n", inferir(img));
}

static int eh_png(const char *nome) {
    size_t L = strlen(nome);
    if (L < 4) return 0;
    return (nome[L-4] == '.'
         && (nome[L-3]=='p' || nome[L-3]=='P')
         && (nome[L-2]=='n' || nome[L-2]=='N')
         && (nome[L-1]=='g' || nome[L-1]=='G'));
}

static void modo_benchmark(void) {
    char pasta[256];
    char full[600];
    DIR *d;
    struct dirent *de;
    FILE *csv;
    uint8_t img[ELM_N_IMG];
    int total = 0, acertos = 0, ninf = 0;
    int esperado = -1;
    double soma = 0.0, soma2 = 0.0;
    const char *csvnome = "benchmark.csv";

    printf("Pasta com PNGs de teste (o nome da pasta e o rotulo, ex.: 5): ");
    read_line(pasta, sizeof(pasta));
    if (!pasta[0]) return;

    {
        const char *base = pasta, *s;
        for (s = pasta; *s; s++)
            if (*s == '/') base = s + 1;
        for (s = base; *s; s++)
            if (*s >= '0' && *s <= '9') { esperado = *s - '0'; break; }
    }
    if (esperado < 0)
        printf("[!] Nao achei digito no nome da pasta; acuracia nao sera medida.\n");

    d = opendir(pasta);
    if (!d) { perror(pasta); return; }

    csv = fopen(csvnome, "w");
    if (!csv) { perror(csvnome); closedir(d); return; }
    fprintf(csv, "arquivo,esperado,predito,correto,latencia_ms\n");

    while ((de = readdir(d)) != NULL) {
        const char *nome = de->d_name;
        struct timespec t0, t1;
        double ms;
        int pred, correto;

        if (!eh_png(nome)) continue;

        snprintf(full, sizeof(full), "%s/%s", pasta, nome);
        if (carregar_png_28(full, img) != 0) continue;

        clock_gettime(CLOCK_MONOTONIC, &t0);
        pred = inferir(img);
        clock_gettime(CLOCK_MONOTONIC, &t1);
        ms = (t1.tv_sec - t0.tv_sec) * 1000.0
           + (t1.tv_nsec - t0.tv_nsec) / 1.0e6;

        correto = (esperado >= 0 && pred == esperado) ? 1 : 0;
        if (esperado >= 0) { total++; acertos += correto; }
        ninf++;
        soma  += ms;
        soma2 += ms * ms;

        fprintf(csv, "%s,%d,%d,%d,%.3f\n", nome, esperado, pred, correto, ms);
        printf("  %-20s esperado=%d predito=%d %s (%.2f ms)\n",
               nome, esperado, pred, correto ? "OK" : "X", ms);
    }
    closedir(d);

    if (ninf == 0) {
        printf("[bench] Nenhum PNG encontrado em '%s'.\n", pasta);
        fclose(csv);
        return;
    }

    {
        double media = soma / ninf;
        double var   = soma2 / ninf - media * media;
        double desvio = (var > 0.0) ? sqrt(var) : 0.0;
        double acc    = (total > 0) ? (100.0 * acertos / total) : 0.0;
        double thr    = (soma > 0.0) ? (1000.0 * ninf / soma) : 0.0;

        fprintf(csv, "\n");
        fprintf(csv, "# imagens,%d\n", ninf);
        fprintf(csv, "# com_rotulo,%d\n", total);
        fprintf(csv, "# acertos,%d\n", acertos);
        fprintf(csv, "# acuracia_pct,%.2f\n", acc);
        fprintf(csv, "# latencia_media_ms,%.3f\n", media);
        fprintf(csv, "# latencia_desvio_ms,%.3f\n", desvio);
        fprintf(csv, "# throughput_img_s,%.2f\n", thr);
        fclose(csv);

        printf("\n========== RESUMO ==========\n");
        printf("  Imagens inferidas : %d\n", ninf);
        printf("  Com rotulo        : %d\n", total);
        printf("  Acertos           : %d\n", acertos);
        printf("  Acuracia          : %.2f%%\n", acc);
        printf("  Latencia media    : %.3f ms\n", media);
        printf("  Latencia desvio   : %.3f ms\n", desvio);
        printf("  Throughput        : %.2f img/s\n", thr);
        printf("  CSV gravado em     : %s\n", csvnome);
    }
}

static int menu(void) {
    printf("\n==================================================\n");
    printf("        CLASSIFICADOR ELM - DE1-SoC (Marco 3)\n");
    printf("==================================================\n");
    printf("  1) Enviar imagem (arquivo PNG)   [mostra, envia e infere]\n");
    printf("  2) Enviar pesos\n");
    printf("  3) Enviar bias\n");
    printf("  4) Enviar beta\n");
    printf("  5) Desenhar imagem (mouse)       [mostra, envia e infere]\n");
    printf("  6) Resetar FPGA\n");
    printf("  7) Benchmark / validacao\n");
    printf("  0) Sair\n");
    printf("--------------------------------------------------\n");
    printf("Opcao: ");
    {
        char l[32];
        int op;
        read_line(l, sizeof(l));
        if (sscanf(l, "%d", &op) != 1) return -1;
        return op;
    }
}

static void uso(const char *prog) {
    printf("Uso: %s [opcoes]\n", prog);
    printf("  -w <arquivo>   caminho dos pesos  (padrao: %s)\n", PATH_WEIGHTS);
    printf("  -b <arquivo>   caminho do beta    (padrao: %s)\n", PATH_BETA);
    printf("  -s <arquivo>   caminho do bias    (padrao: %s)\n", PATH_BIAS);
    printf("  -i <imagem>    infere essa imagem PNG e depois abre o menu\n");
    printf("  -h             mostra esta ajuda\n");
}

int main(int argc, char **argv) {
    int op, i;
    const char *img_cli = NULL;

    for (i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "-w") && i + 1 < argc) { strncpy(g_path_w,  argv[++i], sizeof(g_path_w) -1); }
        else if (!strcmp(argv[i], "-b") && i + 1 < argc) { strncpy(g_path_bt, argv[++i], sizeof(g_path_bt)-1); }
        else if (!strcmp(argv[i], "-s") && i + 1 < argc) { strncpy(g_path_bs, argv[++i], sizeof(g_path_bs)-1); }
        else if (!strcmp(argv[i], "-i") && i + 1 < argc) img_cli = argv[++i];
        else if (!strcmp(argv[i], "-h")) { uso(argv[0]); return 0; }
        else { printf("Argumento invalido: %s\n", argv[i]); uso(argv[0]); return 1; }
    }

    if (mapear_fpga() != ELM_OK) {
        printf("[-] Falha ao mapear /dev/mem (rode como root).\n");
        return 1;
    }
    if (vga_init() != 0) {
        printf("[-] Falha ao iniciar o IP VGA.\n");
        return 1;
    }

    reiniciar_fpga();
    printf("Parametros: pesos=%s | beta=%s | bias=%s\n", g_path_w, g_path_bt, g_path_bs);
    printf("Carregando parametros da rede (pesos/beta/bias)...\n");
    if (carregar_modelo() != 0)
        printf("[!] Aviso: nao foi possivel carregar o modelo. Verifique os\n"
               "    caminhos (-w/-b/-s). As inferencias sairao incorretas.\n");
    else
        printf("[+] Modelo carregado.\n");

    if (img_cli) inferir_arquivo(img_cli);

    do {
        op = menu();
        switch (op) {
            case 1: enviar_imagem_arquivo(); break;
            case 2: enviar_pesos();          break;
            case 3: enviar_bias();           break;
            case 4: enviar_beta();           break;
            case 5: modo_desenho();          break;
            case 6: cmd_reset();             break;
            case 7: modo_benchmark();        break;
            case 0: break;
            default: printf("Opcao invalida.\n");
        }
    } while (op != 0);

    vga_close();
    printf("Encerrado.\n");
    return 0;
}
