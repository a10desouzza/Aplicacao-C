/* ================================================================================================
 * main.c - Aplicacao de usuario do classificador ELM (versao buffer-based / Marco 3)
 * Autores: Pedro Henrique, Lucas Vilas Boas Dourado, Arthur Souza
 *
 * Aplicacao de alto nivel que roda no HPS (ARM + Linux) do SoC. Responsavel pela interface em
 * modo texto, pela exibicao no monitor (IP-Core VGA) e pela orquestracao das inferencias no
 * coprocessador ELM. Toda a comunicacao com a FPGA e feita por MMIO: os registradores do
 * hardware sao mapeados em memoria via /dev/mem + mmap e acessados como ponteiros.
 *
 * DIVISAO DE RESPONSABILIDADES (Marco 3): o C le os dados (de arquivo OU da imagem desenhada) e
 * passa um PONTEIRO ao driver Assembly, que envia ao coprocessador via MMIO sem abrir arquivos.
 * ================================================================================================ */

/* ===================== Bibliotecas ===================== */
#include <stdio.h>         /* I/O padrao e de arquivo: printf, fopen, fread, snprintf */
#include <stdlib.h>        /* utilitarios gerais (abs, etc.) */
#include <string.h>        /* memcpy, memset, strncpy, strlen, strcmp */
#include <stdint.h>        /* tipos de largura fixa (uint8_t/16/32) exigidos pelo hardware */
#include <unistd.h>        /* read, close, usleep */
#include <fcntl.h>         /* open e flags (O_RDWR, O_SYNC, O_NONBLOCK) */
#include <sys/mman.h>      /* mmap/munmap: base do acesso MMIO a ponte HPS-FPGA */
#include <sys/stat.h>      /* mkdir: criacao da pasta de desenhos */
#include <dirent.h>        /* opendir/readdir: varredura da pasta no benchmark */
#include <time.h>          /* clock_gettime: medicao de latencia */
#include <math.h>          /* sqrt: desvio padrao */
#include <termios.h>       /* terminal em modo cru (capturar Enter na hora) */
#include <sys/select.h>    /* select: escutar mouse e teclado simultaneamente */

#define STB_IMAGE_IMPLEMENTATION        /* stb_image: decodificacao de PNG (header-only) */
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION  /* stb_image_write: gravacao de PNG (header-only) */
#include "stb_image_write.h"

#include "elm_driver.h"   /* prototipos do driver Assembly (mapear_fpga, carregar_*, inferir...) */

/* ===== Caminhos dos parametros da rede (ajuste se necessario) ===== */
#define PATH_WEIGHTS "../data/w_in_q.bin"
#define PATH_BETA    "../data/beta_q.bin"
#define PATH_BIAS    "../data/bias_q.bin"

/* Caminhos-base mutaveis: padroes acima, sobrescritos por -w/-b/-s e pelos comandos de envio. */
static char g_path_w [256] = PATH_WEIGHTS;
static char g_path_bt[256] = PATH_BETA;
static char g_path_bs[256] = PATH_BIAS;

/* ===================== Geometria do VGA =====================
 * Imagem 28x28 ampliada: cada pixel vira um bloco 8x8 (CELL), formando 224x224 (CANVAS)
 * centralizado na tela de 320x240 (margens OFF_X/OFF_Y). */
#define VGA_W   320
#define VGA_H   240
#define GRID    28
#define CELL    8
#define CURSOR_CELLS 1
#define CANVAS  (GRID * CELL)
#define OFF_X   ((VGA_W - CANVAS) / 2)
#define OFF_Y   ((VGA_H - CANVAS) / 2)

/* ===================== PIOs do VGA na ponte lightweight =====================
 * Ponte HPS->FPGA em 0xFF200000. Tres PIOs com papeis/direcoes distintos: dados (pixel),
 * sinais de controle (enable/reset) e status (done). */
#define LW_BRIDGE_BASE   0xFF200000
#define LW_BRIDGE_SPAN   0x00001000   /* 4 KB (uma pagina) cobre todos os offsets */
#define VGA_STATUS_OFFSET  0x30       /* done   (FPGA->HPS) */
#define VGA_SIGNALS_OFFSET 0x40       /* enable/reset (HPS->FPGA) */
#define VGA_DATA_OFFSET    0x50       /* pixel  (HPS->FPGA) */

#define VGA_SIG_ENABLE (1u << 0)
#define VGA_SIG_RESET  (1u << 1)

/* IMPORTANTE: 'volatile' impede o compilador de otimizar os acessos ao hardware; sem ele,
 * o laco de espera do 'done' poderia virar laco infinito. */
static volatile uint32_t *g_data;     /* pixel  (0x50) */
static volatile uint32_t *g_signals;  /* ctrl   (0x40) */
static volatile uint32_t *g_status;   /* done   (0x30) */
static void *g_base = MAP_FAILED;
static int   g_fd   = -1;

/* ===================== Camada VGA ===================== */

/* pack_pixel: recebe coordenada (x,y) e cor (r,g,b); retorna a palavra de 32 bits no layout
 * que o ghrd_top.v espera: posy[28:19]|posx[18:9]|red[8:6]|green[5:3]|blue[2:0]. */
static uint32_t pack_pixel(int x, int y, int r, int g, int b) {
    return ((uint32_t)(y & 0x3FF) << 19)
         | ((uint32_t)(x & 0x3FF) << 9)
         | ((uint32_t)(r & 0x7) << 6)
         | ((uint32_t)(g & 0x7) << 3)
         | ((uint32_t)(b & 0x7));
}

/* vga_init: abre /dev/mem, mapeia a pagina da ponte e calcula os ponteiros dos tres PIOs.
 * Nao recebe parametros. Retorna 0 em sucesso, -1 (open) ou -2 (mmap) em erro.
 * Observacao: O_SYNC faz as escritas irem direto ao hardware, sem cache. */
static int vga_init(void) {
    g_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (g_fd < 0) { perror("open /dev/mem"); return -1; }
    g_base = mmap(NULL, LW_BRIDGE_SPAN, PROT_READ | PROT_WRITE,
                  MAP_SHARED, g_fd, LW_BRIDGE_BASE);
    if (g_base == MAP_FAILED) { perror("mmap"); close(g_fd); g_fd = -1; return -2; }
    g_status  = (volatile uint32_t *)((char *)g_base + VGA_STATUS_OFFSET);
    g_signals = (volatile uint32_t *)((char *)g_base + VGA_SIGNALS_OFFSET);
    g_data    = (volatile uint32_t *)((char *)g_base + VGA_DATA_OFFSET);
    *g_signals = VGA_SIG_RESET; usleep(1000);   /* pulso de reset no IP */
    *g_signals = 0;             usleep(1000);
    return 0;
}

/* vga_close: libera o mapeamento e fecha o descritor. Sem parametros, sem retorno. */
static void vga_close(void) {
    if (g_base != MAP_FAILED) munmap(g_base, LW_BRIDGE_SPAN);
    if (g_fd >= 0) close(g_fd);
    g_base = MAP_FAILED; g_fd = -1;
}

/* vga_set_pixel: recebe posicao e cor; sem retorno. PONTO-CHAVE do projeto: implementa o
 * handshake de 4 fases com a FSM do VGA -> 1) dado estavel; 2) sobe enable (gatilho);
 * 3) espera o done; 4) baixa enable. Coordenada fora da tela e ignorada. */
static void vga_set_pixel(int x, int y, int r, int g, int b) {
    uint32_t word;
    if (x < 0 || x >= VGA_W || y < 0 || y >= VGA_H) return;
    word = pack_pixel(x, y, r, g, b);
    *g_data    = word;
    *g_signals = VGA_SIG_ENABLE;
    while ((*g_status & 0x1) == 0) { }
    *g_signals = 0;
}

/* vga_fill_rect: recebe origem (x0,y0), tamanho (w,h) e cor; preenche o retangulo. Sem retorno. */
static void vga_fill_rect(int x0, int y0, int w, int h, int r, int g, int b) {
    int x, y;
    for (y = y0; y < y0 + h; y++)
        for (x = x0; x < x0 + w; x++)
            vga_set_pixel(x, y, r, g, b);
}

/* vga_clear: limpa a tela inteira com a cor dada. Sem retorno. */
static void vga_clear(int r, int g, int b) {
    vga_fill_rect(0, 0, VGA_W, VGA_H, r, g, b);
}

/* vga_show_image28: recebe a imagem 28x28 (784 bytes) e a exibe ampliada 8x e centralizada.
 * Sem retorno. O '>> 5' reduz cada pixel de 8 para 3 bits (resolucao de cor do IP). */
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

/* put_cell_rgb: pinta o bloco 8x8 de uma celula da grade com cor RGB. Sem retorno. */
static void put_cell_rgb(int gx, int gy, int r, int g, int b) {
    if (gx < 0 || gx >= GRID || gy < 0 || gy >= GRID) return;
    vga_fill_rect(OFF_X + gx * CELL, OFF_Y + gy * CELL, CELL, CELL, r, g, b);
}

/* put_cell: atalho preto/branco (on -> branco, senao preto). Sem retorno. */
static void put_cell(int gx, int gy, int on) {
    int v = on ? 7 : 0;
    put_cell_rgb(gx, gy, v, v, v);
}

/* draw_cursor: desenha o bloco do cursor na celula (gx,gy) com a cor dada. Sem retorno. */
static void draw_cursor(int gx, int gy, int r, int g, int b) {
    int dx, dy;
    for (dy = 0; dy < CURSOR_CELLS; dy++)
        for (dx = 0; dx < CURSOR_CELLS; dx++)
            put_cell_rgb(gx + dx, gy + dy, r, g, b);
}

/* restore_block: recebe o estado do desenho (st) e a celula; repinta-a conforme st (e nao a
 * tela). Sem retorno. IMPORTANTE: e o que apaga o rastro do cursor sem apagar o desenho. */
static void restore_block(uint8_t st[GRID][GRID], int gx, int gy) {
    int dx, dy;
    for (dy = 0; dy < CURSOR_CELLS; dy++)
        for (dx = 0; dx < CURSOR_CELLS; dx++) {
            int cgx = gx + dx, cgy = gy + dy;
            if (cgx < GRID && cgy < GRID)
                put_cell(cgx, cgy, st[cgy][cgx] ? 1 : 0);
        }
}

/* paint_block: marca como pintada (255) a celula sob o cursor no estado st, apenas se vazia.
 * Sem retorno. */
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

/* flush_mouse: recebe o fd do mouse e descarta os eventos pendentes (usa modo nao-bloqueante).
 * Sem retorno. Evita que cliques antigos vazem para um novo desenho. */
static void flush_mouse(int fd) {
    int fl = fcntl(fd, F_GETFL, 0);
    unsigned char t[64];
    fcntl(fd, F_SETFL, fl | O_NONBLOCK);
    while (read(fd, t, sizeof(t)) > 0) { }
    fcntl(fd, F_SETFL, fl);
}

/* ===================== Modo de desenho ===================== */

/* draw_mode: captura um digito desenhado com o mouse e o devolve em out[784]. Retorna 0 em
 * sucesso ou -1 se o mouse nao abrir. PONTOS IMPORTANTES: a matriz 'state' e a FONTE DA
 * VERDADE do desenho (a tela e so reflexo); o terminal vai a modo cru para capturar o Enter
 * (confirmacao); 'select' escuta mouse e teclado juntos; o mouse usa protocolo PS/2 (3 bytes)
 * e o eixo Y e invertido; o arrasto traca uma linha continua via algoritmo de Bresenham. */
static int draw_mode(uint8_t out[GRID * GRID]) {
    static uint8_t state[GRID][GRID];   /* static: fora da pilha, zerada por chamada */
    unsigned char p[3];
    int fd, cx, cy, gx, gy, ogx, ogy, i, j;
    int was_left = 0;
    const int gmax = GRID - CURSOR_CELLS;

    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);   /* ICANON/ECHO off: le a tecla na hora, sem eco */
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    fd = open("/dev/input/mice", O_RDONLY);
    if (fd < 0) {
        perror("open /dev/input/mice");
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);   /* restaura o terminal antes de sair */
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
        FD_SET(fd, &fds);
        FD_SET(STDIN_FILENO, &fds);
        if (select(fd + 1, &fds, NULL, NULL, NULL) < 0) break;

        /* Teclado: Enter confirma e encerra o desenho. */
        if (FD_ISSET(STDIN_FILENO, &fds)) {
            char ch;
            if (read(STDIN_FILENO, &ch, 1) == 1) {
                if (ch == '\n' || ch == '\r') { printf("\n"); break; }
            }
        }

        /* Mouse: movimento, pintura e apagar. */
        if (FD_ISSET(fd, &fds)) {
            if (read(fd, p, 3) != 3) continue;
            if (!(p[0] & 0x08)) {                  /* re-sincroniza pacote PS/2 desalinhado */
                unsigned char discard;
                read(fd, &discard, 1);
                continue;
            }

            int dx    = (signed char)p[1];          /* deslocamentos com sinal */
            int dy    = (signed char)p[2];
            int left  = p[0] & 0x1;
            int mid   = p[0] & 0x4;

            cx += dx; cy -= dy;                      /* cy -= dy: inverte o eixo Y */
            if (cx < 0) cx = 0;
            if (cx >= CANVAS) cx = CANVAS - 1;
            if (cy < 0) cy = 0;
            if (cy >= CANVAS) cy = CANVAS - 1;
            gx = cx / CELL; gy = cy / CELL;
            if (gx > gmax) gx = gmax;
            if (gy > gmax) gy = gmax;

            if (mid) {                              /* botao do meio: apaga tudo */
                memset(state, 0, sizeof(state));
                vga_fill_rect(OFF_X, OFF_Y, CANVAS, CANVAS, 0, 0, 0);
                ogx = gx; ogy = gy;
                draw_cursor(gx, gy, 7, 7, 7);
                continue;
            }

            if (left) {                             /* botao esquerdo: pinta */
                if (!was_left) {
                    paint_block(state, gx, gy);     /* clique unico: um ponto */
                } else {                            /* arrasto: linha de Bresenham */
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

            if (gx != ogx || gy != ogy) {           /* apaga o rastro do cursor anterior */
                restore_block(state, ogx, ogy);
                ogx = gx; ogy = gy;
            }
            draw_cursor(gx, gy, 7, 7, 7);
        }
    }
    close(fd);
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);        /* restaura o terminal original */

    /* Converte a matriz 28x28 para o vetor linear out[784] (ordem linha*28+coluna). */
    for (j = 0; j < GRID; j++)
        for (i = 0; i < GRID; i++)
            out[j * GRID + i] = state[j][i];

    return 0;
}

/* ===================== Integracao com o coprocessador ===================== */

/* ler_bin: recebe caminho, buffer destino e limite de bytes; le o arquivo binario para o
 * buffer. Retorna o numero de bytes lidos, ou -1 se nao abrir. (Quem le o disco e o C.) */
static long ler_bin(const char *caminho, void *dst, long max_bytes) {
    FILE *f = fopen(caminho, "rb");
    long n;
    if (!f) { perror(caminho); return -1; }
    n = (long)fread(dst, 1, max_bytes, f);
    fclose(f);
    return n;
}

/* carregar_modelo: le pesos/beta/bias dos caminhos globais e os envia ao coprocessador.
 * Sem parametros. Retorna 0 em sucesso ou -1/-2/-3 se algum arquivo tiver tamanho invalido.
 * Os buffers sao 'static' por serem grandes (pesos ~200 KB) e nao caberem bem na pilha. */
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

/* carregar_png_28: recebe um caminho de PNG e o buffer de saida (784 bytes); carrega a imagem
 * em tom de cinza e garante 28x28 (redimensiona por vizinho mais proximo se necessario).
 * Retorna 0 em sucesso ou -1 se o PNG nao abrir. */
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

/* inferir: recebe a imagem (784 bytes) e executa o pipeline completo de classificacao
 * (envia -> START -> espera o resultado -> reseta). Retorna o digito predito (0..9) ou -1 em
 * falha de envio. O reset ao final limpa o estado de operacao para a proxima inferencia. */
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

/* read_line: recebe um buffer e seu tamanho; le uma linha do teclado sem o '\n'. Sem retorno.
 * Linha vazia -> string vazia, o que permite "Enter mantem o valor atual" nos caminhos. */
static void read_line(char *dst, size_t n) {
    size_t L;
    if (!fgets(dst, (int)n, stdin)) { dst[0] = '\0'; return; }
    L = strlen(dst);
    while (L && (dst[L-1] == '\n' || dst[L-1] == '\r')) dst[--L] = '\0';
}

/* inferir_arquivo: recebe um caminho de PNG; carrega, exibe no monitor, infere e imprime o
 * digito. Retorna 0 em sucesso ou -1 se o PNG nao carregar. */
static int inferir_arquivo(const char *caminho) {
    uint8_t img[ELM_N_IMG];
    if (carregar_png_28(caminho, img) != 0) return -1;
    vga_show_image28(img);
    printf(">> DIGITO PREDITO: %d\n", inferir(img));
    return 0;
}

/* enviar_imagem_arquivo (opcao 1): pede o caminho ao usuario, mostra, envia e infere a imagem.
 * Sem parametros, sem retorno. */
static void enviar_imagem_arquivo(void) {
    char caminho[256];
    printf("Caminho da imagem PNG: ");
    read_line(caminho, sizeof(caminho));
    if (!caminho[0]) { printf("[-] Caminho vazio.\n"); return; }
    inferir_arquivo(caminho);
}

/* pedir_caminho_base: recebe um rotulo, o buffer-base e seu tamanho; mostra o caminho atual e
 * so o substitui se o usuario digitar algo (Enter mantem). Sem retorno. */
static void pedir_caminho_base(const char *rotulo, char *base, size_t n) {
    char linha[256];
    printf("Caminho do %s (.bin) [%s]: ", rotulo, base);
    read_line(linha, sizeof(linha));
    if (linha[0]) { strncpy(base, linha, n - 1); base[n-1] = '\0'; }
}

/* enviar_pesos (opcao 2): pede/mantem o caminho, valida o tamanho e envia os pesos. Sem retorno. */
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

/* enviar_bias (opcao 3): analogo a enviar_pesos, para o bias. Sem retorno. */
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

/* enviar_beta (opcao 4): analogo a enviar_pesos, para o beta. Sem retorno. */
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

/* cmd_reset (opcao 6): reseta a FPGA manualmente. Sem parametros, sem retorno. */
static void cmd_reset(void) {
    reiniciar_fpga();
    printf("[+] FPGA resetada.\n");
}

/* salvar_desenho_png: recebe a imagem 28x28 e a grava como PNG na pasta minhas_imagens, com
 * nome unico (timestamp + contador). Sem retorno. */
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

/* modo_desenho (opcao 5): chama o modo de desenho, salva o tracado, envia e infere. Sem
 * parametros, sem retorno. */
static void modo_desenho(void) {
    uint8_t img[ELM_N_IMG];
    if (draw_mode(img) != 0) {
        printf("[-] Mouse indisponivel (/dev/input/mice). Rode como root.\n");
        return;
    }
    salvar_desenho_png(img);
    printf(">> DIGITO PREDITO: %d\n", inferir(img));
}

/* eh_png: recebe um nome de arquivo; retorna 1 se terminar em ".png" (qualquer caixa), senao 0. */
static int eh_png(const char *nome) {
    size_t L = strlen(nome);
    if (L < 4) return 0;
    return (nome[L-4] == '.'
         && (nome[L-3]=='p' || nome[L-3]=='P')
         && (nome[L-2]=='n' || nome[L-2]=='N')
         && (nome[L-1]=='g' || nome[L-1]=='G'));
}

/* modo_benchmark (opcao 7): pede uma pasta (cujo nome e o rotulo esperado), infere cada PNG
 * medindo a latencia e, ao final, calcula acuracia, latencia media, desvio e throughput,
 * gravando tudo em benchmark.csv. Sem parametros, sem retorno.
 * PONTOS IMPORTANTES: usa CLOCK_MONOTONIC (relogio que so avanca) para a latencia, e acumula
 * soma e soma dos quadrados para obter media e desvio numa unica passada. */
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

    /* Rotulo esperado = primeiro digito do ultimo componente do caminho (nome da pasta). */
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

    /* Estatisticas: variancia = E[x^2] - E[x]^2; throughput = 1000*ninf/soma_ms. */
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

/* menu: imprime as opcoes e le a escolha do usuario. Sem parametros. Retorna o numero da
 * opcao, ou -1 se a entrada nao for numerica. */
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

/* uso: recebe o nome do programa e imprime a ajuda das opcoes de linha de comando. Sem retorno. */
static void uso(const char *prog) {
    printf("Uso: %s [opcoes]\n", prog);
    printf("  -w <arquivo>   caminho dos pesos  (padrao: %s)\n", PATH_WEIGHTS);
    printf("  -b <arquivo>   caminho do beta    (padrao: %s)\n", PATH_BETA);
    printf("  -s <arquivo>   caminho do bias    (padrao: %s)\n", PATH_BIAS);
    printf("  -i <imagem>    infere essa imagem PNG e depois abre o menu\n");
    printf("  -h             mostra esta ajuda\n");
}

/* main: ponto de entrada. Faz o parsing dos argumentos (-w/-b/-s/-i/-h), inicializa o MMIO do
 * coprocessador e do VGA, carrega o modelo e entra no laco do menu. Retorna 0 em saida normal
 * ou 1 em erro de inicializacao/argumento. */
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
