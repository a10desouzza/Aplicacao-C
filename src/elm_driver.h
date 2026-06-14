/*
 * elm_driver.h - Driver para controle do FPGA ELM (versao buffer-based / Marco 3)
 *
 * MUDANCA: os carregadores recebem PONTEIROS para os dados ja na memoria
 * (lidos de arquivo OU desenhados na tela pelo usuario). O driver nao abre
 * arquivos - quem le do disco e a aplicacao C.
 */
#ifndef ELM_DRIVER_H
#define ELM_DRIVER_H

#include <stdint.h>

/* --- Enderecos e Tamanhos de Memoria --- */
#define ELM_BRIDGE_PHYS      0xFF200000UL
#define ELM_BRIDGE_PAGE_SIZE 4096

/* --- Offsets dos Registradores do coprocessador --- */
#define ELM_REG_DATA_OUT  0x00
#define ELM_REG_SIGNALS   0x10
#define ELM_REG_DATA_IN   0x20

/* --- Mascaras do Registrador de Saida (DATA_OUT) --- */
#define ELM_BIT_DONE   (1 << 4)
#define ELM_BIT_BUSY   (1 << 5)
#define ELM_BIT_ERROR  (1 << 6)
#define ELM_MASK_DIGIT 0xF

/* --- Sinais de Controle (SIGNALS) --- */
#define ELM_SIG_ENABLE  (1 << 0)
#define ELM_SIG_CLR_OP  (1 << 1)
#define ELM_SIG_RESET   (1 << 2)

/* --- Codigos de Retorno --- */
#define ELM_OK              0
#define ELM_ERR_DEVMEM     -1
#define ELM_ERR_MMAP       -2
#define ELM_ERR_READ_IMG   -7
#define ELM_ERR_FPGA       -99

/* --- Tamanhos esperados dos buffers --- */
#define ELM_N_WEIGHTS  100352   /* halfwords fp16 */
#define ELM_N_BETA       1280   /* halfwords fp16 */
#define ELM_N_BIAS        128   /* halfwords fp16 */
#define ELM_N_IMG         784   /* bytes (28x28 pixels) */

/* --- Prototipos --- */

/* Mapeia /dev/mem (MMIO da ponte HPS-FPGA). 0 ok, -1/-2 erro. */
int mapear_fpga(void);

/* Pulso de reset na FSM. 0 ok, -1 se base nula. */
int reiniciar_fpga(void);

/* Envia dados JA na memoria (ponteiros vindos do C, sem acesso a arquivo).
 * Retornam 0 (ok) ou -99 (erro de hardware reportado pela FPGA). */
int carregar_pesos (const uint16_t *w);    /* ELM_N_WEIGHTS halfwords */
int carregar_beta  (const uint16_t *bt);   /* ELM_N_BETA    halfwords */
int carregar_bias  (const uint16_t *bs);   /* ELM_N_BIAS    halfwords */
int carregar_imagem(const uint8_t  *img);  /* ELM_N_IMG     bytes     */

/* Dispara a inferencia. */
int iniciar_inferencia(void);

/* Bloqueia ate DONE e retorna digito [0..9]. */
int obter_resultado(void);

/* Le valor bruto do status. */
int ler_status_fpga(void);

#endif