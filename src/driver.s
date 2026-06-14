@================================================================================================
@ driver.s - Driver para controle do FPGA ELM (versao buffer-based)
@ Autores: Pedro Henrique, Lucas Vilas Boas Dourado, Arthur Souza
@
@ MUDANCA (Marco 3): os carregadores NAO abrem mais arquivos. A aplicacao C le
@ os dados (de arquivo OU de uma imagem desenhada na tela) e passa um PONTEIRO
@ para o buffer ja na memoria. O driver apenas copia/envia esses dados ao
@ coprocessador via MMIO. Assim o modo "desenhar com o mouse" (que nao tem
@ arquivo) usa exatamente o mesmo caminho.
@
@ Novas assinaturas (ver elm_driver.h):
@   int carregar_pesos (const unsigned short *w);   // 100352 halfwords fp16
@   int carregar_beta  (const unsigned short *bt);  //   1280 halfwords fp16
@   int carregar_bias  (const unsigned short *bs);  //    128 halfwords fp16
@   int carregar_imagem(const unsigned char  *img); //    784 bytes (pixels)
@ ================================================================================================
.global mapear_fpga
.type mapear_fpga, %function
.global reiniciar_fpga
.type reiniciar_fpga, %function
.global carregar_pesos
.type carregar_pesos, %function
.global carregar_beta
.type carregar_beta, %function
.global carregar_bias
.type carregar_bias, %function
.global carregar_imagem
.type carregar_imagem, %function
.global iniciar_inferencia
.type iniciar_inferencia, %function
.global obter_resultado
.type obter_resultado, %function
.global ler_status_fpga
.type ler_status_fpga, %function

.section .data
dev_mem:  .asciz "/dev/mem"          @ caminho do /dev/mem (necessario apenas para o MMIO)

.section .bss
.balign 4
base:    .space 4                    @ endereco virtual mapeado da ponte HPS-FPGA

.section .text

@ ------------------------------------------------------------------------------
@ Abre /dev/mem e mapeia a regiao de perifericos da FPGA via mmap2.
@ (Esta funcao continua usando /dev/mem porque e o acesso ao MMIO, nao a dados.)
@ ------------------------------------------------------------------------------
mapear_fpga:
    push {r4, r5, r7, lr}

    mov  r7, #5                      @ syscall open
    ldr  r0, =dev_mem
    ldr  r1, =0x101002               @ O_RDWR | O_SYNC
    mov  r2, #0
    svc  0
    cmp  r0, #0
    bge  mem_ok
    mov  r0, #-1
    pop  {r4, r5, r7, pc}

mem_ok:
    mov  r4, r0                      @ salva fd

    mov  r7, #192                    @ syscall mmap2
    mov  r0, #0
    mov  r1, #4096                   @ 1 pagina
    mov  r2, #3                      @ PROT_READ | PROT_WRITE
    mov  r3, #1                      @ MAP_SHARED
    ldr  r5, =0xFF200000
    lsr  r5, r5, #12
    svc  0

    cmn  r0, #4096
    bcc  mmap_ok
    mov  r0, #-2
    pop  {r4, r5, r7, pc}

mmap_ok:
    ldr  r1, =base
    str  r0, [r1]
    mov  r0, #0
    pop  {r4, r5, r7, pc}

@ ------------------------------------------------------------------------------
@ Envia pulso de reset ao coprocessador.
@ ------------------------------------------------------------------------------
reiniciar_fpga:
    push {r9, lr}
    ldr  r9, =base
    ldr  r9, [r9]
    cmp  r9, #0
    beq  rst_err

    mov  r3, #0x4                    @ sobe reset (bit 2 do SIGNALS)
    str  r3, [r9, #0x10]
    mov  r3, #0                      @ desce reset
    str  r3, [r9, #0x10]

    mov  r0, #0
    pop  {r9, pc}
rst_err:
    mov  r0, #-1
    pop  {r9, pc}

@ ------------------------------------------------------------------------------
@ carregar_pesos(const unsigned short *w)
@   r0 = ponteiro para 100352 halfwords (fp16). Envia em pares (endereco, valor).
@ ------------------------------------------------------------------------------
carregar_pesos:
    push {r4, r5, r9-r11, lr}
    mov  r4, r0                      @ r4 = ponteiro dos dados (vindo do C)
    ldr  r9, =base
    ldr  r9, [r9]

    mov  r10, #0
    ldr  r11, =100352

loop_w:
    cmp  r10, r11
    bhs  w_done

    lsl  r5, r10, #1                 @ offset = indice * 2 bytes
    ldrh r0, [r4, r5]                @ le halfword do buffer do C
    rev16 r0, r0                     @ corrige endianness do fp16

    mov  r1, r10
    bl   fmt_w_addr                  @ instrucao de endereco (opcode 1)
    bl   send_cmd

    bl   fmt_w_val                   @ instrucao de valor (opcode 2)
    bl   send_cmd
    bl   chk_err
    cmp  r0, #0
    blt  end_w

    add  r10, r10, #1
    b    loop_w
w_done:
    mov  r0, #0
end_w:
    pop  {r4, r5, r9-r11, pc}

@ ------------------------------------------------------------------------------
@ carregar_beta(const unsigned short *bt)  -> 1280 halfwords, opcode 4
@ ------------------------------------------------------------------------------
carregar_beta:
    push {r4, r5, r9-r11, lr}
    mov  r4, r0
    ldr  r9, =base
    ldr  r9, [r9]

    mov  r10, #0
    ldr  r11, =1280

loop_bt:
    cmp  r10, r11
    bhs  bt_done

    lsl  r5, r10, #1
    ldrh r0, [r4, r5]
    rev16 r0, r0

    mov  r1, r10
    bl   fmt_bt
    bl   send_cmd
    bl   chk_err
    cmp  r0, #0
    blt  end_bt

    add  r10, r10, #1
    b    loop_bt
bt_done:
    mov  r0, #0
end_bt:
    pop  {r4, r5, r9-r11, pc}

@ ------------------------------------------------------------------------------
@ carregar_bias(const unsigned short *bs)  -> 128 halfwords, opcode 3
@ ------------------------------------------------------------------------------
carregar_bias:
    push {r4, r5, r9-r11, lr}
    mov  r4, r0
    ldr  r9, =base
    ldr  r9, [r9]

    mov  r10, #0
    ldr  r11, =128

loop_bs:
    cmp  r10, r11
    bhs  bs_done

    lsl  r5, r10, #1
    ldrh r0, [r4, r5]
    rev16 r0, r0

    mov  r1, r10
    bl   fmt_bs
    bl   send_cmd
    bl   chk_err
    cmp  r0, #0
    blt  end_bs

    add  r10, r10, #1
    b    loop_bs
bs_done:
    mov  r0, #0
end_bs:
    pop  {r4, r5, r9-r11, pc}

@ ------------------------------------------------------------------------------
@ carregar_imagem(const unsigned char *img)  -> 784 bytes, opcode 0
@   r0 = ponteiro para 784 pixels (de arquivo OU desenhados na tela).
@ ------------------------------------------------------------------------------
carregar_imagem:
    push {r4, r9-r11, lr}
    mov  r4, r0                      @ r4 = ponteiro da imagem (vindo do C)
    ldr  r9, =base
    ldr  r9, [r9]

    cmp  r4, #0                      @ ponteiro nulo?
    beq  img_nul

    mov  r10, #0
    ldr  r11, =784

loop_img:
    cmp  r10, r11
    bhs  img_done

    ldrb r0, [r4, r10]              @ le 1 byte (pixel) do buffer do C
    mov  r1, r10
    bl   fmt_img                     @ formato de imagem (opcode 0)
    bl   send_cmd
    bl   chk_err
    cmp  r0, #0
    blt  end_img

    add  r10, r10, #1
    b    loop_img
img_done:
    mov  r0, #0
    pop  {r4, r9-r11, pc}
img_nul:
    mov  r0, #-7                     @ erro: ponteiro nulo
end_img:
    pop  {r4, r9-r11, pc}

@ ------------------------------------------------------------------------------
@ Escreve opcode 5 e envia pulso de start para disparar a inferencia.
@ ------------------------------------------------------------------------------
iniciar_inferencia:
    push {r9, lr}
    ldr  r9, =base
    ldr  r9, [r9]

    @ limpa o status anterior (DONE/ERROR) ANTES de comecar, para que o
    @ obter_resultado espere o DONE desta imagem, e nao o da anterior.
    mov  r3, #2                     @ CLR_OP (bit 1)
    str  r3, [r9, #0x10]
    mov  r3, #0
    str  r3, [r9, #0x10]

    mov  r2, #5                     @ opcode START
    str  r2, [r9, #0x20]
    mov  r3, #1                     @ pulso de enable (bit 0)
    str  r3, [r9, #0x10]
    mov  r3, #0
    str  r3, [r9, #0x10]

    mov  r0, #0
    pop  {r9, pc}

@ ------------------------------------------------------------------------------
@ Aguarda bit 4 (done) desta inferencia, le o digito [3:0] e limpa o DONE
@ para que a proxima inferencia comece com o status zerado.
@ ------------------------------------------------------------------------------
obter_resultado:
    push {r4, r9, lr}
    ldr  r9, =base
    ldr  r9, [r9]
poll_done:
    ldr  r0, [r9, #0x00]
    tst  r0, #0x10
    beq  poll_done
    and  r4, r0, #0xF               @ guarda o digito predito

    mov  r3, #2                     @ CLR_OP: limpa o DONE apos a leitura
    str  r3, [r9, #0x10]
    mov  r3, #0
    str  r3, [r9, #0x10]

    mov  r0, r4
    pop  {r4, r9, pc}

@ ------------------------------------------------------------------------------
@ Retorna o valor bruto do registrador de status.
@ ------------------------------------------------------------------------------
ler_status_fpga:
    push {r3, lr}
    ldr  r3, =base
    ldr  r3, [r3]
    ldr  r0, [r3, #0x00]
    pop  {r3, pc}

@ ============================ formatadores ============================
@ Empacota pixel (8b) e indice em r2; opcode 0 implicito.
fmt_img:
    lsl  r2, r0, #13
    lsl  r3, r1, #3
    orr  r2, r2, r3
    bx   lr

fmt_w_addr:
    lsl  r2, r1, #3
    orr  r2, r2, #1
    bx   lr

fmt_w_val:
    lsl  r2, r0, #3
    orr  r2, r2, #2
    bx   lr

fmt_bs:
    lsl  r2, r0, #10
    lsl  r3, r1, #3
    orr  r2, r2, r3
    orr  r2, r2, #3
    bx   lr

fmt_bt:
    lsl  r2, r0, #14
    lsl  r3, r1, #3
    orr  r2, r2, r3
    orr  r2, r2, #4
    bx   lr

@ Escreve r2 no DATA_IN e pulsa o clock/enable do coprocessador.
send_cmd:
    str  r2, [r9, #0x20]
    mov  r3, #1
    str  r3, [r9, #0x10]
    mov  r3, #0
    str  r3, [r9, #0x10]
    ldr  r3, [r9, #0x00]            @ pipeline drain
    ldr  r3, [r9, #0x00]
    bx   lr

@ Verifica bit 6 (erro); retorna 0 se ok, -99 e envia clear se erro.
chk_err:
    ldr  r3, [r9, #0x00]
    tst  r3, #0x40
    bne  chk_fail
    mov  r0, #0
    bx   lr
chk_fail:
    mov  r3, #2
    str  r3, [r9, #0x10]
    mov  r3, #0
    str  r3, [r9, #0x10]
    mov  r0, #-99
    bx   lr
