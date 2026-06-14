# Documentação da API

Esta documentação descreve a API de software em C para o driver desenvolvido em **ARMv8 Assembly**. O driver realiza o mapeamento de memória direta via **MMIO** utilizando a ponte Lightweight HPS-to-FPGA da placa Altera Cyclone V (De1-SoC), permitindo o carregamento de dados e a execução de inferências em lote no CoProcessador de Redes Neurais (ELM).

# Driver Linux ARM Assembly — ELM Co-processor
**TEC 499 — MI Sistemas Digitais**  
Documentação da API Assembly — v1.0 | 2026.1

---

## 1. Visão Geral

O driver é implementado inteiramente em Assembly ARM (ARMv7) e roda no espaço de usuário do Linux embarcado na DE1-SoC (HPS — Hard Processor System). A comunicação com o co-processador ELM na FPGA é feita via MMIO (Memory-Mapped I/O) através da Lightweight HPS-to-FPGA Bridge, mapeada no endereço físico `0xFF200000`.

Todas as funções exportadas seguem a **AAPCS** (ARM Architecture Procedure Call Standard): argumentos em `r0–r3`, valor de retorno em `r0`, registradores `r4–r11` preservados pelo chamado.

---

## 2. Convenção de Registradores Internos

| Registrador | Papel |
|---|---|
| `r0 – r3` | Argumentos de entrada e valor de retorno (caller-saved) |
| `r4` | File descriptor de `/dev/mem` (salvo na pilha) |
| `r5` | Offset de página para mmap2 (`0xFF200000 ÷ 4096`) |
| `r7` | Número da syscall Linux ARM |
| `r9` | Ponteiro virtual base do MMIO (resultado do mmap2) |
| `r10` | Contador do laço corrente |
| `r11` | Limite superior do laço corrente |
| `r2` | Instrução de 32 bits a enviar (entrada de `send_cmd`) |
| `r1` | Índice lógico do dado corrente (entrada das rotinas `fmt_*`) |
| `lr` | Endereço de retorno (salvo na pilha em todas as funções) |

---

## 3. Mapa de Registradores MMIO

> Base física: `0xFF200000` | Mapeado via `mmap2` com offset em páginas (÷ 4096)

| Offset | Nome | Acesso | Descrição |
|---|---|---|---|
| `0x00` | DATA_OUT | R | Status e resultado. Bit[4]=done, Bit[5]=busy, Bit[6]=error, Bits[3:0]=dígito predito. |
| `0x10` | SIGNALS | W | Controle: Bit[0]=enable (pulso strobe), Bit[1]=clr_operation, Bit[2]=rst (reset). |
| `0x20` | DATA_IN | W | Instrução de 32 bits a enviar ao co-processador. Escrita antes do pulso de enable. |

---

## 4. Funções Exportadas

### 4.1 `mapear_fpga`

```c
int mapear_fpga(void);
```

| Campo | Valor |
|---|---|
| Entrada | Nenhum argumento |
| Salva na pilha | `r4, r5, r7, lr` |
| Retorno `0` | Sucesso |
| Retorno `-1` | Falha ao abrir `/dev/mem` (executar com sudo) |
| Retorno `-2` | Falha no `mmap2` (endereço físico inválido) |

**Fluxo interno de registradores:**

```asm
r7 ← 5              ; syscall open()
r0 ← &dev_mem       ; path "/dev/mem"
r1 ← 0x101002       ; flags O_RDWR | O_SYNC
svc 0
r4 ← r0             ; salva o fd retornado

r7 ← 192            ; syscall mmap2()
r0 ← 0              ; addr = NULL
r1 ← 4096           ; length = 1 página
r2 ← 3              ; PROT_READ | PROT_WRITE
r3 ← 1              ; MAP_SHARED
r5 ← 0xFF200000 >> 12   ; offset em páginas
svc 0

base ← r0           ; salva ponteiro virtual
```

> `O_SYNC` é obrigatório para garantir que escritas MMIO não sejam reordenadas pelo kernel. `MAP_SHARED` garante que as escritas cheguem ao hardware físico.

---

### 4.2 `reiniciar_fpga`

```c
int reiniciar_fpga(void);
```

| Campo | Valor |
|---|---|
| Entrada | Nenhum argumento |
| Salva na pilha | `r9, lr` |
| Retorno `0` | Sucesso |
| Retorno `-1` | FPGA não mapeada (`mapear_fpga` não foi chamada) |

**Fluxo interno de registradores:**

```asm
r9 ← base                ; carrega ponteiro da FPGA
r3 ← 0x4
[r9, #0x10] ← r3         ; sobe  (bit RST = 1)
r3 ← 0
[r9, #0x10] ← r3         ; desce (pulso completo)
```

> A FPGA detecta a transição de borda (0→1→0). O pulso completo é obrigatório para que o hardware saia do estado de reset.

---

### 4.3 `carregar_pesos`

```c
int carregar_pesos(const char *path);
```

| Campo | Valor |
|---|---|
| Entrada | `r0` = ponteiro para o path do arquivo |
| Salva na pilha | `r4, r7, r9, r10, r11, lr` |
| Retorno `0` | Sucesso |
| Retorno `-3` | Falha ao abrir o arquivo de pesos |
| Retorno `-99` | FPGA reportou erro (bit 6 de DATA_OUT) durante a transferência |

**Fluxo interno de registradores:**

```asm
r7 ← 3, r1 ← &buf_w, r2 ← 200704   ; syscall read() → buffer na RAM do HPS
r7 ← 6, r0 ← fd                     ; syscall close()
r10 ← 0, r11 ← 100352               ; inicializa o loop

loop:
  r0 ← ldrh buf_w[r10*2]            ; carrega peso de 16 bits
  r0 ← rev16(r0)                    ; corrige endianness big→little
  r1 ← r10                          ; índice como argumento
  bl fmt_w_addr  →  r2              ; monta instrução STORE_WEIGHTS_ADDR
  bl send_cmd                       ; envia endereço à FPGA
  bl fmt_w_val   →  r2              ; monta instrução STORE_WEIGHTS_VALUE
  bl send_cmd                       ; envia valor à FPGA
  bl chk_err     →  r0              ; verifica bit 6 de DATA_OUT
  r10 ← r10 + 1
```

> Cada peso exige dois envios: `STORE_WEIGHTS_ADDR` seguido de `STORE_WEIGHTS_VALUE`. Valores em ponto fixo Q4.12, 2 bytes cada. Total: 100.352 pesos × 2 instruções = 200.704 envios.

---

### 4.4 `carregar_beta`

```c
int carregar_beta(const char *path);
```

| Campo | Valor |
|---|---|
| Entrada | `r0` = ponteiro para o path do arquivo |
| Salva na pilha | `r4, r7, r9, r10, r11, lr` |
| Retorno `0` | Sucesso |
| Retorno `-4` | Falha ao abrir o arquivo de beta |
| Retorno `-99` | FPGA reportou erro durante a transferência |

**Diferenças em relação a `carregar_pesos`:**

```asm
r2 ← 2560      ; 1280 valores × 2 bytes
r11 ← 1280     ; limite do loop
bl fmt_bt  →  r2   ; monta instrução STORE_BETA (opcode 100)
```

---

### 4.5 `carregar_bias`

```c
int carregar_bias(const char *path);
```

| Campo | Valor |
|---|---|
| Entrada | `r0` = ponteiro para o path do arquivo |
| Salva na pilha | `r4, r7, r9, r10, r11, lr` |
| Retorno `0` | Sucesso |
| Retorno `-5` | Falha ao abrir o arquivo de bias |
| Retorno `-99` | FPGA reportou erro durante a transferência |

**Diferenças em relação a `carregar_pesos`:**

```asm
r2 ← 256       ; 128 valores × 2 bytes
r11 ← 128      ; limite do loop
bl fmt_bs  →  r2   ; monta instrução STORE_BIAS (opcode 011)
```

---

### 4.6 `carregar_imagem`

```c
int carregar_imagem(const char *path);
```

| Campo | Valor |
|---|---|
| Entrada | `r0` = ponteiro para o path do arquivo |
| Salva na pilha | `r4, r7, r9, r10, r11, lr` |
| Retorno `0` | Sucesso |
| Retorno `-6` | Falha ao abrir o arquivo de imagem |
| Retorno `-7` | Falha na leitura (arquivo vazio ou corrompido) |

**Fluxo interno de registradores:**

```asm
r2 ← 784       ; 784 pixels × 1 byte
r11 ← 784      ; limite do loop

loop:
  r0 ← ldrb buf_img[r10]   ; carrega 1 byte (sem rev16 — 8 bits não têm endianness)
  r1 ← r10                 ; índice
  bl fmt_img  →  r2         ; monta instrução STORE_IMG (opcode 000)
  bl send_cmd               ; envia pixel à FPGA
```

---

### 4.7 `iniciar_inferencia`

```c
int iniciar_inferencia(void);
```

| Campo | Valor |
|---|---|
| Entrada | Nenhum argumento |
| Salva na pilha | `r9, lr` |
| Retorno `0` | Comando enviado com sucesso |
| Retorno `-1` | FPGA não mapeada |

**Fluxo interno de registradores:**

```asm
r2 ← 5                    ; opcode START
[r9, #0x20] ← r2          ; DATA_IN ← instrução START
r3 ← 1
[r9, #0x10] ← r3          ; SIGNALS: enable = 1 (strobe sobe)
r3 ← 0
[r9, #0x10] ← r3          ; SIGNALS: enable = 0 (strobe desce)
r0 ← 0                    ; retorna imediatamente
```

> A função retorna imediatamente após o pulso. O cálculo ocorre em paralelo no hardware da FPGA. Use `obter_resultado()` para aguardar e ler o dígito.

---

### 4.8 `obter_resultado`

```c
int obter_resultado(void);
```

| Campo | Valor |
|---|---|
| Entrada | Nenhum argumento |
| Salva na pilha | `r9, lr` |
| Retorno | Inteiro de `0` a `9` — dígito classificado pela rede neural |

**Fluxo interno de registradores:**

```asm
poll:
  r0 ← [r9, #0x00]        ; lê DATA_OUT
  tst r0, #0x10            ; testa Bit[4] (fl_processor_done)
  beq poll                 ; aguarda enquanto done = 0

r0 ← r0 & 0xF             ; isola Bits[3:0] = dígito predito
```

> Busy-wait por polling. O bit done sobe quando a inferência ELM está completa.

---

### 4.9 `ler_status_fpga`

```c
int ler_status_fpga(void);
```

| Campo | Valor |
|---|---|
| Entrada | Nenhum argumento |
| Salva na pilha | `r3, lr` |
| Retorno | Valor bruto de 32 bits do registrador DATA_OUT (offset `0x00`) |

Use as constantes do `elm_driver.h` para decodificar o retorno:

| Constante | Bit | Significado |
|---|---|---|
| `ELM_BIT_DONE` | 4 | Inferência concluída |
| `ELM_BIT_BUSY` | 5 | Hardware ocupado |
| `ELM_BIT_ERROR` | 6 | Erro interno na FPGA |
| `ELM_MASK_DIGIT` | 3:0 | Dígito predito (0–9) |

---

## 5. Funções Internas (não exportadas)

### 5.1 `send_cmd`

Envia uma instrução de 32 bits à FPGA. Chamada por todas as rotinas de carga de dados.

| Campo | Valor |
|---|---|
| Entrada | `r2` = instrução formatada, `r9` = ponteiro base MMIO |
| Saída | Não modifica `r0` |
| Tipo | Função folha (sem push/pop — usa `bx lr`) |

```asm
[r9, #0x20] ← r2          ; DATA_IN ← instrução
r3 ← 1
[r9, #0x10] ← r3          ; SIGNALS: enable = 1 (strobe sobe)
r3 ← 0
[r9, #0x10] ← r3          ; SIGNALS: enable = 0 (strobe desce)
r3 ← [r9, #0x00]          ; dummy read 1 — absorve latência do barramento AXI
r3 ← [r9, #0x00]          ; dummy read 2 — estado do status garantido estável
```

> As duas leituras consecutivas de DATA_OUT não são redundância. O barramento AXI tem latência de propagação após o pulso. A primeira leitura ocorre durante essa janela e pode retornar o estado antigo. A segunda garante que o status já foi atualizado pela FPGA.

---

### 5.2 `chk_err`

Verifica o flag de erro da FPGA (bit 6 de DATA_OUT) após cada instrução enviada.

| Campo | Valor |
|---|---|
| Entrada | `r9` = ponteiro base MMIO |
| Retorno `0` | Sem erro |
| Retorno `-99` | Erro detectado — pulso de `clr_operation` enviado antes de retornar |

```asm
r3 ← [r9, #0x00]          ; lê DATA_OUT
tst r3, #0x40              ; testa Bit[6] (fl_error)
bne erro

erro:
  r3 ← 2
  [r9, #0x10] ← r3        ; pulso clr_operation (SIGNALS[1] = 1 depois 0)
  r3 ← 0
  [r9, #0x10] ← r3
  r0 ← -99
```

---

### 5.3 Formatadores de instrução (`fmt_*`)

Todas recebem `r0` (valor) e `r1` (índice), produzem o word de 32 bits em `r2` e retornam com `bx lr` (funções folha, sem uso de pilha).

| Função | Opcode `[2:0]` | Formato de `r2` |
|---|---|---|
| `fmt_img` | `000` | `r0<<13 \| r1<<3` |
| `fmt_w_addr` | `001` | `r1<<3 \| 0x1` |
| `fmt_w_val` | `010` | `r0<<3 \| 0x2` |
| `fmt_bs` | `011` | `r0<<10 \| r1<<3 \| 0x3` |
| `fmt_bt` | `100` | `r0<<14 \| r1<<3 \| 0x4` |

---

## 6. Conjunto de Instruções (ISA)

Todas as instruções têm 32 bits. Os bits `[2:0]` identificam o opcode.

| Instrução | Opcode `[2:0]` | Campos | Função |
|---|---|---|---|
| `STORE_IMG` | `000` | `[20:13]`=pixel, `[12:3]`=addr | Armazena 1 pixel no endereço indicado |
| `STORE_WEIGHTS_ADDR` | `001` | `[19:3]`=addr (17 bits) | Define o endereço de destino do próximo peso |
| `STORE_WEIGHTS_VALUE` | `010` | `[18:3]`=dado (16 bits) | Armazena o valor do peso no endereço previamente definido |
| `STORE_BIAS` | `011` | `[25:10]`=dado, `[9:3]`=addr | Armazena 1 valor de bias no endereço indicado |
| `STORE_BETA` | `100` | `[29:14]`=dado, `[13:3]`=addr | Armazena 1 valor de beta no endereço indicado |
| `START` | `101` | (sem campos) | Inicia a inferência com os dados já carregados |

> Todos os valores de 16 bits (pesos, beta, bias) estão em ponto fixo Q4.12. Os bytes são invertidos com `rev16` após `ldrh` para corrigir a ordem big-endian dos arquivos antes do envio à FPGA.

---

## 7. Arquivos de Parâmetros

| Arquivo | Tamanho | Tipo | Conteúdo |
|---|---|---|---|
| `../data/w_in_q.bin` | 200.704 B | int16 Q4.12 | 100.352 pesos W_in (784 × 128) |
| `../data/beta_q.bin` | 2.560 B | int16 Q4.12 | 1.280 valores beta (128 × 10) |
| `../data/bias_q.bin` | 256 B | int16 Q4.12 | 128 valores de bias b |
| `../data/image/*.bin` | 784 B | uint8 | 784 pixels da imagem 28 × 28 (MNIST) |

---

## 8. Códigos de Retorno

| Código | Constante | Função de origem | Causa |
|---|---|---|---|
| `0` | `ELM_OK` | todas | Sucesso |
| `-1` | `ELM_ERR_MAP` | `mapear_fpga` / `reiniciar_fpga` | Falha no `open("/dev/mem")` ou FPGA não mapeada |
| `-2` | `ELM_ERR_MMAP` | `mapear_fpga` | Falha no `mmap2` |
| `-3` | `ELM_ERR_PESOS` | `carregar_pesos` | Falha ao abrir `w_in_q.bin` |
| `-4` | `ELM_ERR_BETA` | `carregar_beta` | Falha ao abrir `beta_q.bin` |
| `-5` | `ELM_ERR_BIAS` | `carregar_bias` | Falha ao abrir `bias_q.bin` |
| `-6` | `ELM_ERR_IMG_OPEN` | `carregar_imagem` | Falha ao abrir o arquivo de imagem |
| `-7` | `ELM_ERR_IMG_READ` | `carregar_imagem` | Arquivo de imagem vazio ou corrompido |
| `-99` | `ELM_ERR_FPGA` | `carregar_*` | Flag `fl_error` (bit 6) ativada na FPGA durante transferência |

---

## 9. Build e Execução

```bash
sudo su
gcc main.c driver.s -o elm -I. -no-pie
./elm
```

O `-no-pie` desativa o PIE (Position Independent Executable) para que o linker aceite misturar código C e Assembly sem conflito de relocações. O `sudo` é obrigatório pois o acesso ao `/dev/mem` exige privilégio root.

> A instrução `rev16` exige `-march=armv6t2` ou superior. O Cortex-A9 da DE1-SoC suporta ARMv7, portanto nenhuma flag adicional é necessária.
#define ELM_ERR_READ_IMG    -7   // Arquivo de imagem vazio ou corrompido
#define ELM_ERR_FPGA        -99  // Hardware reportou estouro de memória (Bit 6)
