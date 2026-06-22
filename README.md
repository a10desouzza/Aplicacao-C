# Especificações da Aplicação
Esta é uma Aplicação em C para a placa De1-SoC, responsável por integrar três componentes distintos em um único fluxo funcional: o Driver em Assembly (descrito anteriormente), o CoProcessador de Maike e a Controladora VGA presente na ponte lightweight da HPS.

A Aplicação atua como a camada de mais alto nível do sistema. É ela quem lê os arquivos do disco, interpreta a entrada do usuário — seja via arquivo PNG ou via desenho com o mouse — prepara os dados e os entrega ao Driver, que por sua vez os envia ao CoProcessador via MMIO. Paralelamente, a Aplicação controla o IP do VGA para exibir ao usuário, em tempo real, a imagem que está sendo processada.

O programa consiste em um arquivo `.c` (Aplicação principal), um `.h` (Cabeçalho com protótipos e constantes do Driver) e um `.s` (o próprio Driver em Assembly, reutilizado sem modificações na lógica de envio).

# Fundamentação Teórica

1. **Controladora VGA e MMIO**
   - A saída de vídeo da placa De1-SoC é controlada por um IP core dedicado, acessível através da mesma ponte lightweight HPS-to-FPGA que já utilizamos pelo Driver. Assim como os registradores do CoProcessador, os registradores do IP VGA são mapeados em endereços físicos da memória, tornando o acesso idêntico ao de qualquer outra operação MMIO: abre-se o `/dev/mem`, mapeia-se a página física com `mmap` e escreve-se diretamente nos offsets correspondentes.
   - No nosso projeto, o IP VGA recebe um pixel por vez, contendo as coordenadas `(x, y)` e os valores de cor em RGB de 3 bits cada, em uma única palavra de 32 bits. Após a escrita no registrador de dados, um pulso no registrador de sinais aciona o processamento, e o registrador de status é lido em polling até que o IP sinalize a conclusão.

2. **Protocolo de Pacotes do Pixel VGA**
   - Para que o IP VGA reconheça e pinte corretamente cada pixel na tela, a palavra de 32 bits precisa ser compactada em um formato específico, definido pela controladora. O compactamento segue a lógica abaixo:

   ```c
   static uint32_t pack_pixel(int x, int y, int r, int g, int b) {
       return ((uint32_t)(y & 0x3FF) << 19)
            | ((uint32_t)(x & 0x3FF) << 9)
            | ((uint32_t)(r & 0x7)   << 6)
            | ((uint32_t)(g & 0x7)   << 3)
            | ((uint32_t)(b & 0x7));
   }
   ```

   - A resolução máxima é de 320×240 pixels. Como a imagem da rede neural tem 28×28 pixels, cada unidade da grade é ampliada em um bloco de 8×8 pixels na tela, resultando em uma área de desenho de 224×224 pixels centralizada no monitor.

3. **Entrada via Mouse no Linux (PS/2 e `/dev/input/mice`)**
   - Em sistemas Linux, o dispositivo `/dev/input/mice` agrega os eventos de todos os mouses conectados ao sistema em um protocolo de pacotes de 3 bytes. O primeiro byte carrega as flags dos botões e bits de sinal dos deslocamentos; o segundo e o terceiro carregam os deltas de movimento em `x` e `y`, como valores `signed char`.

   - A leitura desse dispositivo é restritiva por natureza, o que exigiria uma thread dedicada caso a aplicação precisasse escutar o teclado ao mesmo tempo. Optamos por usar uma syscall `select`, que permite monitorar múltiplos descritores de arquivo simultaneamente, bloqueando a execução apenas até que qualquer um deles tenha dados disponíveis sem gastar ciclos de CPU em espera ativa.

4. **Algoritmo de linha de Bresenham**
   - O problema da rasterização de linhas está na adaptação de uma reta matemática contínua para uma tela digital composta por uma matriz discreta de pixels inteiros. Algoritmos tradicionais resolvem isso calculando a equação da reta passo a passo, o que exige divisões, números decimais (floating-point) e arredondamentos constantes — operações que são computacionalmente lentas e pesadas para o hardware de processamento.

   - O Algoritmo de Bresenham resolve isso eliminando os números decimais e aplicando apenas aritmética de inteiros. Ele utiliza uma variável de decisão que acumula o erro de aproximação a cada passo; dependendo do sinal dessa variável (positivo ou negativo), o algoritmo descobre se o próximo pixel correto deve ser o vizinho direto ou o em diagonal. Como a atualização desse erro envolve apenas somas, subtrações e multiplicações por dois (deslocamento de bits), a renderização se torna extremamente rápida e otimizada para o nível de hardware.

5. **Polling**
   - É uma maneira de um Processador tratar suas entradas e saídas. Nessa filosofia, o Processador fica em um ciclo de pedir um sinal para a conexão, e só desvia sua atenção para a próxima, quando algum sinal for recebido.

    - Embora possua seus problemas, por conta da arquitetura do projeto, e a frequência em que as conexões são usadas, optamos por usar essa filosofia.


# Metodologia

- **Entradas:** A Aplicação aceita imagens de dois modos distintos: a partir de um arquivo PNG fornecido pelo usuário, ou diretamente do desenho feito com o mouse sobre a tela VGA. Em ambos os casos, a saída desta etapa é um buffer de 784 bytes (`uint8_t img[28*28]`) que é passado ao Driver.

- **Saídas:** O dígito predito pelo CoProcessador é exibido no terminal. O VGA exibe a imagem 28×28 ampliada antes da inferência, e em modo de desenho, atualiza a tela em tempo real conforme o usuário pinta.

Com essa arquitetura definida, nosso objetivo era criar uma Aplicação que conectasse esses três sistemas: Driver, CoProcessador e VGA, de forma transparente para o usuário final, com uma interface de menu simples no terminal.

## 1. A Camada VGA
   - **Requisitos**
     
     Foi requisitado pelo problema, que o usuário pudesse vizualizar as imagens que fossem enviadas para a inferência, além de conseguir desenhar, no monitor VGA, uma nova imagem de um número. Para isso, tivemos que adaptar, além da vizualização, uma maneira do usuário desenhar com o mouse, e mostrar na tela em tempo real. Não nos foi requisitada nenhum padrão de cor específico, então não vamos entrar em detalhes sobre isso.
     
   - **Inicialização e Mapeamento**

     Assim como o Driver mapeia os registradores do CoProcessador, a Aplicação realiza seu próprio mapeamento da ponte lightweight para acessar os registradores do IP VGA. A inicialização abre `/dev/mem`, mapeia a página física em `0xFF200000` e calcula os ponteiros para os três registradores relevantes: `g_status` (0x30), `g_signals` (0x40) e `g_data` (0x50). Um pulso de reset é enviado ao IP logo após o mapeamento para garantir um estado inicial limpo.

   - **Escrita de Pixels**

     A função `vga_set_pixel` é a principal função da camada de vídeo. Ela comprime as coordenadas e a cor em uma palavra de 32 bits, escreve no registrador de dados, envia o enable e aguarda o bit de conclusão no registrador de status em polling. A partir dela, `vga_fill_rect` permite pintar retângulos inteiros, e `vga_show_image28` converte o buffer de 784 bytes em 784 blocos 8×8 na tela, mapeando a intensidade de cada pixel de 8 bits para os 3 bits de brilho disponíveis no IP (`v = pixel >> 5`).
      Tinhamos adicionado uma função "blur", que fazia uma média entre todos os espaços 3x3 de píxel, e igualava as cores dessa área à essa média. Essa foi uma escolha de projeto tomada por muitos de nossos colegas, já que para a maioria dos casos, a acurácia do projeto foi aumentada, mesmo que o blur não estivesse sendo mostrado no monitor, porém no nosso caso, essa função causou uma queda no valor médio da acurácia, o que nos fez escolher tirar essa função do projeto, já que ela não era um dos requisitos.
     
   - **Modo de Desenho**

     O modo de desenho é a função mais complexa da Aplicação. Ele combina a leitura do mouse via `/dev/input/mice` com a atualização em tempo real do VGA. O cursor é representado visualmente como um bloco branco que se move pela grade 28×28. Quando o botão esquerdo está pressionado, a função `paint_block` marca a célula como branca no estado interno e na tela. Para traços rápidos, onde o mouse percorre múltiplas células entre duas leituras, implementamos, com a biblioteca stb, o algoritmo de linha de Bresenham, que interpola e pinta todas as células intermediárias, para não ficarem fragmentos.

     O botão do meio do mouse limpa o a matriz inteira. O Enter no teclado confirma e encerra o modo de desenho, devolvendo o buffer preenchido.

     O uso de `select` para monitorar tanto o mouse (`/dev/input/mice`) quanto o teclado (`STDIN_FILENO`) ao mesmo tempo foi a solução para evitar que a aplicação ficasse travada esperando o mouse quando o usuário queria pressionar Enter, e vice-versa.

   ```c
   if (select(fd + 1, &fds, NULL, NULL, NULL) < 0) break;

   if (FD_ISSET(STDIN_FILENO, &fds)) { /* verifica tecla Enter */ }
   if (FD_ISSET(fd, &fds))           { /* processa evento do mouse */ }
   ```

## 2. A Integração com o Driver e o CoProcessador

   ![DiagramaDados](/data/HPS-Update.png)
   
   - **Arquitetura do Sistema**
      
      Primeiramente, tivemos que reformular o Driver em assembly, do Marco 2 para não acessar a pasta raiz do sistema de arquivos por conta própria. Embora a maneira que o Driver acessava os arquivos, não tinha sido um requisito do segundo marco, foi estabelecida como requisito desse terceiro, então isso não foi somente uma escolha de projeto. Agora, suas funções `carregar_pesos`, `carregar_bias`, `carregar_beta` e `carregar_imagem` recebem apenas ponteiros para buffers já carregados na memória pelo C, algo que foi facilitado pelo uso de bibliotecas. Isso tornou possível que a imagem desenhada com o mouse que nunca existiu como arquivo percorra exatamente o mesmo caminho que uma imagem lida do disco.

      
   ```c
   static int inferir(const uint8_t img[ELM_N_IMG]) {
       carregar_imagem(img);   /* ponteiro direto, sem arquivo */
       iniciar_inferencia();
       return obter_resultado();
   }
   ```

   - **Carregamento do Modelo**

     Os parâmetros da rede (pesos, beta e bias) são carregados uma única vez na inicialização, pela função `carregar_modelo`. Os caminhos padrão são definidos por macros (`PATH_WEIGHTS`, `PATH_BETA`, `PATH_BIAS`) e podem ser reescritos por argumentos de linha de comando (`-w`, `-b`, `-s`), facilitando a configuração sem precisar recompilar.

   - **Pipeline de Inferência da Imagem Desenhada**

     Após o usuário confirmar o desenho, a Aplicação executa três etapas antes de enviar ao Driver: (1) salva o desenho como PNG em `minhas_imagens/`, permitindo que o usuário reveja ou reutilize seus desenhos; (2) aplica o blur de suavização apenas no buffer em memória RAM, sem alterar o que está exibido no VGA; (3) passa o buffer suavizado ao `inferir()`.

     Essa ordem é intencional: o VGA continua exibindo o desenho original, do jeito que foi recebido, enquanto o CoProcessador recebe a versão processada.

## 3. Interface de Usuário e Benchmark

   - **Menu Interativo**
     
     Tentamos manter a interface do usuário apenas no que nos foi requisitado: inferência por arquivo, envio individual de cada parâmetro do modelo, modo de desenho, reset da FPGA e benchmark. A separação de cada ação em uma função própria (`enviar_imagem_arquivo`, `enviar_pesos`, `modo_desenho`, etc.) ajudou na organização do nosso código princial.

   - **Modo Benchmark**

     O modo benchmark percorre todas as imagens de uma pasta informada pelo usuário, infere cada um e mede a latência individualmente com `clock_gettime(CLOCK_MONOTONIC)`. O rótulo esperado é o nome da pasta (A pasta tem o caminho do dígito "/5"). Ao final, imprime e salva em `benchmark.csv` as métricas completas: acurácia, latência média, desvio padrão e throughput de imagens por segundo. Embora o formato de saída desse benchmark tenha sido escolha de projeto, as informações que ele contêm foram padrão das outras aplicações, já que passavam por um requisito de conteúdo.

# Manual de Uso

- Faça o download de todo o conteúdo das pastas `data` e `src`;
- Carregue, na placa De1-SoC, o projeto CoProcessador de Maike;
- Conecte sua máquina remotamente ao terminal do Processador ARMv7 da De1-SoC;
- Certifique-se de que um mouse está conectado à placa e acessível via `/dev/input/mice`;
- Execute no terminal:

```bash
sudo su
gcc main.c driver.s -o elm -I. -no-pie -lm
./elm
```

- Inferir diretamente uma imagem:

```bash
./elm -w caminho/pesos.bin -b caminho/beta.bin -s caminho/bias.bin -i imagem.png
```

O `-lm` é necessário para as funções matemáticas (`sqrt`) usadas pelo benchmark. O `-no-pie` e o `sudo su` seguem a mesma necessidade descrita no Driver: acesso root ao `/dev/mem` e linkagem sem conflito entre o `.c` e o `.s`.


# Testes e Discussões

  - Assim como requisitado pelo projeto, o usuário tem a opção de gerar, em .CSV, os resultados dos benchmarks de cada dígito, onde todas as imagens passam pela inferência. Esse teste já foi realizado por nós, e está disponível como o benchmark geral de todas as imagens:

### Resultados gerais (9.358 imagens)

| Dígito   |   Imagens |   Acertos |   Erros | Acurácia   |   Lat. média (ms) |   Desvio (ms) |   Throughput (img/s) | Erro mais frequente   |
|:---------|----------:|----------:|--------:|:-----------|------------------:|--------------:|---------------------:|:----------------------|
| 0        |       980 |       910 |      70 | 92.9%      |            18.63  |         0.055 |                53.68 | 6 (18 casos)          |
| 1        |      1135 |      1106 |      29 | 97.4%      |            18.647 |         0.4   |                53.63 | 8 (10 casos)          |
| 2        |      1032 |       810 |     222 | 78.5%      |            18.552 |         0.062 |                53.9  | 3 (47 casos)          |
| 3        |      1010 |       817 |     193 | 80.9%      |            18.603 |         0.224 |                53.76 | 5 (43 casos)          |
| 4        |       982 |       833 |     149 | 84.8%      |            18.629 |         0.052 |                53.68 | 9 (69 casos)          |
| 5        |       892 |       572 |     320 | 64.1%      |            18.642 |         0.057 |                53.64 | 3 (80 casos)          |
| 6        |       958 |       856 |     102 | 89.4%      |            18.641 |         0.279 |                53.65 | 0 (37 casos)          |
| 7        |      1028 |       859 |     169 | 83.6%      |            18.611 |         0.236 |                53.73 | 1 (48 casos)          |
| 8        |       332 |       236 |      96 | 71.1%      |            18.654 |         0.08  |                53.61 | 1 / 3 / 5 (16 cada)   |
| 9        |      1009 |       778 |     231 | 77.1%      |            18.614 |         0.468 |                53.72 | 4 (68 casos)          |
| Geral    |      9358 |      7777 |    1581 | 83.1%      |            18.62  |         0.255 |                53.71 | —                     |

   - A acurácia do nosso sistema atingiu os mesmos 83% do último marco, o que mostra o uso correto do mesmo driver em ambos.

   - O Dígito que o algoritmo mais acerta é o 1, com 97.4% de acurácia, enquanto o 5, é o que mais erra, com 64.1% de acurácia, sendo que 25% dos erros, é o dígito 3.

   - A latência mantém uma média estável, com um desvio baixíssimo, o que era esperado, já que mesmo que para nós as imagens sejam diferentes, elas possuem os mesmos formatos e pesos.

   - O sistema tem um throughput de 53.71 imagens por segundo.

# Conclusão

Acreditamos ter atendido todos os requisitos do projeto, e seguido as escolhas discutidas em sala. Nosso projeto é totalmente funcional, integramos o Driver com a controladora VGA, e geramos uma aplicação simples, mas funcional, para o usuário ter controle e visualização dos resultados.

Corrigimos o Driver do último marco, que agora tem um funcionamento condizente com um Driver real, isso ajudou muito com o funcionamento da aplicação e da nova conexão com a controladora VGA e o modo de desenho.

Como ponto de melhoria, consideramos que o Blur poderia ter sido implementado, se tivéssemos percebido o problema com mais tempo, já que ele costuma melhorar a acurácia da inferência, e gerar uma imagem melhor para o usuário. Também podemos argumentar que o nosso projeto é dependente do polling, que gera uma certa latência e atrapalha a fluidez do modo de desenho.

# Referências

Controladora VGA e CoProcessador, ambos de Maike: https://github.com/DestinyWolf/Problema_SD_2026_1?authuser=0

Resultado do Marco 2, de nossa autoria: https://github.com/a10desouzza/Driver

Biblioteca stb:
https://github.com/nothings/stb

Algoritmo de Bresenham:
https://materialpublic.imd.ufrn.br/curso/disciplina/5/69/7/4
