# Especificações da Aplicação
Esta é uma Aplicação em C, desenvolvida para máquinas ARMv8. A Aplicação integra o Driver em Assembly do marco 2, o Co-Processador de Maike e monitores VGA, através da controladora VGA. Para que a Aplicação realize sua função, é necessário, além do Driver em Assembly, o Co-Processador esteja carregado e funcionando na placa De1-SoC.

O CoProcessador tem a função definida de receber uma imagem de um número, dentre as separadas e retornar uma inferência de qual número está escrito nessa imagem. Ao mesmo tempo, o coprocessador também retorna o seu estado (Done, Busy ou Error)

O driver consiste em um arquivo .s (Código em Assembly), um arquivo .h (Arquivo de definição dos registradores) e um .c (Importação do programa e visualização das saídas).

A Controladora VGA de Maike, tem a função de receber dados de entrada específicos de sua configuração, e retornar, no VGA, uma saída visual, dentro da resolução previamente limitada.

A Aplicação consiste em um .c (Ligação entre Driver e Controladora VGA), além de seus outros arquivos necessários.

# Fundamentação Teórica

# Metodologia

# Manual de uso

# Testes e Resultados

# Conclusão

# Referências

