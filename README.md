# Simulador de Computador RISC-V (RV32I)

**Disciplina:** Organização de Computadores  
**Professor:** Walter S. Oliveira  
**Projeto:** Desenvolvimento de um Simulador RISC-V
**Link do video** -> https://youtu.be/ka-XsrGhIR8?si=DnuShVLv0I6W9pt0
---

## 1. Introdução

Este projeto consiste na implementação de um simulador funcional de um computador baseado na arquitetura **RISC-V**, especificamente o conjunto de instruções **RV32I** (números inteiros de 32 bits).

O objetivo é simular o ciclo de vida completo de instruções (Busca, Decodificação e Execução), o gerenciamento de memória (RAM e VRAM) e a comunicação via barramento, incluindo o tratamento de interrupções de hardware.

O simulador foi desenvolvido em **C++17**, privilegiando performance e tipagem forte conforme sugerido na especificação do projeto.

---

## 2. Arquitetura do Sistema

O simulador está estruturado em módulos independentes que se comunicam através de um barramento central, replicando a arquitetura física de um computador real.

### 2.1. CPU (Processador)

A classe `CPU` (`src/cpu.cpp`) é o núcleo do sistema. Ela implementa o ciclo _Fetch-Decode-Execute_ (Busca-Decodificação-Execução):

1.  **Fetch:** O `pc` (Program Counter) é usado para buscar a próxima instrução de 32 bits via barramento.
2.  **Decode:** A instrução é quebrada em seus campos constituintes (`opcode`, `rd`, `funct3`, `rs1`, `rs2`, `funct7`) utilizando máscaras de bits.
3.  **Execute:** Um `switch` baseado no `opcode` despacha a operação correspondente (lógica, aritmética, acesso à memória ou desvio).
4.  **Write Back:** O resultado é escrito no registrador de destino (`rd`), garantindo que o registrador `x0` permaneça sempre zero.

**Instruções Suportadas (RV32I Base):**

-   **Aritméticas/Lógicas:** ADD, SUB, AND, OR, XOR, SLT, etc.
-   **Imediatas:** ADDI, ANDI, ORI, LUI, AUIPC.
-   **Memória:** LW, SW, LB, SB, LH, SH (com suporte a _Little Endian_).
-   **Desvio:** BEQ, BNE, BLT, BGE, JAL, JALR.
-   **Sistema:** MRET (retorno de interrupção).

### 2.2. Barramento (Bus)

A classe `Bus` (`src/bus.cpp`) atua como o decodificador de endereços do sistema. Todas as operações de leitura e escrita da CPU passam por ele. O barramento verifica o endereço requisitado e roteia o acesso para o dispositivo correto, lançando exceções caso o endereço seja inválido ou reservado.

**Mapa de Memória Implementado:**

| Faixa de Endereço (Hex) | Dispositivo       | Descrição                                         |
| :---------------------- | :---------------- | :------------------------------------------------ |
| `0x00000` - `0x7FFFF`   | **RAM Principal** | Dados, Pilha e Programa                           |
| `0x80000` - `0x8FFFF`   | **VRAM**          | Memória de Vídeo (Saída ASCII)                    |
| `0x90000` - `0x9FBFF`   | _Reservado_       | Área protegida (Gera exceção de acesso)           |
| `0x9FC00` - `0x9FFFF`   | **E/S (IO)**      | Periféricos mapeados em memória (Ex: Teclado/IRQ) |

### 2.3. Memória (RAM e VRAM)

A classe `Memory` (`src/memory.cpp`) simula o armazenamento físico. Ela utiliza um `std::vector<uint8_t>` para armazenar os bytes.

-   **Leitura/Escrita de 32 bits:** A memória implementa a lógica _Little Endian_, onde o byte menos significativo é armazenado no menor endereço (ex: `0x11223344` é armazenado como `44 33 22 11`).

---

## 3. Funcionalidades Avançadas e Pontos Extras

Além dos requisitos básicos, o simulador implementa funcionalidades adicionais para enriquecer a experiência e cumprir os critérios de pontuação extra.

### 3.1. E/S Programada e VRAM

Conforme exigido, o simulador monitora a execução. A cada _N_ instruções (configurável), o conteúdo da VRAM (faixa `0x80000`) é lido e impresso no terminal como uma matriz de caracteres ASCII, simulando um monitor de vídeo.

### 3.2. Sistema de Interrupções (Ponto Extra)

O simulador suporta interrupções externas reais.

-   **Funcionamento:** Um periférico fictício (`FakeIO`) mapeado em `0x9FC00` possui uma flag de interrupção.
-   **Gatilho:** Ao pressionar a tecla **'k'** durante a execução contínua, o sistema aciona essa flag via E/S mapeada.
-   **Tratamento na CPU:** Antes de cada ciclo `step()`, a CPU verifica o barramento. Se houver interrupção pendente:
    1.  O endereço atual do PC é salvo em `saved_pc`.
    2.  O fluxo desvia para o vetor de interrupção `0x100`.
    3.  O _handler_ executa e retorna via instrução `MRET`.

### 3.3. Interatividade Total (Ponto Extra)

Diferente de simuladores "caixa-preta", este projeto oferece um menu interativo robusto que permite:

-   Executar o processador passo-a-passo ou em lote.
-   **Inspecionar** o estado interno (Registradores e Memória).
-   **Editar manualmente** qualquer byte da RAM ou VRAM em tempo de execução, permitindo testar comportamentos sem recompilar o código.

---

## 4. Compilação e Execução

### Pré-requisitos

-   Compilador compatível com **C++17** (GCC/MinGW recomendado).
-   Sistema Operacional Windows (devido ao uso da biblioteca `<conio.h>` para captura de teclado em tempo real).

### Como Compilar

Na raiz do projeto, execute:

```bash
g++ -std=c++17 -Iinclude src/*.cpp -o main.exe
```

### Como Executar

```bash
main.exe
```

## 5. Manual de Uso

Ao iniciar, o simulador carrega automaticamente um programa de teste na memória (que realiza cálculos, manipula a VRAM e configura o vetor de interrupção). O menu principal oferece as seguintes opções:

1.  **Executar 1 instrução:** Avança um único ciclo de clock. Ideal para depuração detalhada.
2.  **Executar N instruções:** Processamento em lote.
    -   _Dica:_ Pressione **'k'** durante a execução para testar a interrupção externa.
3.  **Mostrar registradores:** Exibe o banco de registradores (x0-x31) e o PC atual.
4.  **Dump de RAM:** Visualiza os primeiros 64 bytes da memória principal em hexadecimal.
5.  **Dump de VRAM ASCII:** Renderiza o conteúdo da memória de vídeo no terminal.
6.  **Editar RAM:** Permite injetar dados ou código diretamente na memória.
    -   _Entrada:_ Endereço (Decimal) e Valor (0-255).
7.  **Editar VRAM:** Permite desenhar na tela manualmente alterando bytes da VRAM.

---

## 6. Referências

1.  David Patterson e Andrew Waterman. _Guia Prático RISC-V: Atlas de uma Arquitetura aberta_. Strawberry Canyon LLC, 2019.
2.  RISC-V International. https://riscv.org/.
3.  Material da disciplina de Organização de Computadores - UNISANTOS.
