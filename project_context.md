# Project Context: RISC-V Simulator (Maximum Detail)

This document provides an exhaustive, line-by-line explanation of every file in the workspace to serve as a context source for LLM prompts.

## File List

- `CMakeLists.txt`: CMake build configuration.
- `README.md`: Project documentation (Empty).
- `assets/`
    - `vram_dump_example.txt`: Example VRAM dump (Empty).
- `include/`
    - `bus.hpp`: Bus class header.
    - `cpu.hpp`: CPU class header.
    - `isa.hpp`: Instruction Set Architecture definitions.
    - `memory.hpp`: Memory class header.
    - `utils.hpp`: Utility functions header (instruction encoding).
- `programs/`
    - `hello_rv32i.hex`: Hex file (Not read, assumed binary/data).
    - `loop_test.hex`: Hex file (Not read, assumed binary/data).
    - `memory_test.hex`: Hex file (Not read, assumed binary/data).
- `scripts/`
    - `compile.sh`: Compilation script (Empty).
    - `convert_hex.py`: Hex conversion script (Empty).
    - `run.sh`: Execution script (Empty).
- `src/`
    - `bus.cpp`: Bus class implementation.
    - `cpu.cpp`: CPU class implementation.
    - `loader.cpp`: Loader implementation (Empty).
    - `main.cpp`: Main entry point.
    - `memory.cpp`: Memory class implementation.
    - `utils.cpp`: Utility functions implementation.
- `tests/`
    - `test_cpu.cpp`: CPU tests (Empty).
    - `test_memory.cpp`: Memory tests (Empty).
    - `test_vram_output.cpp`: VRAM output tests (Empty).

---

## Detailed File Documentation

### `CMakeLists.txt`
**Purpose**: Configures the build system using CMake.
**Line-by-Line Explanation**:
- `cmake_minimum_required(VERSION 3.16)`: Ensures the user has at least CMake 3.16 installed.
- `project(riscv_sim LANGUAGES CXX)`: Defines the project name as "riscv_sim" and specifies it uses C++.
- `set(CMAKE_CXX_STANDARD 20)`: Sets the C++ standard to C++20. This allows using modern features like `std::span` (though not currently used) or concepts if needed.
- `set(CMAKE_CXX_STANDARD_REQUIRED ON)`: Forces the build to fail if the compiler doesn't support C++20.
- `include_directories(include)`: Adds the `include/` folder to the compiler's search path. This allows `#include "bus.hpp"` to work from `src/` files without needing `../include/bus.hpp`.
- `add_executable(riscv_sim ...)`: Defines the final executable binary named `riscv_sim`. It lists all the source files (`.cpp`) that need to be compiled and linked together:
    - `src/main.cpp`: The entry point.
    - `src/memory.cpp`: Memory logic.
    - `src/bus.cpp`: Bus logic.
    - `src/cpu.cpp`: CPU logic.
    - `src/utils.cpp`: Helper functions.

### `include/bus.hpp`
**Purpose**: Defines the `Bus` class, which acts as the central communication hub between the CPU and Memory. In a real computer, the bus transfers data; here, it's an abstraction to route read/write requests.
**Line-by-Line Explanation**:
- `#pragma once`: Prevents this header from being included multiple times in a single compilation unit.
- `#include "memory.hpp"`: The Bus needs to know about the `Memory` class to talk to it.
- `class Bus { ... };`: Defines the class.
    - `private: Memory& memory;`: A reference to the `Memory` object. It's a reference (`&`) because the Bus doesn't *own* the memory; it just connects to it.
    - `public: explicit Bus(Memory& mem);`: Constructor. `explicit` prevents accidental implicit conversions (e.g., passing a Memory object where a Bus is expected, though unlikely here, it's best practice).
    - `uint8_t read8(uint32_t address) const;`: Declares a function to read 1 byte (8 bits). `const` means this function doesn't change the state of the *Bus* itself (though it might read from Memory).
    - `void write8(uint32_t address, uint8_t value);`: Declares a function to write 1 byte.
    - `uint32_t read32(uint32_t address) const;`: Declares a function to read a 32-bit word (4 bytes). RISC-V instructions are 32-bit, so this is crucial for fetching instructions.
    - `void write32(uint32_t address, uint32_t value);`: Declares a function to write a 32-bit word.

### `include/cpu.hpp`
**Purpose**: Defines the `CPU` class, representing a RISC-V RV32I processor core.
**Line-by-Line Explanation**:
- `#include "bus.hpp"`: The CPU needs the Bus to fetch instructions and access data.
- `class CPU { ... };`:
    - `private:`
        - `std::array<uint32_t, 32> x{}`: The 32 general-purpose registers (x0 to x31). `uint32_t` means they are 32-bit unsigned integers. `{}` initializes them to zero.
        - `uint32_t pc{0}`: The Program Counter. It holds the memory address of the *next* instruction to execute. Initialized to 0.
        - `Bus& bus`: Reference to the Bus.
        - `uint32_t getBits(...)`: Helper to extract specific bits from an instruction (e.g., getting the opcode).
        - `int32_t signExtend(...)`: Helper to convert a smaller number (e.g., 12-bit immediate) into a full 32-bit signed integer, preserving the sign.
    - `public:`
        - `explicit CPU(Bus& b)`: Constructor.
        - `void reset()`: Puts the CPU in a known initial state.
        - `void dump() const`: Debugging function to show register contents.
        - `void step()`: The heart of the simulation. Executes exactly *one* instruction.

### `include/isa.hpp`
**Purpose**: Defines the "vocabulary" of the processor. These are the magic numbers that the CPU understands.
**Line-by-Line Explanation**:
- `namespace rv32i { ... }`: Wraps constants in a namespace to avoid name collisions.
- **Opcodes** (The last 7 bits of an instruction):
    - `OPCODE_OP_IMM = 0x13`: Instructions that do math with a register and a constant (Immediate) (e.g., `ADDI`).
    - `OPCODE_OP = 0x33`: Instructions that do math with two registers (e.g., `ADD`).
    - `OPCODE_LOAD = 0x03`: Instructions that read from memory (e.g., `LW`).
    - `OPCODE_STORE = 0x23`: Instructions that write to memory (e.g., `SW`).
    - `OPCODE_BRANCH = 0x63`: Conditional jumps (e.g., `BEQ`).
    - `OPCODE_LUI = 0x37`: Load Upper Immediate (loads high 20 bits).
    - `OPCODE_AUIPC = 0x17`: Add Upper Immediate to PC.
    - `OPCODE_JAL = 0x6F`: Jump And Link (unconditional jump).
    - `OPCODE_JALR = 0x67`: Jump And Link Register (indirect jump).
- **Funct3** (Bits 14-12): Distinguishes instructions sharing the same opcode.
    - E.g., `OPCODE_OP_IMM` (0x13) could be `ADDI` (funct3=0x0), `SLTI` (funct3=0x2), etc.
    - `FUNCT3_LW`, `FUNCT3_SW`: Width of memory access (Word, Halfword, Byte).
- **Funct7** (Bits 31-25): Further distinguishes R-Type instructions.
    - E.g., `ADD` and `SUB` both use `OPCODE_OP` (0x33) and `FUNCT3_ADD_SUB` (0x0). `FUNCT7_ADD` (0x00) vs `FUNCT7_SUB` (0x20) tells them apart.

### `include/memory.hpp`
**Purpose**: Defines the `Memory` class, simulating RAM.
**Line-by-Line Explanation**:
- `std::vector<uint8_t> data`: The actual storage. A dynamic array of bytes.
- `explicit Memory(std::size_t size)`: Constructor taking the size in bytes.
- `read8`/`write8`: Access individual bytes.
- `read32`/`write32`: Access 4 bytes at once. Important: RISC-V is Little Endian, meaning the least significant byte is at the lowest address.
- `dump(...)`: Helper to print a chunk of memory for debugging.

### `include/utils.hpp`
**Purpose**: Declares functions to *create* machine code. This is essentially a mini-assembler.
**Line-by-Line Explanation**:
- `encodeADDI(rd, rs1, imm)`: Creates the 32-bit binary for `ADDI rd, rs1, imm`.
- `encodeR(...)`: Generic encoder for R-Type instructions (ADD, SUB, XOR, etc.).
- `encodeB(...)`: Encoder for Branch instructions (BEQ, BNE, etc.). Note the complex immediate encoding in RISC-V branches.
- `encodeJAL`, `encodeLUI`: Encoders for J-Type and U-Type instructions.

### `src/bus.cpp`
**Purpose**: Implements the Bus logic.
**Line-by-Line Explanation**:
- `Bus::Bus(Memory& mem) : memory(mem) {}`: Initializes the reference.
- `read8/write8`:
    - `std::cout << ...`: Logs the access. This is useful for tracing execution but slows it down.
    - `return memory.read8(address)`: Delegates the actual work to the Memory object.
- `read32/write32`: Same as above, but for 32-bit words.

### `src/cpu.cpp`
**Purpose**: The core logic. Implements the Fetch-Decode-Execute cycle.
**Detailed Logic**:
- **`getBits(value, hi, lo)`**:
    - `mask = ((1u << (hi - lo + 1)) - 1u)`: Creates a mask of 1s. E.g., if we want 3 bits, `1 << 3` is `1000` (8). `8 - 1` is `111` (7).
    - `(value >> lo) & mask`: Shifts the bits down and masks the rest.
- **`signExtend(value, bits)`**:
    - `shift = 32 - bits`: Calculates how far to shift left to hit the sign bit.
    - `(s << shift) >> shift`: Shifts left to move the sign bit to the MSB (Most Significant Bit), then arithmetic shifts right. In C++, right-shifting a signed integer fills with the sign bit (1 for negative, 0 for positive).
- **`step()`**:
    1.  **Fetch**: `inst = bus.read32(pc)`. Gets the 32-bit instruction at the current PC.
    2.  **Decode**:
        - `opcode = getBits(inst, 6, 0)`: Bottom 7 bits.
        - `rd = getBits(inst, 11, 7)`: Destination register.
        - `funct3 = getBits(inst, 14, 12)`: Function code 3.
        - `rs1 = getBits(inst, 19, 15)`: Source register 1.
        - `rs2 = getBits(inst, 24, 20)`: Source register 2.
        - `funct7 = getBits(inst, 31, 25)`: Function code 7.
    3.  **Execute (Switch on Opcode)**:
        - **`OPCODE_LUI`**: `x[rd] = imm_u`. Loads a 20-bit immediate into the upper 20 bits of the register. Used for constructing large constants.
        - **`OPCODE_AUIPC`**: `x[rd] = pc + imm_u`. Adds the upper immediate to the *current PC*. Used for PC-relative addressing.
        - **`OPCODE_JAL`**: Unconditional jump.
            - Decodes the scrambled immediate (RISC-V J-immediate is shuffled).
            - `x[rd] = pc + 4`: Saves the return address (instruction after the jump).
            - `next_pc = pc + imm`: Sets the new PC.
        - **`OPCODE_JALR`**: Indirect jump.
            - `target = (x[rs1] + imm) & ~1u`: Jumps to address in `rs1` + offset. The `& ~1u` clears the bottom bit (alignment).
        - **`OPCODE_BRANCH`**:
            - Decodes B-immediate (scrambled).
            - Checks condition (e.g., `x[rs1] == x[rs2]` for BEQ).
            - If true, `next_pc = pc + imm`.
        - **`OPCODE_OP_IMM`**:
            - `ADDI`: `x[rd] = x[rs1] + imm`. Note: `SUBI` doesn't exist; use `ADDI` with negative immediate.
            - `SLTI/SLTIU`: Set Less Than (Immediate). Sets `x[rd]` to 1 if `x[rs1] < imm`.
            - `ANDI/ORI/XORI`: Bitwise logic.
            - `SLLI/SRLI/SRAI`: Shifts. `SRAI` is Arithmetic Right Shift (preserves sign).
        - **`OPCODE_OP`**:
            - `ADD/SUB`: Standard math.
            - `SLT/SLTU`: Set Less Than (Register vs Register).
            - `SLL/SRL/SRA`: Register-controlled shifts.
        - **`OPCODE_LOAD`**:
            - `LW`: Loads 32 bits.
            - `LH/LHU`: Loads 16 bits. `LH` sign-extends (e.g., 0xFFFF becomes 0xFFFFFFFF = -1). `LHU` zero-extends (0xFFFF becomes 0x0000FFFF = 65535).
            - `LB/LBU`: Loads 8 bits.
        - **`OPCODE_STORE`**:
            - `SW`: Stores 32 bits.
            - `SH`: Stores 16 bits (bottom 16 bits of register).
            - `SB`: Stores 8 bits (bottom 8 bits of register).
    4.  **Cleanup**:
        - `pc = next_pc`: Advances execution.
        - `x[0] = 0`: **Crucial**. Register x0 is *always* 0 in RISC-V. Writes to it are ignored (or rather, overwritten here to ensure consistency).

### `src/memory.cpp`
**Purpose**: Implements Memory access.
**Line-by-Line Explanation**:
- `read32`:
    - `b0 = data.at(address)`
    - `b1 = data.at(address + 1)` ...
    - `return b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)`: Combines 4 bytes into a 32-bit integer.
    - Example: Memory has `0x11, 0x22, 0x33, 0x44` at address 0.
    - `b0=0x11`, `b1=0x22`...
    - Result: `0x11 | 0x2200 | 0x330000 | 0x44000000` = `0x44332211`. This is Little Endian.
- `write32`:
    - `data.at(address) = value & 0xFF`: Extracts bottom 8 bits.
    - `data.at(address + 1) = (value >> 8) & 0xFF`: Extracts next 8 bits.
    - ... and so on.

### `src/utils.cpp`
**Purpose**: Implements instruction encoding.
**Line-by-Line Explanation**:
- **`encodeADDI(rd, rs1, imm)`**:
    - `imm12 = imm & 0xFFFu`: Takes the bottom 12 bits of the immediate.
    - `(imm12 << 20)`: Moves immediate to the top (bits 31-20).
    - `(rs1 << 15)`: Moves source register index to bits 19-15.
    - `(funct3 << 12)`: Moves funct3 to bits 14-12.
    - `(rd << 7)`: Moves destination register index to bits 11-7.
    - `OPCODE`: Sets the bottom 7 bits.
    - `|`: ORs them all together to form the 32-bit instruction.
- **`encodeB` (Branch)**:
    - RISC-V branch immediate is weirdly scrambled: `imm[12|10:5|4:1|11]`.
    - `imm12 = (imm >> 12) & 0x1`: Extracts bit 12.
    - `imm10_5 = (imm >> 5) & 0x3F`: Extracts bits 10 down to 5.
    - `imm4_1 = (imm >> 1) & 0xF`: Extracts bits 4 down to 1. (Bit 0 is always 0 for alignment).
    - `imm11 = (imm >> 11) & 0x1`: Extracts bit 11.
    - Then shifts them into their specific positions in the instruction word.

### `src/main.cpp`
**Purpose**: The test bench.
**Line-by-Line Explanation**:
- `Memory mem(256)`: Creates 256 bytes of RAM.
- `Bus bus(mem)`: Connects bus to RAM.
- `CPU cpu(bus)`: Connects CPU to bus.
- **Program**:
    - `encodeADDI(10, 0, 64)`: `x10 = x0 + 64`. Sets x10 to 64.
    - `encodeADDI(1, 0, 0)`: `x1 = 0`.
    - `encodeADDI(2, 0, 5)`: `x2 = 5`.
    - `encodeADDI(3, 0, 1)`: `x3 = 1`.
    - `encodeR(..., ADD)`: `x1 = x1 + x3`.
    - `encodeADDI(..., -1)`: `x2 = x2 - 1`.
    - `encodeB(..., -8)`: `BNE x2, x0, -8`. If x2 != 0, jump back 8 bytes (2 instructions). This creates a loop.
    - `encodeSW(...)`: Stores x1 to memory at address x10 (64).
    - `encodeLW(...)`: Loads from memory at address x10 into x4.
- `mem.write32(...)`: Loads the program into memory.
- `cpu.step()`: Runs the loop.

### Empty Files
- `src/loader.cpp`: Placeholder for a future HEX/ELF loader.
- `scripts/`: Placeholders for build/run automation.
- `tests/`: Placeholders for unit tests (currently `main.cpp` acts as the test).
