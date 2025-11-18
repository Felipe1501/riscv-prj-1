#include <iostream>

#include "bus.hpp"
#include "cpu.hpp"
#include "memory.hpp"
#include "utils.hpp"

int main() {
    std::cout << "Iniciando teste (LW/SW)\n";

    Memory mem(256);
    Bus bus(mem);
    CPU cpu(bus);

    uint32_t inst0 = encodeADDI(10, 0, 32);
    uint32_t inst1 = encodeADDI(1, 0, 123);
    uint32_t inst2 = encodeSW(1, 10, 0);
    uint32_t inst3 = encodeLW(2, 10, 0);
    uint32_t inst4 = encodeR(3, 1, 2, 0x0, 0x00);

    mem.write32(0x00, inst0);
    mem.write32(0x04, inst1);
    mem.write32(0x08, inst2);
    mem.write32(0x0C, inst3);
    mem.write32(0x10, inst4);

    std::cout << "\nDump inicial da CPU:\n";
    cpu.dump();

    std::cout << "\nExecutando 5 instrucoes:\n";
    cpu.step();
    cpu.step();
    cpu.step();
    cpu.step();
    cpu.step();

    std::cout << "\nDump final da CPU:\n";
    cpu.dump();

    return 0;
}