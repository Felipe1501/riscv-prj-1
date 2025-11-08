#include <iostream>
#include "memory.hpp"
#include "bus.hpp"

int main() {
    std::cout << "Iniciando teste (barramento)" << std::endl;

    Memory mem(64);
    Bus bus(mem);

    bus.write(10, 42);
    bus.write(11, 99);
    bus.write(12, 7);

    uint8_t v10 = bus.read(10);
    uint8_t v11 = bus.read(11);
    uint8_t v12 = bus.read(12);

    std::cout << "Valores lidos do barramento:" << std::endl;
    std::cout << "Endereco 10: " << (int)v10 << std::endl;
    std::cout << "Endereco 11: " << (int)v11 << std::endl;
    std::cout << "Endereco 12: " << (int)v12 << std::endl;

    mem.dump(8, 8);
    return 0;
}