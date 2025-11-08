#include "bus.hpp"

Bus::Bus(Memory& mem) 
        : memory(mem) {}

uint8_t Bus::read(uint32_t address) const {
    std::cout << "Bus: lendo endereco " << address << std::endl;
    return memory.read(address);
}

void Bus::write(uint32_t address, uint8_t value) {
    std::cout << "Bus: escrevendo valor " << (int)value 
              << " no endereco " << address << std::endl;
    memory.write(address, value);
}