#pragma once
#include "memory.hpp"
#include <cstdint>
#include <iostream>

class Bus{
    private:
        Memory& memory;
    
    public:
        explicit Bus(Memory& mem);
        uint8_t read(uint32_t address) const;
        void write(uint32_t address, uint8_t value);

};
       