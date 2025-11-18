#include "cpu.hpp"

#include "isa.hpp"

CPU::CPU(Bus& b) 
    : bus(b) {
    reset();
}

void CPU::reset() {
    x.fill(0);
    pc = 0;
    std::cout << "CPU resetada. \n";
}

void CPU::dump() const {
    std::cout << "[CPU] Estado atual:\n";
    std::cout << "PC = " << pc << std::endl;
    for (int i = 0; i < 32; i++) {
        std::cout << "x" << i << " = " << x[i] << "\t";
        if ((i + 1) % 8 == 0) {
            std::cout << "\n";
        }
    }
}

uint32_t CPU::getBits(uint32_t value, int hi, int lo) const {
    uint32_t mask = ((1u << (hi - lo + 1)) - 1u);
    return (value >> lo) & mask;
}

int32_t CPU::signExtend(uint32_t value, int bits) const{
    int32_t s = static_cast<int32_t>(value);
    int shift = 32 - bits;
    return (s << shift) >> shift;
}

void CPU::step(){
    std::cout << "[CPU] Executando ciclo no @PC = " << pc << std::endl;

    uint32_t inst = bus.read32(pc);
    std::cout << "[CPU] Instrucao lida: 0x" << std::hex << inst << std::dec << std::endl;

    uint32_t opcode = getBits(inst, 6, 0);
    uint32_t rd     = getBits(inst, 11, 7);
    uint32_t funct3 = getBits(inst, 14, 12);
    uint32_t rs1    = getBits(inst, 19, 15);
    uint32_t rs2    = getBits(inst, 24, 20);
    uint32_t funct7 = getBits(inst, 31, 25);


    std::cout << "[CPU] opcode= " << opcode 
              << ", rd=" << rd 
              << ", funct3=" << funct3 
              << ", rs1=" << rs1 
              << ", rs2=" << rs2
              << ", funct7=" << funct7 << std::endl;
    
    if (opcode == rv32i::OPCODE_OP_IMM){
        uint32_t imm12 = getBits(inst, 31, 20);
        int32_t imm = signExtend(imm12, 12);

        std::cout << "[CPU] (I-type) imm=" << imm << std::endl;

        if (funct3 == rv32i::FUNCT3_ADDI){
            uint32_t old = x[rs1];
            uint32_t result = old + imm;

            if(rd != 0){
              x[rd] = result;
            }
           std::cout << "[CPU] Executando ADDI: x" << rd
                     << " = x" << rs1 << "(" << old << ") + "
                     << imm << " -> " << result << std::endl;
        }else{
            std::cout << "[CPU] I-type nao implementado.\n";
        }
    } else if (opcode == rv32i::OPCODE_OP){
        if (funct3 == rv32i::FUNCT3_ADD_SUB && funct7 == rv32i::FUNCT7_ADD){
            uint32_t a = x[rs1];
            uint32_t b = x[rs2];
            uint32_t result = a + b;
            if(rd != 0){
              x[rd] = result;
            }
            std::cout << "[CPU] Executando ADD: x" << rd
                      << " = x" << rs1 << "(" << a << ") + "
                      << "x" << rs2 << "(" << b << ") -> " 
                      << result << std::endl;
        } else if(funct3 == rv32i::FUNCT3_ADD_SUB && funct7 == rv32i::FUNCT7_SUB){
            uint32_t a = x[rs1];
            uint32_t b = x[rs2];
            uint32_t result = a - b;
            if(rd != 0){
              x[rd] = result;
            }
            std::cout << "[CPU] Executando SUB: x" << rd
                      << " = x" << rs1 << "(" << a << ") - "
                      << "x" << rs2 << "(" << b << ") -> " 
                      << result << std::endl;
        }else {
            std::cout << "[CPU] R-type nao implementado.\n";
        }
    } else if (opcode == rv32i::OPCODE_LOAD){
        uint32_t imm12 = getBits(inst, 31, 20);
        int32_t imm = signExtend(imm12, 12);
        uint32_t addr = x[rs1] + imm;

        if (funct3 == rv32i::FUNCT3_LW){
            if(addr % 4 != 0){
                std::cout << "[CPU] EXC: LW desalinhado em " << addr << " (ignorado)\n";
            }else {
                uint32_t value = bus.read32(addr);
                if(rd != 0){
                  x[rd] = value;
                }
                std::cout << "[CPU] Executando LW: x" << rd
                          << " = MEM[" << addr << "] -> " 
                          << value << std::endl;
            }
        } else {
            std::cout << "[CPU] Load nao implementado.\n";
        }

    }else if(opcode == rv32i::OPCODE_STORE){
        uint32_t imm_high = getBits(inst, 31, 25);
        uint32_t imm_low = getBits(inst, 11, 7);
        uint32_t imm12u = (imm_high << 5) | imm_low;
        int32_t imm = signExtend(imm12u, 12);
        uint32_t addr = x[rs1] + imm;

        if(funct3 == rv32i::FUNCT3_SW){
            if(addr % 4 != 0){
                std::cout <<  "[CPU] EXC: SW desalinhado em " << addr << " (ignorado)\n";
            } else {
                uint32_t value = x[rs2];
                bus.write32(addr, value);
                std::cout << "[CPU] Executando SW: MEM[" << addr << "] = x" 
                          << rs2 << "(" << value << ")\n";
            }
        } else{
            std::cout << "[CPU] Store nao implementado.\n";
        }
    }else{
        std::cout << "[CPU] Instrucao desconhecida. NOP.\n";
    }
    pc += 4;
    x[0] = 0;
}