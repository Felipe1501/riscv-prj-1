#include "utils.hpp"

#include "isa.hpp"

uint32_t encodeADDI(uint8_t rd, uint8_t rs1, int32_t imm) {
    const uint32_t imm12 = static_cast<uint32_t>(imm) & 0xFFFu;

    return (imm12 << 20) | (static_cast<uint32_t>(rs1) << 15) |
           (rv32i::FUNCT3_ADDI << 12) | (static_cast<uint32_t>(rd) << 7) |
           rv32i::OPCODE_OP_IMM;
}

uint32_t encodeLW(uint8_t rd, uint8_t rs1, int32_t imm) {
    const uint32_t imm12 = static_cast<uint32_t>(imm) & 0xFFFu;

    return (imm12 << 20) | (static_cast<uint32_t>(rs1) << 15) |
           (rv32i::FUNCT3_LW << 12) | (static_cast<uint32_t>(rd) << 7) |
           rv32i::OPCODE_LOAD;
}

uint32_t encodeSW(uint8_t rs2, uint8_t rs1, int32_t imm) {
    const uint32_t imm12 = static_cast<uint32_t>(imm) & 0xFFFu;
    const uint32_t imm_low = imm12 & 0x1Fu;
    const uint32_t imm_high = (imm12 >> 5) & 0x7Fu;

    return (imm_high << 25) | (static_cast<uint32_t>(rs2) << 20) |
           (static_cast<uint32_t>(rs1) << 15) | (rv32i::FUNCT3_SW << 12) |
           (imm_low << 7) | rv32i::OPCODE_STORE;
}

uint32_t encodeR(uint8_t rd, uint8_t rs1, uint8_t rs2, uint8_t funct3,
                 uint8_t funct7) {
    return (static_cast<uint32_t>(funct7) << 25) |
           (static_cast<uint32_t>(rs2) << 20) |
           (static_cast<uint32_t>(rs1) << 15) |
           (static_cast<uint32_t>(funct3) << 12) |
           (static_cast<uint32_t>(rd) << 7) | rv32i::OPCODE_OP;
}
