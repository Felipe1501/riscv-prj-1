#pragma once

#include <cstdint>

namespace rv32i {
constexpr uint32_t OPCODE_OP_IMM = 0x13;
constexpr uint32_t OPCODE_OP = 0x33;
constexpr uint32_t OPCODE_LOAD = 0x03;
constexpr uint32_t OPCODE_STORE = 0x23;

constexpr uint32_t FUNCT3_ADDI = 0x0;
constexpr uint32_t FUNCT3_LW = 0x2;
constexpr uint32_t FUNCT3_SW = 0x2;
constexpr uint32_t FUNCT3_ADD_SUB = 0x0;
constexpr uint32_t FUNCT7_ADD = 0x00;
constexpr uint32_t FUNCT7_SUB = 0x20;
}  // namespace rv32i
