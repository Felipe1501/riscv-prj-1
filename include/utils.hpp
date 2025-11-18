#pragma once

#include <cstdint>

uint32_t encodeADDI(uint8_t rd, uint8_t rs1, int32_t imm);
uint32_t encodeLW(uint8_t rd, uint8_t rs1, int32_t imm);
uint32_t encodeSW(uint8_t rs2, uint8_t rs1, int32_t imm);
uint32_t encodeR(uint8_t rd, uint8_t rs1, uint8_t rs2, uint8_t funct3,
                 uint8_t funct7);
