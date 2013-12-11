
#include <string.h>
#include "Storage.h"

using namespace RoverBaseboard;

RoverBaseboardStorage::RoverBaseboardStorage()
{}

void RoverBaseboardStorage::init(void*)
{}

uint8_t RoverBaseboardStorage::read_byte(uint16_t loc){
    return 0;
}

uint16_t RoverBaseboardStorage::read_word(uint16_t loc){
    return 0;
}

uint32_t RoverBaseboardStorage::read_dword(uint16_t loc){
    return 0;
}

void RoverBaseboardStorage::read_block(void* dst, uint16_t src, size_t n) {
    memset(dst, 0, n);
}

void RoverBaseboardStorage::write_byte(uint16_t loc, uint8_t value)
{}

void RoverBaseboardStorage::write_word(uint16_t loc, uint16_t value)
{}

void RoverBaseboardStorage::write_dword(uint16_t loc, uint32_t value)
{}

void RoverBaseboardStorage::write_block(uint16_t loc, const void* src, size_t n)
{}

