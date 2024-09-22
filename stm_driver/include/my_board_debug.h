#ifndef MY_BOARD_DEBUG_H
#define MY_BOARD_DEBUG_H
#include<iostream>

uint8_t checksum(uint8_t data[], int len);

uint8_t reverseBits(uint8_t value);

uint8_t crc8(uint8_t* data, size_t length, uint8_t poly, uint8_t init, uint8_t refIn, uint8_t refOut, uint8_t xorOut);

uint16_t crc16(uint8_t* data, size_t length, uint16_t poly, uint16_t init, uint8_t refIn, uint8_t refOut, uint16_t xorOut);

uint32_t crc32(uint8_t* data, size_t length, uint32_t poly, uint32_t init, uint8_t refIn, uint8_t refOut, uint32_t xorOut) ;

#endif