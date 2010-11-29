#pragma once

namespace jaldimac {

// Initialize CRC32 table if needed
void initcrc32();

// Calculate CRC32
uint32_t crc32(const uint8_t* block, unsigned length);

}
