#include "Frame.hh"

#include "CRC32.hh"

namespace jaldimac {

void Frame::Initialize()
{
	// Set up preamble
	memcpy(preamble, PREAMBLE, sizeof(PREAMBLE));

	// Set up defaults for other values
	dest_id = 0;
	type = DATA_FRAME;
	length = Frame::header_size + Frame::footer_size;
	seq = 0;
	timestamp = 0;
}

void Frame::ComputeCRC()
{
    unsigned payloadLength = PayloadLength();
    uint8_t* frameCRC = &payload[payloadLength];
    uint32_t crc = htonl(crc32((const uint8_t*) this, payloadLength + Frame::header_size));
    memcpy(frameCRC, &crc, sizeof(uint32_t));
}

}
