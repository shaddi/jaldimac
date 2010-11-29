#pragma once

namespace jaldimac {

// Constants
const unsigned MASTER_ID = 0;
const unsigned DATA_MTU = 1500;
const unsigned VOIP_MTU = 300;
const uint8_t CURRENT_VERSION = 1;
const uint8_t PREAMBLE[4] = ['J', 'L', 'D', CURRENT_VERSION];

// Frame types
enum FrameType
{
	DATA_FRAME = 0,
	VOIP_FRAME,
	CONTENTION_SLOT,
	VOIP_SLOT,
	TRANSMIT_MESSAGE,
	RECEIVE_MESSAGE,
	BITRATE_MESSAGE,
	ROUND_COMPLETE_MESSAGE
};

struct Frame
{
	uint8_t preamble[4];
	uint8_t dest_id;
	uint8_t type;
	uint16_t length;
	uint32_t seq;

	// Payload.
	// For a DATA_FRAME or VOIP_FRAME, this is the actual data, up to MTU in size.
	// For a CONTENTION_SLOT, this is the duration of the contention slot.
	// For a VOIP_SLOT, this is a duration followed by 4 bytes enumerating
	// the stations which may transmit, in order.
	// For a TRANSMIT_MESSAGE, this is the duration for which the station may transmit.
	// For a RECEIVE_MESSAGE, this is the duration for which the driver must receive.
	// For a BITRATE_MESSAGE, this is the new bitrate to set.
	// For a ROUND_COMPLETE_MESSAGE, no payload is neccessary. (Only the footer.)
	// The payload has 64 bits of footer appended: 32 bits of CRC32 followed by 32 bits
    // of timestamp which is added by the driver and is not included in the CRC32.
	uint8_t payload[0];

	static const size_t header_size = sizeof(preamble) + sizeof(dest_id) + sizeof(type)
					+ sizeof(length) + sizeof(seq);
	static const size_t footer_size = sizeof(uint32_t) /* CRC32 */ + sizeof(uint32_t) /* timestamp */;

	void Initialize();
	void ComputeCRC();
	const size_t PayloadLength() const { return length - (header_size + footer_size); }
} __attribute__((__packed__));

}
