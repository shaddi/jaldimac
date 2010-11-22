#pragma once

namespace jaldimac {

// Constants
#define MASTER_ID 0
#define DATA_MTU 1500
#define VOIP_MTU 300

// Frame types
#define DATA_FRAME 0
#define VOIP_FRAME 1
#define CONTENTION_SLOT 2
#define VOIP_SLOT 3
#define TRANSMIT_MESSAGE 4
#define RECEIVE_MESSAGE 5
#define BITRATE_MESSAGE 6

struct frame
{
	const uint8_t preamble = ['J', 'L', 'D', 'I'];
	uint8_t dest_id;
	uint8_t type;
	uint16_t length;
	uint32_t timestamp;		// Leave at -1 or 0, add timestamp of intended time to TCP packet

	// Payload.
	// For a DATA_FRAME or VOIP_FRAME, this is the actual data, up to MTU in size.
	// For a CONTENTION_SLOT, this is the duration of the contention slot.
	// For a VOIP_SLOT, this is a duration followed by 4 bytes enumerating
	// the stations which may transmit, in order.
	// For a TRANSMIT_MESSAGE, this is the duration for which the station may transmit.
	// For a RECEIVE_MESSAGE, this is the duration for which the driver must receive.
	// For a BITRATE_MESSAGE, this is the new bitrate to set.
	// The last 32 bits of the payload are the checksum.
	uint8_t payload[0];
} __attribute__((__packed__));

}
