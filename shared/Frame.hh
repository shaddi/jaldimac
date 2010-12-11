#ifndef FRAME_HH
#define FRAME_HH

namespace jaldimac {

// Constants
const uint8_t BROADCAST_ID = 0;
const uint8_t DRIVER_ID = 0;
const uint8_t MASTER_ID = 1;
const uint8_t FIRST_STATION_ID = 2;
const unsigned DATA_MTU__BYTES = 1500;
const unsigned VOIP_MTU__BYTES = 300;
const unsigned FLOWS_PER_VOIP_SLOT = 4;
const uint8_t CURRENT_VERSION = 1;
const uint8_t PREAMBLE[4] = {'J', 'L', 'D', CURRENT_VERSION};

// Bitrate constants: (this is all temporary, for testing on Ethernet)
const unsigned MEGABIT__BYTES = 1000000 / 8;
const unsigned ETH_10_MEGABIT__BYTES_PER_US = (MEGABIT__BYTES * 10) / 1000000;
const unsigned ETH_100_MEGABIT__BYTES_PER_US = (MEGABIT__BYTES * 100) / 1000000;
const unsigned ETH_1000_MEGABIT__BYTES_PER_US = (MEGABIT__BYTES * 1000) / 1000000;
const unsigned BITRATE__BYTES_PER_US = ETH_100_MEGABIT__BYTES_PER_US;

// Station-related constants: (this is all temporary, for testing)
const unsigned STATION_COUNT = 4;

// Frame types:
// There are three general classes of frames. "Content" frames are sent over
// the air and exist to transfer data. "Local Control" frames are not sent over
// the air and exist to communicate between Click and the kernel driver.
// "Global Control" frames are broadcast, but also serve to give instructions
// to the driver. Each type of frame is categorized below.
enum FrameType
{
    DATA_FRAME = 0,         // Content
    VOIP_FRAME,             // Content
    REQUEST_FRAME,          // Content
    CONTENTION_SLOT,        // Global Control
    VOIP_SLOT,              // Global Control
    TRANSMIT_SLOT,          // Global Control
    BITRATE_MESSAGE,        // Local Control
    ROUND_COMPLETE_MESSAGE, // Local Control
    DELAY_MESSAGE           // Local Control
};

struct Frame
{
    // Fields
    uint8_t preamble[4];
    uint8_t src_id;
    uint8_t dest_id;
    uint8_t type;
    uint8_t tag;            // Currently unused
    uint32_t length;
    uint32_t seq;
    uint8_t payload[0];     // Actual size determined by length

    // Static constants
    static const size_t header_size = sizeof(preamble) + sizeof(dest_id) + sizeof(type)
                                    + sizeof(length) + sizeof(seq);
    static const size_t footer_size = sizeof(uint32_t) /* timestamp */;
    static const size_t empty_packet_size = header_size + footer_size;

    // Member functions
    inline void initialize();
    inline size_t payload_length() const { return length - empty_packet_size; }

} __attribute__((__packed__));

// Cast the payload to one of the following structs as appropriate for the
// frame type.  After the payload comes an additional 32 bit TX timestamp which
// is added by the driver; it is only used for debugging purposes and should
// not affect the semantics of the protocol.
// DATA_FRAME and VOIP_FRAME do not have a struct below as their payload consists
// of an encapsulated IP packet. ROUND_COMPLETE_MESSAGE does not have a struct
// because it requires no payload.

struct RequestFramePayload
{
    uint32_t bulk_request_bytes;
    uint8_t voip_request_flows;
} __attribute__((__packed__));

struct ContentionSlotPayload
{
    uint32_t duration_us;
} __attribute__((__packed__));

struct VoIPSlotPayload
{
    uint32_t duration_us;
    uint8_t stations[FLOWS_PER_VOIP_SLOT];
} __attribute__((__packed__));

struct TransmitSlotPayload
{
    uint32_t duration_us;
    uint8_t voip_granted_flows;
} __attribute__((__packed__));

struct BitrateMessagePayload
{
    /* REPLACE WITH BITRATE ENUM TYPE */ uint32_t bitrate;
} __attribute__((__packed__));

struct DelayMessagePayload
{
    uint32_t duration_us;
} __attribute__((__packed__));

inline void Frame::initialize()
{
    // Set up preamble
    memcpy(preamble, PREAMBLE, sizeof(PREAMBLE));

    // Set up defaults for other values
    src_id = 0;
    dest_id = 0;
    type = DATA_FRAME;
    tag = 0;
    length = empty_packet_size;
    seq = 0;
}

}

#endif
