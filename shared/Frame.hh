#pragma once

namespace jaldimac {

// Constants
const uint8_t BROADCAST_ID = 0;
const uint8_t MASTER_ID = 1;
const unsigned DATA_MTU = 1500;
const unsigned VOIP_MTU = 300;
const uint8_t CURRENT_VERSION = 1;
const uint8_t PREAMBLE[4] = {'J', 'L', 'D', CURRENT_VERSION};

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
    CONTENTION_SLOT,        // Global Control
    VOIP_SLOT,              // Global Control
    TRANSMIT_SLOT,          // Global Control
    BITRATE_MESSAGE,        // Local Control
    ROUND_COMPLETE_MESSAGE  // Local Control
};

struct Frame
{
    // Fields
    uint8_t preamble[4];
    uint8_t dest_id;
    uint8_t type;
    uint16_t length;
    uint32_t seq;
    uint8_t payload[0];     // Actual size determined by length

    // Static constants
    static const size_t header_size = sizeof(preamble) + sizeof(dest_id) + sizeof(type)
                                    + sizeof(length) + sizeof(seq);
    static const size_t footer_size = sizeof(uint32_t) /* timestamp */;

    // Member functions
    template<uint8_t Type>
    void Initialize()
    {
        // Set up preamble
        memcpy(preamble, PREAMBLE, sizeof(PREAMBLE));

        // Set up defaults for other values
        dest_id = 0;
        type = Type;
        length = Frame::header_size + Frame::footer_size;
        seq = 0;
    }

    size_t PayloadLength() const { return length - (header_size + footer_size); }
} __attribute__((__packed__));

// Cast the payload to one of the following structs as appropriate for the
// frame type.  After the payload comes an additional 32 bit TX timestamp which
// is added by the driver; it is only used for debugging purposes and should
// not affect the semantics of the protocol.
// DATA_FRAME and VOIP_FRAME do not have a struct below as their payload consists
// of an encapsulated IP packet. ROUND_COMPLETE_MESSAGE does not have a struct
// because it requires no payload.

struct ContentionSlotPayload
{
    uint32_t duration_us;
} __attribute__((__packed__));

struct VoIPSlotPayload
{
    uint32_t duration_us;
    uint8_t dest_ids[4];
} __attribute__((__packed__));

struct TransmitSlotPayload
{
    uint32_t duration_us;
} __attribute__((__packed__));

struct BitrateMessagePayload
{
    /* REPLACE WITH BITRATE ENUM TYPE */ uint32_t bitrate;
} __attribute__((__packed__));

}
