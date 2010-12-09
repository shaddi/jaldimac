#ifndef JALDI_GATE_HH
#define JALDI_GATE_HH

#include <click/packet.hh>
#include "Frame.hh"

template<uint8_t FrameType, uint8_t DestId, typename PayloadType>
WritablePacket* make_jaldi_frame(PayloadType*& payload_out)
{
    WritablePacket* wp = Packet::make(jaldimac::Frame::empty_packet_size + sizeof(PayloadType));
    jaldimac::Frame* f = (jaldimac::Frame*) wp->data();
    f->initialize();
    f->type = FrameType;
    f->dest_id = DestId;
    f->length = jaldimac::Frame::empty_packet_size + sizeof(PayloadType);
    payload_out = (PayloadType*) f->payload;
    return wp;
}

#endif
