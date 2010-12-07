/*
 * JaldiDecap.{cc,hh} -- decapsulates Jaldi frame
 */

#include <click/config.h>
#include "JaldiDecap.hh"
#include <click/confparse.hh>
#include <click/error.hh>
#include <click/glue.hh>
#include "Frame.hh"

using namespace jaldimac;

CLICK_DECLS

JaldiDecap::JaldiDecap()
{
}

JaldiDecap::~JaldiDecap()
{
}

int JaldiDecap::configure(Vector<String>& conf, ErrorHandler* errh)
{
    // Parse configuration parameters
    if (cp_va_kparse(conf, this, errh,
             "DEST", cpkP+cpkC, &should_filter_by_dest, cpByte, &dest_id,
             cpEnd) < 0)
        return -1;
    else
        return 0;
}

JaldiDecap::PacketType JaldiDecap::action(Packet* p)
{
    // Treat the packet as a Frame
    const Frame* f = (const Frame*) p->data();

    // Filter by dest_id if requested
    if (should_filter_by_dest && !(f->dest_id == BROADCAST_ID || f->dest_id == dest_id))
        return BAD;

    // Classify the packet (Control, Data, or Bad)?
    switch (f->type)
    {
        case DATA_FRAME:
        case VOIP_FRAME:
            // Strip Jaldi header and footer
            p->pull(Frame::header_size);
            p->take(Frame::footer_size);
            return DATA;

        case CONTENTION_SLOT:
        case VOIP_SLOT:
        case TRANSMIT_SLOT:
        case BITRATE_MESSAGE:
        case ROUND_COMPLETE_MESSAGE:
            return CONTROL;

        default:
            return BAD;
    }
}

void JaldiDecap::push(int, Packet* p)
{
    switch (action(p))
    {
        case CONTROL:
            output(0).push(p); break;

        case DATA:
            output(1).push(p); break;

        default:
            if (port_active(true, 2))
                output(2).push(p);
            else
                p->kill();
            break;
    }
}

CLICK_ENDDECLS
EXPORT_ELEMENT(JaldiDecap)
