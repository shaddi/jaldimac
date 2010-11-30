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

enum PacketType
{
	CONTROL = 0,
	DATA,
	BAD
};

static PacketType action(Packet* p)
{
    PacketType type;

	// Treat the packet as a Frame
	const Frame* f = (const Frame*) p->data();

	// Classify the packet (Control, Data, or Bad)?
	switch (f->type)
	{
		case DATA_FRAME:
		case VOIP_FRAME:
			type = DATA; break;

		case CONTENTION_SLOT:
		case VOIP_SLOT:
		case TRANSMIT_MESSAGE:
		case BITRATE_MESSAGE:
		case ROUND_COMPLETE_MESSAGE:
			type = CONTROL; break;

		default:
			type = BAD; break;
	}

	if (type == DATA)
	{
		// Strip Jaldi header and footer
		p->pull(Frame::HeaderSize);
		p->take(Frame::FooterSize);
	}

    return type;
}

void JaldiEncap::push(int port, Packet* p)
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
