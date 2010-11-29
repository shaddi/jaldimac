/*
 * JaldiEncap.{cc,hh} -- encapsulates packet in Jaldi header
 */

#include <click/config.h>
#include "JaldiEncap.hh"
#include <click/confparse.hh>
#include <click/error.hh>
#include <click/glue.hh>
#include "Frame.hh"

using namespace jaldimac;

CLICK_DECLS

JaldiEncap::JaldiEncap()
{
}

JaldiEncap::~JaldiEncap()
{
}

int JaldiEncap::configure(Vector<String>& conf, ErrorHandler* errh)
{
    String name_of_type;

    // Parse configuration parameters
    if (cp_va_kparse(conf, this, errh,
		     "TYPE", cpkP+cpkM, cpString, &name_of_type,
		     "DST", cpkP+cpkM, cpByte, &dest_id,
		     cpEnd) < 0)
        return -1;

    // Convert TYPE field from a string to the appropriate code
    if (name_of_type.equals("DATA_FRAME", -1))
        type = DATA_FRAME;
    else if (name_of_type.equals("VOIP_FRAME", -1))
        type = VOIP_FRAME;
    else if (name_of_type.equals("CONTENTION_SLOT", -1))
        type = CONTENTION_SLOT;
    else if (name_of_type.equals("VOIP_SLOT", -1))
        type = VOIP_SLOT;
    else if (name_of_type.equals("TRANSMIT_MESSAGE", -1))
        type = TRANSMIT_MESSAGE;
    else if (name_of_type.equals("RECEIVE_MESSAGE", -1))
        type = RECEIVE_MESSAGE;
    else if (name_of_type.equals("BITRATE_MESSAGE", -1))
        type = BITRATE_MESSAGE;
    else if (name_of_type.equals("ROUND_COMPLETE_MESSAGE", -1))
        type = ROUND_COMPLETE_MESSAGE;
    else
    {
        errh->error("invalid Jaldi frame type: %s", name_of_type.c_str());
        return -1;
    }

    // Initialize the sequence number to 0
    seq = 0;

    return 0;
}

void JaldiEncap::take_state(Element* old, ErrorHandler* errh)
{
    JaldiEncap* oldJE = (JaldiEncap*) old->cast("JaldiEncap");

    if (oldJE)
        seq = oldJE->seq;
}

Packet* JaldiEncap::action(Packet* p)
{
    // Remember the "real" length of this packet
    uint32_t length = p->length();

    // If the packet's too long, kill it
    if (length > UINT16_MAX)
    {
        p->kill();
        return NULL;
    }

    // Add space for Jaldi frame header and footer to packet
    WriteablePacket* p0 = p->push(Frame::HeaderSize);
    WriteablePacket* p1 = p0->put(Frame::FooterSize);

    // Create header
    Frame* f = (Frame*) p1->data();
    f->Initialize();
    f->dest_id = dest_id;
    f->type = type;
    f->length = p1->length() + Frame::HeaderSize + Frame::FooterSize;
    f->seq = seq++;

    // Create footer (actually only the CRC; the timestamp will be added by the driver)
    f->ComputeCRC();

    // Kill intermediate packets that are now unnecessary
    p->kill();
    p0->kill();

    // Return the final encapsulated packet
    return p1;
}

void JaldiEncap::push(int port, Packet* p)
{
    if (Packet* q = action(p))
        output(0).push(q);
}

Packet* JaldiEncap::pull(int port)
{
    if (Packet *p = input(0).pull())
        return action(p);
    else
        return NULL;
}

CLICK_ENDDECLS
EXPORT_ELEMENT(JaldiEncap)
