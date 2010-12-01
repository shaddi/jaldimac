/*
 * JaldiEncap.{cc,hh} -- encapsulates packet in Jaldi header
 */

#include <click/config.h>
#include "JaldiGate.hh"
#include <click/confparse.hh>
#include <click/error.hh>
#include <click/glue.hh>
#include "Frame.hh"

using namespace jaldimac;

CLICK_DECLS

JaldiGate::JaldiGate()
{
}

JaldiGate::~JaldiGate()
{
}

void JaldiGate::take_state(Element* old, ErrorHandler* errh)
{
/*
    JaldiEncap* oldJE = (JaldiEncap*) old->cast("JaldiEncap");

    if (oldJE)
        seq = oldJE->seq;
*/
}

static Packet* action(Packet* p)
{
    // Remember the "real" length of this packet
    uint32_t length = p->length();

    // If the packet's too long, kill it
    if (length > UINT16_MAX)
    {
        if (port_active(true, 1))
            output(1).push(p);
        else
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

// Inputs: 0 (push) = control, 1 (pull) = bulk, 2 (pull) = voip
// Outputs: 0 (push) = everything

void JaldiEncap::push(int port, Packet* p)
{
    // We've received some kind of control traffic; take action based on the
    // specific type and parameters.
    const Frame* f = (const Frame*) p->data();

    switch (f->type)
    {
        case CONTENTION_SLOT:
            // If we made no requests in the previous round, make requests here
            // If we made requests in the previous round, forget that fact
            break;

        case VOIP_SLOT:
            // If we have VOIP stuff, send it out
            break;

        case TRANSMIT_SLOT:
            // If we have requests, send them
            // If we have BULK stuff, send it out
            // (In the future we potentially need to send VOIP stuff here too, but can wait to implement that)
            break;

        default:
            // Bad stuff; dump it out the optional output
            break;
    }

    if (Packet* q = action(p))
        output(0).push(q);
}

CLICK_ENDDECLS
EXPORT_ELEMENT(JaldiGate)
