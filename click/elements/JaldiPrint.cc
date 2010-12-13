/*
 * JaldiPrint.{cc,hh} -- prints Jaldi frames
 */

#define __STDC_LIMIT_MACROS
#include <stdint.h>
#include <click/config.h>
#include <click/confparse.hh>
#include <click/error.hh>
#include <click/glue.hh>
#include <algorithm>

#include "JaldiPrint.hh"
#include "Frame.hh"

using namespace jaldimac;
using namespace std;

CLICK_DECLS

JaldiPrint::JaldiPrint()
{
}

JaldiPrint::~JaldiPrint()
{
}

int JaldiPrint::configure(Vector<String>& conf, ErrorHandler* errh)
{
    bool contents_supplied = false;

    // Parse configuration parameters
    if (cp_va_kparse(conf, this, errh,
             "CONTENTS", cpkP+cpkC, &contents_supplied, cpBool, &show_contents,
             cpEnd) < 0)
        return -1;

    if (! contents_supplied)
        show_contents = false;

    return 0;
}

Packet* JaldiPrint::action(Packet* p)
{
    // Treat the packet as a Frame
    const Frame* f = (const Frame*) p->data();

    // Print header
    click_chatter("===========================");

    click_chatter("Preamble: %c%c%c%u    Source: %u    Dest: %u\n", f->preamble[0], f->preamble[1], f->preamble[2], unsigned(f->preamble[3]), f->src_id, f->dest_id);

    switch (f->type)
    {
        case BULK_FRAME:
            click_chatter("Type: BULK_FRAME\n"); break;

        case VOIP_FRAME:
            click_chatter("Type: VOIP_FRAME\n"); break;

        case REQUEST_FRAME:
            click_chatter("Type: REQUEST_FRAME\n"); break;

        case CONTENTION_SLOT:
            click_chatter("Type: CONTENTION_SLOT\n"); break;

        case VOIP_SLOT:
            click_chatter("Type: VOIP_SLOT\n"); break;

        case TRANSMIT_SLOT:
            click_chatter("Type: TRANSMIT_SLOT\n"); break;

        case BITRATE_MESSAGE:
            click_chatter("Type: BITRATE_MESSAGE\n"); break;

        case ROUND_COMPLETE_MESSAGE:
            click_chatter("Type: ROUND_COMPLETE_MESSAGE\n"); break;

        case DELAY_MESSAGE:
            click_chatter("Type: DELAY_MESSAGE\n"); break;

        default:
            click_chatter("Type: <<<UNKNOWN TYPE>>>\n"); break;
    }

    click_chatter("Tag: %u    Length: %u    Payload Length: %u    Sequence Number: %u\n",
                  unsigned(f->tag), unsigned(f->length),
                  unsigned(f->payload_length()), unsigned(f->seq));

    // Print payload (if requested)
    if (show_contents)
    {
        char buffer[5000];
        char* buf = buffer;
        unsigned length = min(f->payload_length(), (const uint32_t) 2000);
        const uint8_t* payload = f->payload;

        for (unsigned i = 0 ; i < length ; ++i)
        {
            if (i && (i % 4) == 0)
                *buf++ = ' ';

            sprintf(buf, "%02x", *payload++ & 0xff);
            buf += 2;
        }

        *buf = '\0';

        click_chatter("Payload: %s\n", buffer);
    }

    click_chatter("===========================");

    return p;
}

void JaldiPrint::push(int, Packet* p)
{
    if (Packet* q = action(p))
        output(out_port).push(q);
}

Packet* JaldiPrint::pull(int)
{
    if (Packet *p = input(in_port).pull())
        return action(p);
    else
        return NULL;
}

CLICK_ENDDECLS
ELEMENT_REQUIRES(Frame)
EXPORT_ELEMENT(JaldiPrint)
