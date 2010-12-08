/*
 * JaldiGate.{cc,hh} -- sends packets to master at appropriate times based upon control packets
 */

// TODO: Remove unneeded comments about ports
// TODO: Add configuration parameter for station id - ABSOLUTELY NEEDED!
// TODO: Move make_jaldi_frame into separate header
// TODO: Make voip protocol work as intended - i.e., flow oriented rather than packet oriented

#include <click/config.h>
#include "JaldiGate.hh"
#include <click/confparse.hh>
#include <click/error.hh>
#include <click/glue.hh>
#include <click/router.hh>
#include <click/routervisitor.hh>
#include "JaldiQueue.hh"
#include "Frame.hh"

using namespace jaldimac;

CLICK_DECLS

JaldiGate::JaldiGate() : bulk_queue(NULL), voip_queue(NULL), outstanding_bulk_requests(false),
                         bulk_requested(0), voip_requested(0), station_id(0)
{
}

JaldiGate::~JaldiGate()
{
}

int JaldiGate::initialize(ErrorHandler* errh)
{
    // Seed PRNG
    srand(time(NULL));

    // Find the nearest upstream bulk queue
    ElementCastTracker filter(router(), "JaldiQueue");

    if (router()->visit_upstream(this, in_port_bulk, &filter) < 0 || filter.size() == 0)
        return errh->error("couldn't find an upstream bulk JaldiQueue using flow-based router context");

    if (! (bulk_queue = (JaldiQueue*) filter[0]->cast("JaldiQueue")))
        return errh->error("bulk queue %<%s%> is not a valid JaldiQueue (cast failed)", filter[0]->name().c_str());

    // Find the nearest upstream VoIP queue
    filter.clear();

    if (router()->visit_upstream(this, in_port_voip, &filter) < 0 || filter.size() == 0)
        return errh->error("couldn't find an upstream VoIP JaldiQueue using flow-based router context");

    if (! (voip_queue = (JaldiQueue*) filter[0]->cast("JaldiQueue")))
        return errh->error("VoIP queue %<%s%> is not a valid JaldiQueue (cast failed)", filter[0]->name().c_str());

    // Success!
    return 0;
}

void JaldiGate::take_state(Element* /*old*/, ErrorHandler* /*errh*/)
{
/*
    JaldiEncap* oldJE = (JaldiEncap*) old->cast("JaldiEncap");

    if (oldJE)
        seq = oldJE->seq;
*/
}

// Inputs: 0 (push) = control, 1 (pull) = bulk, 2 (pull) = voip
// Outputs: 0 (push) = everything

template<uint8_t FrameType, uint8_t DestId, typename PayloadType>
WritablePacket* make_jaldi_frame(PayloadType*& payload_out)
{
    WritablePacket* wp = Packet::make(Frame::empty_packet_size + sizeof(PayloadType));
    Frame* f = (Frame*) wp->data();
    f->initialize();
    f->type = FrameType;
    f->dest_id = DestId;
    f->length = Frame::empty_packet_size + sizeof(PayloadType);
    payload_out = (PayloadType*) f->payload;
    return wp;
}

void JaldiGate::push(int, Packet* p)
{
    // We've received some kind of control traffic; take action based on the
    // specific type and parameters.
    const Frame* f = (const Frame*) p->data();

    switch (f->type)
    {
        case CONTENTION_SLOT:
        {
            // Send requests if we need to and we won't get a chance later

            if (outstanding_bulk_requests)
                outstanding_bulk_requests = false;  // We'll make the requests when it's our turn to send
            else if (! (bulk_queue->empty() && voip_queue->empty()))
            {
                // Construct a request frame
                RequestFramePayload* rfp;
                WritablePacket* rp = make_jaldi_frame<REQUEST_FRAME, MASTER_ID>(rfp);

                // Configure the request according to the queue sizes
                rfp->bulk_request_bytes = bulk_queue->total_length();
                rfp->voip_request_bytes = voip_queue->total_length() - voip_requested;

                // Update state
                bulk_requested = rfp->bulk_request_bytes;
                voip_requested += rfp->voip_request_bytes;
		outstanding_bulk_requests = true;

                // If possible, create a delay message with a random delay within the contention slot
                const ContentionSlotPayload* payload = (const ContentionSlotPayload*) f->payload;
                uint32_t requested_duration_us = rp->length() / BITRATE__BYTES_PER_US + 1;

                if (requested_duration_us < payload->duration_us)
                {
                    // Construct and send a delay message frame
                    DelayMessagePayload* dmp;
                    WritablePacket* dp = make_jaldi_frame<DELAY_MESSAGE, DRIVER_ID>(dmp);
                    dmp->duration_us = rand() % (payload->duration_us - requested_duration_us + 1);
                    output(out_port).push(dp);
                }

                // Send it
                output(out_port).push(rp);

            }

            p->kill();

            break;
        }

        case VOIP_SLOT:
        {
            // Send a VoIP packet if the master has given us a chance to do so

            const VoIPSlotPayload* payload = (const VoIPSlotPayload*) f->payload;

            for (unsigned i = 0 ; i < STATIONS_PER_VOIP_SLOT ; ++i)
            {
                if (payload->stations[i] == station_id)
                {
                    Packet* vp = input(in_port_voip).pull();
                    voip_requested -= vp->length();
                    output(out_port).push(vp);                      // Send one of our VoIP packets
                }
                else
                {
                    // Construct a delay message frame for the driver
                    DelayMessagePayload* dmp;
                    WritablePacket* dp = make_jaldi_frame<DELAY_MESSAGE, DRIVER_ID>(dmp);
                    dmp->duration_us = VOIP_MTU__BYTES / BITRATE__BYTES_PER_US + 1;

                    // Send it
                    output(out_port).push(dp);
                }
            }

            p->kill();

            break;
        }

        case TRANSMIT_SLOT:
        {
            // If we have requests or bulk data, send them
            // (In the future we potentially need to send VOIP stuff here too, but can wait to implement that)
            
            const TransmitSlotPayload* payload = (const TransmitSlotPayload*) f->payload;
            uint32_t duration_us = payload->duration_us;

            unsigned bulk_queued = bulk_queue->total_length();
            unsigned voip_queued = voip_queue->total_length();

            if (bulk_queued > bulk_requested || voip_queued > voip_requested)
            {
                // Construct a request frame
                RequestFramePayload* rfp;
                WritablePacket* rp = make_jaldi_frame<REQUEST_FRAME, MASTER_ID>(rfp);

                // Configure the request according to the queue sizes
                rfp->bulk_request_bytes = bulk_queued - bulk_requested;
                rfp->voip_request_bytes = voip_queued - voip_requested;

                // Send it
                output(out_port).push(rp);

                // Update state
		if (bulk_queued > bulk_requested)
			outstanding_bulk_requests = true;

                bulk_requested = bulk_queued;
                voip_requested = voip_queued;
                duration_us -= rp->length() / BITRATE__BYTES_PER_US + 1;
            }

            // Send bulk frames
            uint32_t next_frame_duration_us;
            while ((next_frame_duration_us = bulk_queue->head_length() / BITRATE__BYTES_PER_US + 1) > duration_us)
            {
                // Pull it, update stats, and send it
                Packet* bp = input(in_port_bulk).pull();
                bulk_requested -= bp->length();
                output(out_port).push(bp);

                // Update remaining duration
                duration_us -= next_frame_duration_us;
            }

            p->kill();

            break;
        }

        default:
        {
            // Bad stuff; dump it out the optional output
            checked_output_push(out_port_bad, p);
            break;
        }
    }
}

CLICK_ENDDECLS
EXPORT_ELEMENT(JaldiGate)
