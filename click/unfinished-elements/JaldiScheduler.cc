/*
 * JaldiScheduler.{cc,hh} -- constructs a Jaldi round layout from incoming packets and requests
 */

// TODO: Make voip protocol work as intended - i.e., flow oriented rather than packet oriented

#include <click/config.h>
#include <click/confparse.hh>
#include <click/error.hh>
#include <click/glue.hh>
#include <click/router.hh>
#include <click/routervisitor.hh>

#include "JaldiClick.hh"
#include "JaldiQueue.hh"
#include "Frame.hh"
#include "JaldiGate.hh"

using namespace jaldimac;

CLICK_DECLS

JaldiScheduler::JaldiScheduler() : bulk_queue(NULL), voip_queue(NULL),
				   outstanding_bulk_requests(false), bulk_requested(0),
				   voip_requested(0), station_id(0)
{
}

JaldiScheduler::~JaldiScheduler()
{
}

int JaldiScheduler::configure(Vector<String>& conf, ErrorHandler* errh)
{
    // We should have 2 input ports for every station and 1 control input
    if (ninputs() != STATION_COUNT * 2 + 1)
        return errh->error("wrong number of input ports connected; need a control port and, for each station, a bulk port and a VoIP port");
    else
        return 0;
}

int JaldiScheduler::initialize(ErrorHandler* errh)
{
    // Find the nearest upstream queues
    ElementCastTracker filter(router(), "JaldiQueue");

    for (unsigned station = 0 ; station < STATION_COUNT ; ++station)
    {
        // Get bulk queue
        filter.clear();

        if (router()->visit_upstream(this, in_ports_bulk[station], &filter) < 0 || filter.size() == 0)
            return errh->error("couldn't find an upstream bulk JaldiQueue on input port %<%d%> using flow-based router context", in_ports_bulk[station]);

        if (! (bulk_queues[station] = (JaldiQueue*) filter[0]->cast("JaldiQueue")))
            return errh->error("bulk queue %<%s%> found on input port %<%d%> is not a valid JaldiQueue (cast failed)", filter[0]->name().c_str(), in_ports_bulk[station]);

        // Get VoIP queue
        filter.clear();

        if (router()->visit_upstream(this, in_ports_voip[station], &filter) < 0 || filter.size() == 0)
            return errh->error("couldn't find an upstream VoIP JaldiQueue on input port %<%d%> using flow-based router context", in_ports_voip[station]);

        if (! (voip_queues[station] = (JaldiQueue*) filter[0]->cast("JaldiQueue")))
            return errh->error("VoIP queue %<%s%> found on input port %<%d%> is not a valid JaldiQueue (cast failed)", filter[0]->name().c_str(), in_ports_voip[station]);
    }

    // Success!
    return 0;
}

/*
void JaldiGate::take_state(Element* old, ErrorHandler*)
{
    JaldiGate* oldJG = (JaldiGate*) old->cast("JaldiGate");

    if (oldJG)
    {
        outstanding_bulk_requests = oldJG->outstanding_bulk_requests;
        bulk_requested = oldJG->bulk_requested;
        voip_requested = oldJG->voip_requested;
        station_id = oldJG->station_id;
    }
}
*/

void JaldiScheduler::push(int, Packet* p)
{
    // We've received some kind of control traffic; take action based on the
    // specific type and parameters.
    const Frame* f = (const Frame*) p->data();

    if (! (f->dest_id == BROADCAST_ID || f->dest_id == MASTER_ID))
    {
        // Not for us! Dump it out the optional output port
        checked_output_push(out_port_bad, p);
        return;
    }

    switch (f->type)
    {
        case REQUEST_FRAME:
        {
            uint8_t station_idx = f->src_id - FIRST_STATION_ID;

            if (f->src_id < FIRST_STATION_ID || station_idx >= STATION_COUNT)
            {
                // Invalid station! dump it out the optional output port
                checked_output_push(out_port_bad, p);
                return;
            }

            // Update requests
            const RequestFramePayload* rfp = (const RequestFramePayload*) f->payload;

            bulk_requested_bytes[station_idx] += rfp->bulk_request_bytes;
            voip_requested_bytes[station_idx] += rfp->voip_request_bytes;

            p->kill();

            break;
        }

        case ROUND_COMPLETE_MESSAGE:
        {
            // All requests have been received, and all upstream traffic eligible
            // for distribution this round is in the queues. It's time to compute
            // the layout for the next round.

            p->kill();

            // Need to:
            // 1. Count the frames destined for each station in the queues.
            // 2. Run a fairness algorithm over the upstream frames and requests
            //    to determine the allocation each station will receive.
            // 3. Actually compute a layout based on this allocation.

            count_upstream_bytes();
            compute_fair_allocation();
            generate_layout();

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

void JaldiScheduler::count_upstream_bytes()
{
    // Look in each queue and record their total size in bytes.
    // FIXME: This will need to be changed once we add bulk ACKs.
    for (unsigned station = 0 ; station < STATION_COUNT ; ++station)
    {
        bulk_upstream_bytes[station] = bulk_queues[station]->total_length();
        voip_upstream_bytes[station] = voip_queues[station]->total_length();
    }
}

void JaldiScheduler::compute_fair_allocation()
{
    // We now have all upstream and downstream requests; now we need to
    // decide on an allocation for each station (and each direction) that
    // satisfies the constraints we're operating under (such as the maximum
    // round size) and is in some sense "fair".

    // VoIP is simple: we cycle through the stations, assigning VOIP_MTU bytes
    // for each station that has an outstanding VoIP request, until we're out
    // of VoIP slots or requests. Note that this won't work that well unless
    // stations request VOIP_MTU bytes for every VoIP requests, but it's not
    // a big deal as this will be fixed when we move to flow-based VoIP
    // requests.
    bool more_voip_requests = true;
    unsigned available_voip_flows = DISTINCT_VOIP_FLOWS_PER_ROUND;
    for (unsigned station = 0 ; ; station = (station + 1) % STATION_COUNT)
    {
        if (station == 0)
        {
            if (more_voip_requests)
                more_voip_requests = false;
            else
                break;
        }

        
    }
}

CLICK_ENDDECLS
EXPORT_ELEMENT(JaldiScheduler)
