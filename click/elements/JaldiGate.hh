#ifndef CLICK_JALDIGATE_HH
#define CLICK_JALDIGATE_HH
#include <click/element.hh>
CLICK_DECLS

/*
=c

JaldiGate

=s jaldi

sends Jaldi frames in response to control messages

=d

JaldiGate observes incoming control messages and sends data queued on its
inputs to its output when requested by the master. It's also responsible for
sending requests to the master when more data is queued than can be sent in the
current round. JaldiGate is, in some sense, the equivalent of JaldiScheduler
for stations.

JaldiGate has 3 inputs: input 0 (push) is for control traffic, input 1 (pull)
is for bulk traffic, and input 2 (pull) is for voip traffic. It has one push
output (though a second push output may be connected to receive erroneous
packets). Everything arriving on the inputs should be encapsulated in Jaldi
frames.

=a

JaldiGate */

class JaldiQueue;

class JaldiGate : public Element { public:

    JaldiGate();
    ~JaldiGate();

    const char* class_name() const  { return "JaldiGate"; }
    const char* port_count() const  { return "3/1-2"; }
    const char* processing() const  { return "hl/h"; }
    const char* flow_code() const   { return COMPLETE_FLOW; }

    int initialize(ErrorHandler*);
    bool can_live_reconfigure() const   { return true; }
    void take_state(Element*, ErrorHandler*);

    void push(int, Packet*);

  private:
    static const int in_port_control = 0;
    static const int in_port_bulk = 1;
    static const int in_port_voip = 2;
    static const int out_port = 0;
    static const int out_port_bad = 1;

    JaldiQueue* bulk_queue;
    JaldiQueue* voip_queue;
    bool outstanding_bulk_requests;
    uint32_t bulk_requested;
    uint32_t voip_requested;
    uint8_t station_id;
};

CLICK_ENDDECLS
#endif
