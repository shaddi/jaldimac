#ifndef CLICK_JALDISCHEDULER_HH
#define CLICK_JALDISCHEDULER_HH
#include <click/element.hh>
CLICK_DECLS

/*
=c

JaldiScheduler

=s jaldi

constructs a Jaldi round layout from incoming packets and requests

=d

JaldiScheduler constructs a Jaldi round layout (expressed in the form of a
sequence of Jaldi frames that direct the driver, which actually implements the
requested behavior) from incoming packets from the Internet and incoming
requests from stations.

JaldiScheduler has 3 inputs: input 0 (push) is for control traffic, input 1
(pull) is for bulk traffic, and input 2 (pull) is for voip traffic. It has one
push output (though a second push output may be connected to receive erroneous
packets). Everything arriving on the inputs should be encapsulated in Jaldi
frames.

=a

JaldiGate */

class JaldiQueue;

class JaldiScheduler : public Element { public:

    JaldiScheduler();
    ~JaldiScheduler();

    const char* class_name() const  { return "JaldiScheduler"; }
    const char* port_count() const  { return "1-/1-2"; }
    const char* processing() const  { return "hl/h"; }
    const char* flow_code() const   { return COMPLETE_FLOW; }

    int configure(Vector<String>&, ErrorHandler*);
    int initialize(ErrorHandler*);
    bool can_live_reconfigure() const   { return true; }
    void take_state(Element*, ErrorHandler*);

    void push(int, Packet*);

  private:
    static const int in_port_control = 0;
    static const int in_ports_bulk[STATION_COUNT] = { 1, 3, 5, 7 };
    static const int in_ports_voip[STATION_COUNT] = { 2, 4, 6, 8 };
    static const int out_port = 0;
    static const int out_port_bad = 1;

    JaldiQueue* bulk_queues[STATION_COUNT];
    JaldiQueue* voip_queues[STATION_COUNT];
    uint32_t bulk_requested_bytes[STATION_COUNT];
    uint32_t voip_requested_bytes[STATION_COUNT];
    uint32_t bulk_upstream_bytes[STATION_COUNT];
    uint32_t voip_upstream_bytes[STATION_COUNT];
};

CLICK_ENDDECLS
#endif
