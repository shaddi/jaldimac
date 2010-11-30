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

JaldiGate has 3 pull inputs: input 0 is for control traffic, input 1 is for
bulk traffic, and input 2 is for voip traffic. It has one push output.
Everything arriving on the inputs should be encapsulated in Jaldi frames.

=a

JaldiGate */

class JaldiGate : public Element { public:

    JaldiGate();
    ~JaldiGate();

    const char* class_name() const	{ return "JaldiGate"; }
    const char* port_count() const	{ return "3/1" }
    const char* processing() const	{ return PULL_TO_PUSH; }
    const char* flow_code() const	{ return COMPLETE_FLOW; }

    int configure(Vector<String>&, ErrorHandler*);
    bool can_live_reconfigure() const	{ return true; }
    void take_state(Element*, ErrorHandler*);

    void push(int, Packet*);
    Packet* pull(int);

  private:

};

CLICK_ENDDECLS
#endif
