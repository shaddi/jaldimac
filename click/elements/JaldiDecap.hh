#ifndef CLICK_JALDIDECAP_HH
#define CLICK_JALDIDECAP_HH
#include <click/element.hh>
CLICK_DECLS

/*
=c

JaldiDecap

=s jaldi

decapsulates packets which are wrapped in a Jaldi header

=d

Decapsulates Jaldi frames into IP packets. Jaldi control messages, for which
decapsulation is not meaningful, are placed on output 0. IP packets are placed
on output 1. Jaldi frames which could not be decapsulated for whatever reason
(for example, they have an invalid type, or they failed the CRC check) are
placed on output 2, if that output is connected.

This element is push only.

=a

JaldiEncap */

class JaldiDecap : public Element { public:

    JaldiDecap();
    ~JaldiDecap();

    const char* class_name() const	{ return "JaldiDecap"; }
    const char* port_count() const	{ return "1/2-3" }
    const char* processing() const	{ return PUSH; }
    const char* flow_code() const	{ return COMPLETE_FLOW; }

    bool can_live_reconfigure() const	{ return true; }

    void push(int, Packet*);
};

CLICK_ENDDECLS
#endif
