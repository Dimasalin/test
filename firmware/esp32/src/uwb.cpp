// UWB ranging: NOT IMPLEMENTED YET.
//
// The flight brain's beacon tracking is finished and simulated; this file is
// the one piece of hardware glue that still has to be written and tested on
// real radios. It needs a non-blocking double-sided two-way-ranging
// initiator for the Qorvo DW3000 that yields ~10 ranges per second.
// A starting point is the Fhilb/DW3000_Arduino library (its
// dw3000_doublesided_ranging_ping / _pong examples run on ESP32); the "pong"
// example runs unchanged on the beacon, the "ping" example has to be turned
// into a state machine so it does not stall the 50 Hz loop.
//
// The charging pad carries a third radio running the same responder; the
// initiator has to address the two responders in turn.
//
// Until then the build runs as stage 2: uwb_begin() reports "no radio" and
// the fairy orbits its take-off spot instead of following a person.
#include "uwb.h"

bool uwb_begin() { return false; }

bool uwb_poll(float *range_m)
{
    (void)range_m;
    return false;
}

bool uwb_poll_pad(float *range_m)
{
    (void)range_m;
    return false;
}
