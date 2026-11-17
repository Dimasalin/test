// Range to the beacon the companion wears (stage 3).
#pragma once

// Returns true if a DW3000 answered on SPI.
bool uwb_begin();
// Returns true and writes metres when a new two-way-ranging result is ready.
// Call every control tick; it must never block.
bool uwb_poll(float *range_m);
// Same, for the beacon on the charging pad (ranged alternately with the
// companion's, about 10 times a second each).
bool uwb_poll_pad(float *range_m);
