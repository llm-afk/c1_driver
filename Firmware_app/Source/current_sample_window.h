#ifndef CURRENT_SAMPLE_WINDOW_H
#define CURRENT_SAMPLE_WINDOW_H

#include <stdint.h>

#define CURRENT_SAMPLE_LEGACY_ADVANCE_TICKS 5u

/* 120 MHz TIMER0, 30 MHz ADC, 13.5-cycle sampling.
   A/C sample first; B starts 26 ADC clocks later. Advance the trigger
   0.75 us before the peak and reserve 1.25 us on each side of it.
   This leaves room for the board's 200 ns gate-driver dead time. */
#define CURRENT_SAMPLE_TRIGGER_ADVANCE_TICKS 90u
#define CURRENT_SAMPLE_MIN_LOW_TICKS         150u

/* Move zero-vector time to the all-low-side state only when necessary.
   Applying the SAME offset to all three compares preserves their
   differences (line-voltage commands) and never makes a compare negative. */
static inline uint32_t current_sample_window_shift(uint32_t a, uint32_t b,
                                                  uint32_t c, uint32_t period)
{
    uint32_t minimum = a < b ? a : b;
    uint32_t maximum = a > b ? a : b;
    uint32_t limit = period - CURRENT_SAMPLE_MIN_LOW_TICKS;
    uint32_t shift;

    if (c < minimum) minimum = c;
    if (c > maximum) maximum = c;
    if (maximum <= limit) return 0u;
    shift = maximum - limit;
    return shift < minimum ? shift : minimum;
}

#endif
