/*
 * MIDI.h
 * Created: 22.08.2017 16:49:13
 * Author : Kay Knofe LPZW.modules
 * www.leipzigwest.org
 *
 *	SOFTWARE License:
 *	CC BY-NC-SA 4.0
 */

#ifndef MIDI_H_
#define MIDI_H_

#include <stdint.h>

#define voct_range 60

#define vel_range 128


uint16_t voct_lookup[voct_range + 1] = {
0,              // C0
68,             // C#0
137,            // D0
205,            // D#0
272,            // E0
340,            // F0
409,            // F#0
477,            // G0
545,            // G#0
613,            // A0
681,            // A#0
749,            // B0

817,            // C1 corrected value -> 817
885,            // C#1
954,            // D1
1022,           // D#1
1089,           // E1
1157,           // F1
1226,           // F#1
1294,           // G1
1362,           // G#1
1430,           // A1
1498,           // A#1
1566,           // B1

1635,           // C2
1703,           // C#2
1771,           // D2
1839,           // D#2
1907,           // E2
1975,           // F2
2043,           // F#2
2112,           // G2
2180,           // G#2
2248,           // A2
2316,           // A#2
2384,           // B2

2452,           // C3
2520,           // C#3
2588,           // D3
2656,           // D#3
2725,           // E3
2792,           // F3
2861,           // F#3
2929,           // G3
2997,           // G#3
3065,           // A3
3133,           // A#3
3201,           // B3

3269,           // C4
3338,           // C#4
3406,           // D4
3473,           // D#4
3542,           // E4
3610,           // F4
3678,           // F#4
3746,           // G4
3814,           // G#4
3882,           // A4
3951,           // A#4
4019,           // B4

4086            // C5
};



#endif /* MIDI_H_ */