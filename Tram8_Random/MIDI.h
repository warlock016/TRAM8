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

#define MIDI_STATUS_bit 7

/* NOTE_ON/OFF|MIDICHANNEL */

#define MIDI_NOTE_ON 0x90
#define MIDI_NOTE_OFF 0x80

/*  clock (decimal 248, hex 0xF8)
    start (decimal 250, hex 0xFA)
    continue (decimal 251, hex 0xFB)
    stop (decimal 252, hex 0xFC)
*/	

#define MIDI_CLK 0xF8
#define MIDI_START 0xFA
#define MIDI_STOP 0xFC
#define MIDI_CONT 0xFB

uint16_t velocity_lookup[128] = {
0x0000, 
0x0200, 
0x0400, 
0x0610, 
0x0810, 
0x0A10, 
0x0C10, 
0x0E20, 
0x1020, 
0x1220, 
0x1420, 
0x1630,
0x1830, 
0x1A30, 
0x1C30, 
0x1E40, 
0x2040, 
0x2240, 
0x2440, 
0x2650, 
0x2850, 
0x2A50, 
0x2C50, 
0x2E60, 
0x3060, 
0x3260, 
0x3460, 
0x3670, 
0x3870, 
0x3A70, 
0x3C70, 
0x3E80, 
0x4080, 
0x4280, 
0x4480, 
0x4690, 
0x4890, 
0x4A90, 
0x4C90, 
0x4EA0, 
0x50A0, 
0x52A0, 
0x54A0, 
0x56A0, 
0x58B0, 
0x5AB0, 
0x5CB0, 
0x5EB0, 
0x60C0, 
0x62C0, 
0x64C0, 
0x66C0, 
0x68D0, 
0x6AD0, 
0x6CD0, 
0x6ED0, 
0x70E0, 
0x72E0, 
0x74E0, 
0x76E0, 
0x78F0, 
0x7AF0, 
0x7CF0, 
0x7EF0, 
0x8100, 
0x8300, 
0x8500, 
0x8700, 
0x8910, 
0x8B10, 
0x8D10, 
0x8F10, 
0x9120, 
0x9320, 
0x9520, 
0x9720, 
0x9930, 
0x9B30, 
0x9D30, 
0x9F30, 
0xA140, 
0xA340, 
0xA540, 
0xA740, 
0xA950, 
0xAB50, 
0xAD50, 
0xAF50, 
0xB150, 
0xB360, 
0xB560, 
0xB760, 
0xB960, 
0xBB70, 
0xBD70, 
0xBF70, 
0xC170, 
0xC380, 
0xC580, 
0xC780, 
0xC980, 
0xCB90, 
0xCD90, 
0xCF90, 
0xD190, 
0xD3A0, 
0xD5A0, 
0xD7A0, 
0xD9A0, 
0xDBB0, 
0xDDB0, 
0xDFB0, 
0xE1B0, 
0xE3C0, 
0xE5C0, 
0xE7C0, 
0xE9C0, 
0xEBD0, 
0xEDD0, 
0xEFD0, 
0xF1D0, 
0xF3E0, 
0xF5E0, 
0xF7E0, 
0xF9E0, 
0xFBF0, 
0xFDF0, 
0xFFF0};


#endif /* MIDI_H_ */