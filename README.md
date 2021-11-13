# TRAM8 Module Firmware Hacking

LPZW.modules TRAM8 Repository for Firmware Hacking

# Stock Firmware 
8 Gates of the same MIDI Channel + Velocity of those notes or 8 fixed CCs of same channel

# Clocks & Random:

Gate Outputs:
1: Run signal - goes high with start low with stop
2: clock 24ppqn
3: clock 4ppqn aka 16th triggers.
4: 8th note trigger
5: clock 1ppqn aka quarter note triggers
6: half note shifted by a quarter (simple snare position)
7: beginning of bar pulse
8: 16th divided by 3 clock
.
Velocity/CV Outputs:
1: random triggered each 16th
2: random triggered each 16th
3: brownian random triggered each 16th (drunken walk)
4: random triggered each quarter
5: random triggered each quarter
6: brownian random triggered each quarter (drunken walk)
7: random triggered each bar
8: 16th stepped ramp to go from 0-5V in a bar

# CC BY-NC-ND 4.0 KAY KNOFE OF LPZW.modules


