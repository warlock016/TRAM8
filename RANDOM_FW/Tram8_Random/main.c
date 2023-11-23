/*
 * TRAM8 CLOCKS & RANDOM
 *
 *
 * Created: 22.08.2017 16:49:13
 * Author : Kay Knofe LPZW.modules
 * www.leipzigwest.org
`*
 * Version 1.3
 * WORKS WITH HW REV 1.0-1.5
 *
 *	SOFTWARE License:
 *	CC BY-NC-SA 4.0
 */

//#define SIMULATOR

#define BUTTON_LED_DEBUG

#include <avr/interrupt.h>
#include <avr/io.h>
#include <string.h>
#include <stdlib.h>
#include <avr/eeprom.h>
#include <stdbool.h>

#define F_CPU 16000000UL
#define FOSC 16000000UL
#define BAUD 31250UL
#define MYUBRR (FOSC/(16*BAUD)-1)

#include <util/delay.h>

#include "luts.h"
#include "midi.h"

#include "general_twi.h"
#include "MAX5825.h"

//PROTOTYPES
void set_default(void);
void set_LED(uint8_t var);

void set_pin_inv(uint8_t pinnr);
void clear_pin_inv(uint8_t pinnr);
void start_timer1(uint8_t cmp_value);



#define ENABLE 1
#define DISABLE 0

#define TRUE 1
#define FALSE 0

#define EEPROM_CHANNEL_ADDR 0x100
#define EEPROM_MAP_ADDR 0x101
#define	EEPROM_PRESET_ADDR 0x110

#define LED_pin PC0
#define BUTTON_PIN PC1

#define BUTTON_UP		1
#define BUTTON_DOWN		2
#define BUTTON_PRESSED	3
#define BUTTON_RELEASED	4

#define PIN_A 0
#define PIN_B 1
#define PIN_C 2
#define PIN_D 3
#define PIN_E 4
#define PIN_F 5
#define PIN_G 6
#define PIN_H 7

#define maxNotes 10
#define maxSlots 8
#define maxChans 16
/* MIDI GLOBALS */
typedef struct midi_channel_data
{
	uint8_t note_count;
	uint8_t dac_offset;
	uint8_t max_num_notes;
	int8_t midi_note_buf[maxNotes];
	int8_t midi_note_min;
	int8_t midi_note_max;
	int8_t midi_note_offset;
} channel_data;

channel_data *active_chan[maxChans] = {NULL};

uint8_t pulse_reset_flag[maxSlots] = {false}; // track dac on/off when in pulse mode

uint8_t hw_slots = 2;

uint8_t midi_clock_tick_cntr = 0; // 24 ppqn
uint8_t midi_clock_cntr = 0;	  // 4 ppqn
uint8_t midi_clock_run = 0;		  // 1 = run, 0 = stop
uint8_t clk_stop_pulse = 0;		  // EoC reset pulse flag 

void (*set_pin_ptr)(uint8_t) = &set_pin_inv;
void (*clear_pin_ptr)(uint8_t) = &clear_pin_inv;
void register_funcs(void);

void tram8_cfg(void)
{
	/* GPIO INIT */
	DDRC = 0x0C | (1 << LED_pin) | (1 << BUTTON_PIN); // LDAC & CLEAR & LED
	DDRB |= (1 << PB0);								  // Trigger Out 0
	DDRD |= 0xFE;									  // Trigger outs 1-7

	PORTD &= 0x01; // ALL GATES LOW (Inverter Out)
	PORTB &= 0xFE; //

#ifndef SIMULATOR
	set_LED(ENABLE);
	_delay_ms(300);
	set_LED(DISABLE);
#endif

#define BUTTONFIXVARIABLE (uint8_t *)0x07
	// THIS PART IS FOR HARDWARE VERSION 1.5 and up so Code is backwards compatible
	uint8_t buttonfix_flag = 0;

	do
	{
	} while (!eeprom_is_ready());
	// load Channel
	buttonfix_flag = eeprom_read_byte(BUTTONFIXVARIABLE);

	if (buttonfix_flag == 0xAA)
	{
		DDRC &= ~((1 << BUTTON_PIN)); // INPUT
	}

	/* Button Poll Timer */
	TCCR2 = (1 << WGM20) | (0 << WGM21) | (0b111 << CS20); // CTC, PORT disconnected, /1024
	OCR2 = 157;											   // ca. 10ms@16MHz

	/* MIDI INIT */
	UCSRB = (1 << RXCIE) | (1 << RXEN);
	UCSRC = (1 << UCSZ0) | (1 << UCSZ1);
	UBRRH = (unsigned char)(MYUBRR >> 8);
	UBRRL = (unsigned char)MYUBRR;

	/* LOAD MIDI MAP FROM EEPROM*/
	do
	{
	} while (!eeprom_is_ready());
	// load Channel
	// midi_channel = eeprom_read_byte(EEPROM_CHANNEL_ADDR);
	do
	{
	} while (!eeprom_is_ready());
	// load map
	// eeprom_read_block(&midi_note_map, EEPROM_MAP_ADDR, 8);
	do
	{
	} while (!eeprom_is_ready());
	// set map to default if never learned:
	// if (midi_note_map[0] == 0xFF)
		// set_default();

	/* CHECK FOR EXPANDERS */
	PORTC = (1 << PC2) | (1 << PC3);
	DDRC |= (1 << PC2) | (1 << PC3);

#ifndef SIMULATOR
	init_max5825();
#endif

	_delay_ms(100);

	if ((PINB >> PB1) & 1)
	{
		// OLD HARDWARE
		// SET PINS INVERSE
		set_pin_ptr = &set_pin_inv;
		clear_pin_ptr = &clear_pin_inv;
	}
	else
	{
		// NEW HARDWARE - PIN TO GND
		// SET PINS NORMAL
		set_pin_ptr = &clear_pin_inv;
		clear_pin_ptr = &set_pin_inv;
	}
}
void tram8_init(void)
{
	(*clear_pin_ptr)(PIN_A);
	(*clear_pin_ptr)(PIN_B);
	(*clear_pin_ptr)(PIN_C);
	(*clear_pin_ptr)(PIN_D);
	(*clear_pin_ptr)(PIN_E);
	(*clear_pin_ptr)(PIN_F);
	(*clear_pin_ptr)(PIN_H);
	(*clear_pin_ptr)(PIN_G);

	_delay_ms(500);

	// create buffer for I2C
	max_fill_struct all_dacs = {NULL};
	all_dacs.dac1_cmd = MAX5825_REG_CODEn | 1;
	all_dacs.dac2_cmd = MAX5825_REG_CODEn | 2;
	all_dacs.dac3_cmd = MAX5825_REG_CODEn | 3;
	all_dacs.dac4_cmd = MAX5825_REG_CODEn | 4;
	all_dacs.dac5_cmd = MAX5825_REG_CODEn | 5;
	all_dacs.dac6_cmd = MAX5825_REG_CODEn | 6;
	all_dacs.dac7_cmd = MAX5825_REG_CODEn_LOADall | 7;
}

int main(void)
{
	tram8_cfg();
	tram8_init();

	midi_init();

	register_funcs();
	/* INTERRUPTS ENABLE */
	sei();

	/* MAIN LOOP */
	/* Detect user button interaction */
	while (1)
	{

		static uint8_t learn_button = BUTTON_UP;
		static uint8_t button_now = 0;
		static uint8_t button_bounce = 0;
		static uint8_t button_last = 0;

		// CHECK FOR GOTO SETTINGS MENU - LEARN BUTTON HAS TO BE DOWN FOR 3 SEC
		if ((learn_button == BUTTON_RELEASED))
		{
			// setting_wait_counter = 0;
			// setting_wait_flag = 0;
			learn_button == BUTTON_UP;
			set_LED(DISABLE);
		}

		if (learn_button == BUTTON_PRESSED)
		{
			learn_button = BUTTON_DOWN;
			// set_LED(ENABLE);
			// if(setting_wait_flag == 0){	//START THE WAIT
			// setting_wait_counter = 0;
			// setting_wait_flag = 1;
			//
			////LEDs.groupLED = LED_BLINK2;
			//
			//}
		}

		if (learn_button == BUTTON_DOWN)
		{
			// if(setting_wait_flag == 1){
			// if (setting_wait_counter >=200){
			set_LED(ENABLE);
			////	midi_learn();
			// setting_wait_counter = 0;
			// setting_wait_flag = 0;
			//}
			//}
		}

		/************************************************************************/
		/*                    keyscan                                           */
		/************************************************************************/
		if ((TIFR >> OCF2) & 1) // Timer Interrupt Flag Register
		{
			// keyscan(&buttons);
			TCNT2 = 0;			 // reset timer
			TIFR |= (1 << OCF2); // reset flag

			button_now = PINC & (1 << BUTTON_PIN);

			if (button_now != button_bounce)
			{
				button_bounce = button_now;
			}
			else
			{

				if (button_now != button_last)
				{

					if (button_now == 0)
					{
						learn_button = BUTTON_RELEASED;
						// set_LED(DISABLE);
					}
					else
					{
						learn_button = BUTTON_PRESSED;
						//	set_LED(ENABLE);
					}

					button_last = button_now;
				}
			}

		}
	}
}

ISR(TIMER1_COMPA_vect)
{
	/************************************************************************/
	/*                    CLK PULSE                                         */
	/************************************************************************/

	TCCR1B = 0;			  // stop timer
	TIFR |= (1 << OCF1A); // reset flag
	
	
	for (char i = 0; i < maxSlots; ++i)
	{
		if (pulse_reset_flag[i]) // array with flags for all digital pins -> upper row of jacks on tram8
		{
			(*clear_pin_ptr)(i);
			pulse_reset_flag[i] = 0;
		}	
	}
	
	if (clk_stop_pulse)
	{
		max5825_set_load_channel(1, 0x0);
		clk_stop_pulse = 0;
	}
}

/* MIDI RECEIVER */
ISR(USART_RXC_vect)
{
	uint8_t byte = UDR;
	midi_receive_byte(byte);
}

void set_default(void)
{
	// MIDI
	uint8_t midi_channel = 1;
	// memcpy(&midi_note_map, &midi_note_map_default, 8);

	// SAVE TO EEPROM
	do
	{
	} while (!eeprom_is_ready());
	// save Channel
	// midi_channel = eeprom_read_byte(EEPROM_CHANNEL_ADDR);
	eeprom_write_byte(EEPROM_CHANNEL_ADDR, midi_channel);

	do
	{
	} while (!eeprom_is_ready());
	// load map
	//	eeprom_read_block(&midi_note_map,EEPROM_MAP_ADDR,8);
	// eeprom_write_block(&midi_note_map, EEPROM_MAP_ADDR, 8);

	// do {} while (!eeprom_is_ready());
	////load map
	////	eeprom_read_block(&midi_note_map,EEPROM_MAP_ADDR,8);
	// eeprom_write_block(&presets,EEPROM_PRESET_ADDR,sizeof(sys_presets));

	return;
}

void set_LED(uint8_t var)
{

	if (var == ENABLE)
		DDRC |= (1 << LED_pin);
	else
		DDRC &= 0xFF ^ (1 << LED_pin);

	return;
}

void start_timer1(uint8_t cmp_value)
{
	TCNT1 = 0;
	//	ASSR = 0;//(1 << AS0); //sync op - clock <= system clock
	TCCR1A = 0;
	TCCR1B = (0b101 << CS00); /// 1024
	OCR1A = cmp_value;		  // 157;//ca. 10ms@16MHz
	TIFR |= (1 << OCF1A);
	TIMSK |= (1 << OCIE1A);
	return;
}

void set_pin_inv(uint8_t pinnr)
{
	// inverted cos of 74HC1G14 inverter
	switch (pinnr)
	{
	case 0:
		PORTB &= 0xFE;
		break;
	case 1:
		PORTD &= 0xFD;
		break;
	case 2:
		PORTD &= 0xFB;
		break;
	case 3:
		PORTD &= 0xF7;
		break;
	case 4:
		PORTD &= 0xEF;
		break;
	case 5:
		PORTD &= 0xDF;
		break;
	case 6:
		PORTD &= 0xBF;
		break;
	case 7:
		PORTD &= 0x7F;
		break;
	default:
		break;
	}

	return;
}

void clear_pin_inv(uint8_t pinnr)
{
	switch (pinnr)
	{
	case 0:
		PORTB |= 0x01;
		break;
	case 1:
		PORTD |= 0x02;
		break;
	case 2:
		PORTD |= 0x04;
		break;
	case 3:
		PORTD |= 0x08;
		break;
	case 4:
		PORTD |= 0x10;
		break;
	case 5:
		PORTD |= 0x20;
		break;
	case 6:
		PORTD |= 0x40;
		break;
	case 7:
		PORTD |= 0x80;
		break;
	default:
		break;
	}

	return;
}

midi_event_callback_t clock(char chan, char data1, char data2)
{
	if(!midi_clock_run) return;

	bool timer_flag = false;

	if (midi_clock_tick_cntr % 6 == 0)
	{
		(*set_pin_ptr)(PIN_A);
		pulse_reset_flag[PIN_A] = true;
		timer_flag = true;
	}

	if(midi_clock_cntr % 24 == 0)
	{
		(*set_pin_ptr)(PIN_B);
		pulse_reset_flag[PIN_B] = true;
		timer_flag = true;
	}

	midi_clock_tick_cntr++;
	if (midi_clock_tick_cntr > 23) midi_clock_tick_cntr = 0;
	if(timer_flag) start_timer1(36); // enable clock timer (ca. 2-3 ms) to trigger pin reset
	return;
}

midi_event_callback_t run(char chan, char data1, char data2)
{
	if(!midi_clock_run)
	{
		max5825_set_load_channel(0, 0xFFFF);	
		midi_clock_tick_cntr = 0;
		midi_clock_run = 1;
		start_timer1(36);
	}
	return;
}

midi_event_callback_t cont(char chan, char data1, char data2)
{
	if(!midi_clock_run)
	{
		max5825_set_load_channel(0, 0xFFFF);
		midi_clock_run = 1;
	}
	return;
}

midi_event_callback_t stop(char chan, char data1, char data2)
{
	if(midi_clock_run)
	{
		max5825_set_load_channel(0, 0x0);
		midi_clock_run = 0;

		max5825_set_load_channel(1, 0xFFFF);
		clk_stop_pulse = 1;
		start_timer1(36);
	}
	return;
}

midi_event_callback_t note_off(char chan, char data1, char data2)
{
	
	if(active_chan[chan] == NULL) return; // wrong channel
	if(active_chan[chan]->note_count <= 0) return; // no notes

	active_chan[chan]->note_count--;
	
	uint8_t pos = active_chan[chan]->note_count % active_chan[chan]->max_num_notes + active_chan[chan]->dac_offset;
	(*clear_pin_ptr)(pos);
	return;
}

midi_event_callback_t note_on(char chan, char data1, char data2)
{
	if(active_chan[chan] == NULL) return; // channel not registered

	if(data2 == 0) // midi note off message with velocity "0"
	{
		note_off(chan, data1, data2);
		return;
	}
	
	uint8_t dac_pos = (active_chan[chan]->note_count % active_chan[chan]->max_num_notes) + active_chan[chan]->dac_offset;
	int8_t voct_pos = data1 - active_chan[chan]->midi_note_offset;

	if(voct_pos < active_chan[chan]->midi_note_min)
	{
		voct_pos = active_chan[chan]->midi_note_min;
	}
	else if (voct_pos > active_chan[chan]->midi_note_max)
	{
		voct_pos = active_chan[chan]->midi_note_max;
	}

	uint16_t dac_val = voct_lookup[voct_pos] << 4; // retrieve dac value for respective note

	max5825_set_load_channel(dac_pos, dac_val); // set dac first
	(*set_pin_ptr)(dac_pos); // set gate next

	active_chan[chan]->note_count++;
	return;
}

void register_funcs(void)
{

	register_midi_channel(0x0,2,0,60,0);
	register_midi_channel(0x1,2,0,60,0);
	register_midi_channel(0x2,2,0,60,0);

	midi_register_event_handler(EVT_SYS_REALTIME_TIMING_CLOCK, clock);

	midi_register_event_handler(EVT_SYS_REALTIME_SEQ_START, run);
	midi_register_event_handler(EVT_SYS_REALTIME_SEQ_STOP, stop);
	midi_register_event_handler(EVT_SYS_REALTIME_SEQ_CONTINUE, cont);

	midi_register_event_handler(EVT_CHAN_NOTE_ON, note_on);
	midi_register_event_handler(EVT_CHAN_NOTE_OFF, note_off);
}

/* chan = midi channel 0 - 15, note_mode 1,2,3,4 a.k.a. note polyphony */
void register_midi_channel(uint8_t chan, uint8_t note_mode, int8_t note_min, int8_t note_max, int8_t note_offset)
{
	if (note_mode < 1) return;
	if (hw_slots + note_mode > maxSlots) return;
	if(chan > 15) return;
	if(chan < 0) return;
	if(active_chan[chan] != NULL) return;

	active_chan[chan] = (channel_data*)calloc(1,sizeof(channel_data));
	
	if (active_chan[chan] == NULL)
		{
			// Handle memory allocation failure
			return;
		}

	active_chan[chan]->dac_offset = hw_slots;
	hw_slots += note_mode;

	active_chan[chan]->max_num_notes = note_mode; 
	active_chan[chan]->midi_note_offset = note_offset;
	active_chan[chan]->midi_note_min = note_min;
	active_chan[chan]->midi_note_max = note_max;	
}

void deregister_midi_channel(uint8_t chan)
{
	if(chan > 15) return;
	if(chan < 0) return;
	if (active_chan[chan] == NULL) return;
	
	hw_slots -= active_chan[chan]->max_num_notes;

	free(active_chan[chan]);
	active_chan[chan] = NULL;
}

