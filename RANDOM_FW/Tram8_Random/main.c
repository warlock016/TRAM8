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

#define F_CPU 16000000UL
#define FOSC 16000000UL
#define BAUD 31250UL
#define MYUBRR (FOSC/(16*BAUD)-1)

#include <util/delay.h>
#include "MIDI.h"
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

/* MIDI GLOBALS */

uint8_t midi_clock_tick_cntr=0; // 24 ppqn
uint8_t midi_clock_cntr=0; // 4 ppqn
uint8_t midi_clock_run=0; // 1 = run, 0 = stop

uint8_t midi_tripl_cntr=0;
uint8_t midi_quarter_cntr=0;
uint8_t midi_quarter_cntr_prev = 0;

uint8_t sync_counter_match = (1 << 2); //1=QRT 2=HLF 4=BAR 8=2BAR etc.

uint8_t wait_for_match = 0;

uint8_t boundary_led_flag = 0;

uint8_t group_hold = 0;

uint8_t midi_note_map[8] = {60,61,62,63,64,65,66,67};
uint8_t midi_note_map_default[8] = {60,61,62,63,64,65,66,67};
	
int rand_values[8] = {0,0,0,0,0,0,0,0};	

uint8_t midi_buff[3] = {0,0,0};
uint8_t midi_buff_point = 0;
uint8_t midi_buff_allowed = 0;
uint8_t midi_channel = 9;
uint8_t midi_learn_mode = 0;
uint8_t midi_learn_current = 0;

uint8_t velocity_out = 0;

uint16_t setting_wait_counter = 0;
uint8_t setting_wait_flag = 0;

uint8_t start_clock_delay_flag = 0;

void  (*set_pin_ptr)(uint8_t ) = & set_pin_inv;
void  (*clear_pin_ptr)(uint8_t ) = & clear_pin_inv;


int main(void)
{

/* GPIO INIT */
	//DDRA = 0xFF; //ROW&COLUMN FOR LED/BUTTON	
	DDRC = 0x0C | (1 << LED_pin) | (1 << BUTTON_PIN); //LDAC & CLEAR & LED
	DDRB = 0x01; // Trigger Out 0	
	DDRD = 0xFE; //Trigger outs 1-7

    PORTD |= 0xFE; //ALL GATES LOW (Inverter Out)
    PORTB |= 0x01; // 
	
#ifndef SIMULATOR		
	set_LED(ENABLE);
	_delay_ms(300);
	set_LED(DISABLE);
#endif	

	#define BUTTONFIXVARIABLE (uint8_t *) 0x07
	//THIS PART IS FOR HARDWARE VERSION 1.5 and up so Code is backwards compatible
	uint8_t buttonfix_flag = 0;

	do {} while (!eeprom_is_ready());
	//load Channel
	buttonfix_flag = eeprom_read_byte(BUTTONFIXVARIABLE);

	if (buttonfix_flag == 0xAA)
	{
		DDRC &= ~((1 << BUTTON_PIN));//INPUT
	}


	//DIRECT_BUTTONS_DDR &= ~(1 << DIRECT_GROUPBUTTON_bit); //GROUP BUTTON IS INPUT
	//DIRECT_BUTTONS_PORT |= (1 << DIRECT_GROUPBUTTON_bit); //PULLUP SWITCH TO GND
	
/* GUI INIT */ //GLOABL NOW
	/* LED INIT */
	//LEDs = { {LED_ON,LED_ON,LED_ON,LED_ON,LED_ON,LED_ON,LED_ON,LED_ON},LED_OFF,LED_OFF,LED_OFF,LED_OFF };
	//row_select = 0;				
	/* BUTTONS INIT */
	//buttons = { {BUTTON_UP,BUTTON_UP,BUTTON_UP,BUTTON_UP,BUTTON_UP,BUTTON_UP,BUTTON_UP,BUTTON_UP},BUTTON_UP,BUTTON_UP };	
				
	/* Blink Timer Init */
//	TCCR1B = 0b010; //FREE RUNNING, PORT disconnected, /64
	/* Button Poll Timer */
	TCCR2 = (1 << WGM20)|(0 << WGM21)|(0b111<<CS20);//CTC, PORT disconnected, /1024
	OCR2 = 157;//ca. 10ms@16MHz
	
	/* MIDI INIT */
	UCSRB = (1<<RXCIE)|(1<<RXEN);
	UCSRC = (1<<UCSZ0)|(1<<UCSZ1);
	UBRRH = (unsigned char)(MYUBRR>>8);
	UBRRL = (unsigned char) MYUBRR;
	
/* LOAD MIDI MAP FROM EEPROM*/
	do {} while (!eeprom_is_ready());
	//load Channel 
	midi_channel = eeprom_read_byte(EEPROM_CHANNEL_ADDR);
	do {} while (!eeprom_is_ready());
	//load map
	eeprom_read_block(&midi_note_map,EEPROM_MAP_ADDR,8);	
	do {} while (!eeprom_is_ready());
	//set map to default if never learned:
	if (midi_note_map[0]==0xFF)
		set_default();

	
/* CHECK FOR EXPANDERS */
	//uint8_t i2cread[5] = {NULL};
	//TWI_READ_BULK(0x20,0x00,2,&i2cread);
	PORTC = (1 << PC2)|(1 << PC3);	
	DDRC  |= (1 << PC2)|(1 << PC3);

#ifndef SIMULATOR	
	velocity_out = test_max5825();	//Velocity Out Expander present? (a.k.a. WK4) 
//	if(velocity_out){
		init_max5825();
		init_max5825();		
//		max5825_set_load_channel(0,0xFfff);
//	}
#endif
	
	//max5825_set_load_channel(4,0x1F40); //1V@Ref4.1	
	//max5825_set_load_channel(5,0x3E80);	//2V@Ref4.1
	//max5825_set_load_channel(6,0x9C40);	//5V@Ref4.1
	//max5825_set_load_channel(7,0xDAC0);	//7V@Ref4.1
	PORTB |= (1 << PB1); //PULLUP
	DDRB &= ~(1 << PB1); //INPUT		
	_delay_ms(100);
	
	if ((PINB >> PB1) &1){
		//OLD HARDWARE
		//SET PINS INVERSE
		  set_pin_ptr = & set_pin_inv;
		 clear_pin_ptr = & clear_pin_inv;
			
	} else {
		//NEW HARDWARE - PIN TO GND
		//SET PINS NORMAL
		  set_pin_ptr = & clear_pin_inv;
		  clear_pin_ptr = & set_pin_inv;		
	}

				(*clear_pin_ptr)(PIN_B);
				(*clear_pin_ptr)(PIN_C);
				(*clear_pin_ptr)(PIN_D);
				(*clear_pin_ptr)(PIN_E);
				(*clear_pin_ptr)(PIN_F);
				(*clear_pin_ptr)(PIN_H);
				(*clear_pin_ptr)(PIN_G);
				
	_delay_ms(500);			
	
	//create buffer for I2C 
	max_fill_struct all_dacs = {NULL};	
	all_dacs.dac1_cmd = MAX5825_REG_CODEn | 1;
	all_dacs.dac2_cmd = MAX5825_REG_CODEn | 2;	
	all_dacs.dac3_cmd = MAX5825_REG_CODEn | 3;
	all_dacs.dac4_cmd = MAX5825_REG_CODEn | 4;
	all_dacs.dac5_cmd = MAX5825_REG_CODEn | 5;
	all_dacs.dac6_cmd = MAX5825_REG_CODEn | 6;	
	all_dacs.dac7_cmd = MAX5825_REG_CODEn_LOADall | 7;		
	
	
	midi_learn_mode = 0;
/* INTERRUPTS ENABLE */
	sei();
	
/* MAIN LOOP */	
    while (1) {
	
	static uint8_t learn_button = BUTTON_UP;
	static uint8_t button_now = 0;
	static uint8_t button_bounce = 0;
	static uint8_t button_last = 0;
	static uint8_t nxt_rand=0;
	static int int16_temp;
	static int rand_2_temp;
	static int rand_5_temp;
	static uint8_t calc_update_flag = 1;
	static uint8_t dac_update_flag = 1;
		
	if(midi_clock_tick_cntr%6==5){
		calc_update_flag = 1; 
		dac_update_flag = 1;//so the DAC is only updated one every 16ths
	}
	
	if (midi_clock_tick_cntr%6==0)
	{
		if(calc_update_flag){

			//channel 3 brownian noise
			int16_temp = rand()>>2; //half random
	
			if(int16_temp>0x1000)//{
				int16_temp |= 0xE000; //MAKE IT MINUS!!  		
		
			rand_2_temp += int16_temp;	

			if(rand_2_temp>16383)
				rand_2_temp=16383;
			if(rand_2_temp<0)
				rand_2_temp=0;
		
			int16_temp = (uint16_t) rand_2_temp<<2;
				
			all_dacs.dac2_val =	(int16_temp & 0xFF00)>>8; //4095 * (midi_clock_cntr & 0x0F);
			all_dacs.dac2_val |= (int16_temp & 0x00F0)<<8;		
		
			//channel 8 ramp up
			int16_temp = 4095 * (midi_clock_cntr & 0x0F);
		
			all_dacs.dac7_val =	(int16_temp & 0xFF00)>>8; //4095 * (midi_clock_cntr & 0x0F);
			all_dacs.dac7_val |= (int16_temp & 0x00F0)<<8;			
		
			if (midi_clock_cntr%4==0)
			{
			
				all_dacs.dac3_val = rand_values[3];
				all_dacs.dac4_val = rand_values[4];
			
				int16_temp = rand()>>2; //half random
			
				if(int16_temp>0x1000)//{
					int16_temp |= 0xE000; //MAKE IT MINUS!!
				
					rand_5_temp += int16_temp;

					if(rand_5_temp>16383)
					rand_5_temp=16383;
					if(rand_5_temp<0)
					rand_5_temp=0;

					int16_temp = (uint16_t) rand_5_temp<<2;
				
					all_dacs.dac5_val =	(int16_temp & 0xFF00)>>8; //4095 * (midi_clock_cntr & 0x0F);
					all_dacs.dac5_val |= (int16_temp & 0x00F0)<<8;

			}
			
			if (midi_clock_cntr%16 == 0)
				all_dacs.dac6_val = rand_values[6];
			
			if (learn_button!=BUTTON_DOWN)
				max5825_set_load_all(&all_dacs);				

					
			

			
			calc_update_flag=0;
			
		}
	}else{
		//random scheduler 		
		nxt_rand++;
		if(nxt_rand>=10)
			nxt_rand=0;
		
		switch(nxt_rand){
			case 0: 
			case 7:
			case 9:		all_dacs.dac0_val = rand(); break;//directly cos updates only on 16ths anyway
			
			case 1:
			case 8:
			case 10:	all_dacs.dac1_val = rand(); break; //directly cos updates only on 16ths anyway
			
			case 3:		rand_values[3] = rand();	break;
			
			case 4: 	rand_values[4] = rand();	break;
			
			case 6:		rand_values[6] = rand();	break; 		
		
			default: break;
		}		
	}
	
	
	//CHECK FOR GOTO SETTINGS MENU - LEARN BUTTON HAS TO BE DOWN FOR 3 SEC 
	if ((learn_button == BUTTON_RELEASED)){
		//setting_wait_counter = 0;
		//setting_wait_flag = 0;
		learn_button == BUTTON_UP;
		set_LED(DISABLE);
	}

	if (learn_button == BUTTON_PRESSED){
		learn_button = BUTTON_DOWN;
		//set_LED(ENABLE);		
		//if(setting_wait_flag == 0){	//START THE WAIT
			//setting_wait_counter = 0;
			//setting_wait_flag = 1;
	//
			////LEDs.groupLED = LED_BLINK2;
//
		//}
	}
		
	if(learn_button == BUTTON_DOWN){
		//if(setting_wait_flag == 1){
			//if (setting_wait_counter >=200){
				set_LED(ENABLE);
			////	midi_learn();
				//setting_wait_counter = 0;
				//setting_wait_flag = 0;
			//}
		//}
	}
		
	
	/************************************************************************/
	/*                    keyscan                                           */
	/************************************************************************/
		if ((TIFR>>OCF2)&1) // Timer Interrupt Flag Register
		{
			//keyscan(&buttons);
			TCNT2 = 0; //reset timer
			TIFR |= (1 << OCF2); //reset flag		
			
			button_now = PINC & (1 << BUTTON_PIN);
			
			if (button_now != button_bounce){
				button_bounce = button_now;			
			}else{			
										
				if (button_now != button_last){			
					
					if(button_now == 0){
						learn_button = BUTTON_RELEASED;
						//set_LED(DISABLE);
					}else{
						learn_button = BUTTON_PRESSED;		
					//	set_LED(ENABLE);
					}
						
					button_last = button_now;		
				}			
			}

			//if(setting_wait_flag == 1)
				//setting_wait_counter++; 
		}
		
	 }
}




void set_default(void){
	//MIDI
	midi_channel = 9;
	memcpy(&midi_note_map,&midi_note_map_default,8);
	
	//SAVE TO EEPROM
	do {} while (!eeprom_is_ready());
	//save Channel
	//midi_channel = eeprom_read_byte(EEPROM_CHANNEL_ADDR);
	eeprom_write_byte(EEPROM_CHANNEL_ADDR,midi_channel);
	
	do {} while (!eeprom_is_ready());
	//load map
	//	eeprom_read_block(&midi_note_map,EEPROM_MAP_ADDR,8);
	eeprom_write_block(&midi_note_map,EEPROM_MAP_ADDR,8);
	
	//do {} while (!eeprom_is_ready());
	////load map
	////	eeprom_read_block(&midi_note_map,EEPROM_MAP_ADDR,8);
	//eeprom_write_block(&presets,EEPROM_PRESET_ADDR,sizeof(sys_presets));
	
	return;
}

ISR(TIMER1_COMPA_vect){
		/************************************************************************/
		/*                    CLK PULSE                                         */
		/************************************************************************/
		
			TCCR1B = 0; //stop timer
			TIFR |= (1 << OCF1A); //reset flag
			
			(*clear_pin_ptr)(PIN_B);
			(*clear_pin_ptr)(PIN_C);
			(*clear_pin_ptr)(PIN_D);
			(*clear_pin_ptr)(PIN_E);
			(*clear_pin_ptr)(PIN_F);
			(*clear_pin_ptr)(PIN_H);
			(*clear_pin_ptr)(PIN_G);
			
}


/* MIDI RECEIVER */
ISR(USART_RXC_vect)
{
	uint8_t uart_data;
	// static int brown1;
	//int temp;
	uart_data = UDR;
	
	
	//set_LED(ENABLE);
	
	if ((uart_data>>MIDI_STATUS_bit)&1)
	{
		if (uart_data == MIDI_START){
			midi_clock_run = 1;
			midi_clock_tick_cntr = 0;
			midi_clock_cntr = 0;
			midi_tripl_cntr = 0;
			midi_quarter_cntr = 0;
			(* set_pin_ptr)(PIN_A);
		}
		
		if (uart_data == MIDI_STOP){
			midi_clock_run = 0;
			(* clear_pin_ptr)(PIN_A);
			}
		
		if (uart_data == MIDI_CONT){
			midi_clock_run = 1;
			(* set_pin_ptr)(PIN_A);		
			}
		
		//if (((uart_data&0xE0) == MIDI_NOTE_OFF)){ //receives note ons too: &E0 !!
			//midi_buff[0] = uart_data;
			//midi_buff_point = 1;
			//midi_buff_allowed = 2;
			////set_LED(ENABLE);
		//}
		
		// EVERYTHING CLOCK
			if ((midi_clock_run == 1) && (uart_data == MIDI_CLK)) // midi clock rx
			{
			(*set_pin_ptr)(PIN_B);

			if (midi_clock_tick_cntr % 6 == 0)
			{
				(*set_pin_ptr)(PIN_C);
				(*set_pin_ptr)(PIN_D);
				(*set_pin_ptr)(PIN_E);
			}

			if(midi_clock_tick_cntr % 12 == 0) 
				(*set_pin_ptr)(PIN_F);

			midi_clock_tick_cntr++;
			if(midi_clock_tick_cntr > 23) midi_clock_tick_cntr = 0; //reset
			start_timer1(78); // enable clock timer to trigger pin reset
			}

		
		
		
			
		//if ( ((midi_quarter_cntr^midi_quarter_cntr_prev)&sync_counter_match) != 0){ //test for boundary
			//(* set_pin_ptr)(PIN_G);
			//}
			//
		//midi_quarter_cntr_prev = midi_quarter_cntr;	
		
	
		
		
	
		//else if (midi_buff_allowed > 0) {
		////receive bytes of instruction if allowed
		//midi_buff[midi_buff_point] = uart_data;
		//midi_buff_point++;
		//midi_buff_allowed--;
		//
		///************************************************************************/
		///*                    MIDI Buffer translate                             */
		///************************************************************************/
		//
		//if ((midi_buff_point == 3)) //3byte min for whole note message rx
		//{
			//midi_buff_point = 0;
			////set_LED(DISABLE);
			////	asm volatile ("nop");
			//
			//
			////LEARN MODE ONLY:
			//if ((midi_learn_mode) && (midi_learn_current<8) && ((midi_buff[0]&0xF0)==MIDI_NOTE_ON)){//learn map from pressed button and midi RX		
				//
				//if (((midi_buff[0]&0x0F) != midi_channel)&&(midi_learn_current==0))
					//midi_channel = (midi_buff[0]&0x0F); //IF CHANNEL 1 SELECTED ENABLE MIDI CHANNEL OVERRIDE
				//
				//if (((midi_buff[0]&0x0F) == midi_channel) && (midi_buff[2] != 0)){
					////ACTUALLY LEARN MAP FOR SELECTED CHANNEL HERE:
					//midi_note_map[midi_learn_current] = midi_buff[1];
		//
					//
					////SET GATE ON CHANNEL TO SHOW STATUS
					//if(midi_learn_current == 0)
						//PORTB &= 0xFE;//inverted cos of 74HC1G14 inverter 
					//else
						//PORTD &= 0xFF^(1 << midi_learn_current);//inverted cos of 74HC1G14 inverter 
		//
					//midi_learn_current++; //next channel	
				//}
				//if(midi_learn_current>=8)
					//midi_learn_mode=0;//DONE LEARNING! 
			//}
			//
			//
			////BUFFER TRANSLATE
			//if (((midi_buff[0]&0x0F) == midi_channel) && !(midi_learn_mode))
			//{
				//uint8_t i = 0;
				////GATE 1
				//if(midi_buff[1]==midi_note_map[0]){
					//if (((midi_buff[0]&MIDI_NOTE_ON)==MIDI_NOTE_ON) && (midi_buff[2] != 0)){//&&(midi_buff[2] != 0))
						////if(velocity_out && !(presets.velocity_mute && ((mute_state>>i)&1)))		//when velocity expander available
						//set_velocity(0,midi_buff[2]);
						//PORTB &= 0xFE;//inverted cos of 74HC1G14 inverter 
					//}
					//else
					//PORTB |= 1; //
				//}
				////GATES 2-8 other Port
				//for (i=1;i<8;i++){
					//if(midi_buff[1]==midi_note_map[i]){
						//if (((midi_buff[0]&MIDI_NOTE_ON)==MIDI_NOTE_ON) && (midi_buff[2] != 0)){//&&(midi_buff[2] != 0))
							//if(velocity_out)
							//set_velocity(i,midi_buff[2]);
							//PORTD &= 0xFF^(1 << i);//inverted cos of 74HC1G14 inverter 
						//}else
						//PORTD |= (1 << i);
					//}
				//}
			//}
			//
		//}
	//}
} //end ISR


//void set_velocity(uint8_t ch, uint8_t velo){
	//
	//velo = velo & 0x7F; 
	//
	//max5825_set_load_channel((ch&0x0F),velocity_lookup[velo]);
	//
	//return;
//}
}


void set_LED(uint8_t var){
	
	if (var==ENABLE)
		DDRC |= (1<<LED_pin);
	else	
		DDRC &= 0xFF^(1 << LED_pin);
	
	
	return;
}


void start_timer1(uint8_t cmp_value){
	TCNT1 = 0;
//	ASSR = 0;//(1 << AS0); //sync op - clock <= system clock
	TCCR1A = 0;
	TCCR1B = (0b101<<CS00);///1024
	OCR1A = cmp_value; // 157;//ca. 10ms@16MHz
	TIFR |= (1 << OCF1A);	
	TIMSK |= (1 << OCIE1A);
	return;
}


void set_pin_inv(uint8_t pinnr){
	//inverted cos of 74HC1G14 inverter
	switch(pinnr){
		case 0: PORTB &= 0xFE;
				break;
		case 1: PORTD &= 0xFD;
				break;		
		case 2: PORTD &= 0xFB;
				break;
		case 3: PORTD &= 0xF7;
				break;		
		case 4: PORTD &= 0xEF;
				break;		
		case 5: PORTD &= 0xDF;
				break;		
		case 6: PORTD &= 0xBF;
				break;		
		case 7: PORTD &= 0x7F;
				break;			
		default: break;
	}
	
	return;
}

void clear_pin_inv(uint8_t pinnr){
	switch(pinnr){
		case 0: PORTB |= 0x01;
				break;
		case 1: PORTD |= 0x02;
				break;
		case 2: PORTD |= 0x04;
				break;
		case 3: PORTD |= 0x08;
				break;
		case 4: PORTD |= 0x10;
				break;
		case 5: PORTD |= 0x20;
				break;
		case 6: PORTD |= 0x40;
				break;
		case 7: PORTD |= 0x80;
				break;
		default: break;
	}	
		
	return;
}