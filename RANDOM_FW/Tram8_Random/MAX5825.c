/*
 * MAX5825.c
 *
 * Created: 22.08.2017 16:49:13
 * Author : Kay Knofe LPZW.modules
 * www.leipzigwest.org
 *
 *	SOFTWARE License:
 *	CC BY-NC-SA 4.0
 */

#include <avr/io.h>
#include "MAX5825.h"
#include "general_twi.h"

void init_max5825(void){
		
	

	uint8_t data[2] = {NULL};
	TWI_WRITE_BULK(MAX5825_ADDR,(MAX5825_REG_REF | 0b101),2,&data); //INT REF 0b111=4.096V //0b101=2.5V ALL DAC PWR ON 	
		
	TWI_WRITE_BULK(MAX5825_ADDR,MAX5825_REG_CODELOADALL,2,&data); // all to zero 
		
	
	return;
}

uint8_t test_max5825(void){
	uint8_t retval = 0;
	uint8_t buffer[2] = {NULL};
	
	retval = TWI_READ_BULK(MAX5825_ADDR,0,2,&buffer);
	
	return retval;
}


void max5825_set_load_channel(uint8_t ch, uint16_t value){
	
//contains a 16 bit number that gets right-shifted (65535 >> 4 -> 4095) split into upper and lower bytes, before the shift.
// this leads to a change in output value for every sixteen input value units --> 0 + 16*n before the right shift.

	uint8_t cmd_addr = (MAX5825_REG_CODEn_LOADn | (ch & 0x0F));
	uint8_t data[2]; 
	
	data[0]= (uint8_t) ((value>>8) & 0xFF);
	data[1]= (uint8_t) (value & 0xF0);

	TWI_WRITE_BULK(MAX5825_ADDR,cmd_addr,2,&data);
	
	return;
}


void max5825_set_load_all(max_fill_struct * data){
	uint8_t cmd_addr = (MAX5825_REG_CODEn | 0);

	TWI_WRITE_BULK(MAX5825_ADDR,cmd_addr,23,data);	
}