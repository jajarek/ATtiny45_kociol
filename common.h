/*
 * common.h
 *
 *  Created on: 24-11-2012
 *      Author: Miros³aw Kardaœ
 */

#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#ifndef COMMON_H_
#define COMMON_H_



typedef struct {
    uint16_t t_offset;
    uint8_t t_histereza;
} TCFG;




// ustawienia domylne (fabryczne)
#define T_OFFSET 0          // nastawa temperatury za³¹czenia pompy - 26 st C   (29 - T_HISTEREZA)
#define T_HISTEREZA 3        // histereza ZAL/WYL



extern const TCFG pgm_cfg PROGMEM;        // dane w pamiêci FLASH
extern TCFG eem_cfg EEMEM;        // dane w pamiêci EEPROM
extern TCFG ram_cfg;        // dane w pamiêci RAM

extern volatile uint16_t Timer1, Timer2, Timer3, Timer4;    /* timery programowe 100Hz */

extern char bufor[100];


void check_and_load_defaults( void );
void copy_eem_to_ram( void );
void copy_ram_to_eem( void );
void copy_pgm_to_ram( void );
void load_defaults( void );

void soft_timer_init( void );
void pokaz_alarmy( void );

#endif /* COMMON_H_ */