/*
 * common.c
 *
 *  Created on: 24-11-2012
 *      Author: Miros³aw Kardaœ
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#include "common.h"




volatile uint16_t Timer1, Timer2, Timer3, Timer4;    /* timery programowe 100Hz */

char bufor[100];








void copy_eem_to_ram( void ) {
    eeprom_read_block( &ram_cfg, &eem_cfg, sizeof(ram_cfg) );
}

void copy_ram_to_eem( void ) {
    eeprom_write_block( &ram_cfg, &eem_cfg, sizeof(ram_cfg) );
}

void copy_pgm_to_ram( void ) {
    memcpy_P( &ram_cfg, &pgm_cfg, sizeof(ram_cfg) );
}

void load_defaults( void ) {
    copy_pgm_to_ram();
    copy_ram_to_eem();
}

void check_and_load_defaults( void ) {
    uint8_t i, len = sizeof( ram_cfg );
    uint8_t * ram_wsk = (uint8_t*)&ram_cfg;

    copy_eem_to_ram();
    for(i=0; i<len; i++) {
        if( 0xff == *ram_wsk++ ) continue;
        break;
    }

    if( i == len ) {
        load_defaults();
    }

}




