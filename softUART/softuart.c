/*
 * softuart.c
 *
 *  Created on: 24-06-2013
 *      Author: Miros³aw Karadœ
 */
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <avr/interrupt.h>

#include "softuart.h"




//# define        UART_BAUDRATE   (19200)    //to jest oryginal dla F_CPU 1 200 000 Hz
# define UART_BAUDRATE   (115200)


#define TXDELAY  (int)(((F_CPU/UART_BAUDRATE)-7 +1.5)/3)
#define RXROUNDED           (((F_CPU/UART_BAUDRATE)-5 +2)/3)
#if RXROUNDED > 127
# error Low baud rates unsupported - use higher UART_BAUDRATE
#endif


void uart_putc(char c)
{
    uint8_t sreg;

    sreg = SREG;
    cli();
    PORTB |= (1<<UART_TX);
    DDRB |= (1<<UART_TX);
    __asm volatile(
        " cbi %[uart_port], %[uart_pin] \n\t" // start bit
        " in r0, %[uart_port] \n\t"
        " ldi r30, 3 \n\t" // stop bit + idle state
        " ldi r28, %[txdelay] \n\t"
        "TxLoop: \n\t"
        // 8 cycle loop + delay - total = 7 + 3*r22
        " mov r29, r28 \n\t"
        "TxDelay: \n\t"
        // delay (3 cycle * delayCount) - 1
        " dec r29 \n\t"
        " brne TxDelay \n\t"
        " bst %[ch], 0 \n\t"
        " bld r0, %[uart_pin] \n\t"
        " lsr r30 \n\t"
        " ror %[ch] \n\t"
        " out %[uart_port], r0 \n\t"
        " brne TxLoop \n\t"
        :
        : [uart_port] "I" (_SFR_IO_ADDR(PORTB)),
        [uart_pin] "I" (UART_TX),
        [txdelay] "I" (TXDELAY),
        [ch] "r" (c)
        : "r0","r28","r29","r30"
    );
    SREG = sreg;
}

//void uart_puts(const char *s){
void uart_puts(char *s){
         while (*s) uart_putc(*(s++));
}

void uart_putlong( uint32_t liczba, uint8_t radix ) {
    char buf[17];
    ltoa( liczba, buf, radix );
    uart_puts( buf );
}

void uart_puts_P(const char *s)        // wysy³a ³añcuch z pamiêci RAM na UART
{
  register char c;
  while ((c = pgm_read_byte( s++) )) uart_putc(c);            // dopóki nie napotkasz 0 wysy³aj znak
}
