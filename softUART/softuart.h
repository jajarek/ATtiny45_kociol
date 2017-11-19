/*
 * softuart.h
 *    wersja JKA
 * Attiny45 8MHz only Tx
 */





#define UART_TX PB0 // Use PB0 as TX pin


void uart_putc(char c);
void uart_puts( char * s );
void uart_puts_P(const char *s);
void uart_putlong( uint32_t liczba, uint8_t radix );
