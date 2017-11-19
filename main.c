        /*
 * main.c    ATtiny45    F_CPU = 8000000 MHz     UART 115 200 /8/1
 *         Przyklad dla czasowego zalaczania/wylaczania pompy  oraz regulacji czujnikiem NTC
 *
 * pomoce: https://et06.dunweber.com/atmega_timers/
 *         http://diycenter.acid19.linuxpl.com/readarticle.php?article_id=3
 *         http://mikrokontrolery.blogspot.com/2011/03/led-sterowany-przez-timer.html
 *         Kardaœ, Bluebook strona 43 o licznikach
           Zrodlo informacji o NTC w powiazaniu z ADC atmegi (m in. tabela):
           http://aterlux.ru/index.php?page=article&art=ntcresistor-en
           opis obslugi przycisku trzymanego krotko i dlugo
           http://forum.atnel.pl/post152780.html#p152780
           film o eeprom:
           https://www.youtube.com/watch?v=W7f0EnDzS_M

 ATTiny45 Hookup
            RESET PB5 -|1 v 8|- Vcc
    (NTC ADC ->)  PB3 -|2   7|- PB2/SCK        (-> LED)
         (KEY->)  PB4 -|3   6|- PB1/MISO/DO    (-> Pompa)
                  GND -|4 _ 5|- PB0/MOSI/SDA   (-> Tx)

 PB2 jako LED  (zielony)
 PB4 jako klawisz


     Zasada dzialania:
     1. Po wlaczeniu zasilania wlacza sie zielona dioda LED
     2. Przez 30 sekund jest czas na ustawienie przyciskiem KEY nastawy temperatury wylaczenia pompy. Ka¿de wciœniecie jest sygnalizowanie
        przez mrugniecie zielonej diody LED.
     3. Gdy up³ynie 30 sekund zielona dioda LED zamruga szybko oko³o 5 razy.
     4. Nastepnie przez kolejne 30 sekund jest czas na ustawienie przyciskiem KEY nastawy histerezy zalaczenia pompy.
     5. Gdy up³ynie 30 sekund zielona dioda LED zamruga szybko oko³o 5 razy.
     6. Jesli by³y dokonywane zmiany to teraz zostan¹ zapisane w pamieci.
     7. Po 9 sekundach zielona dioda LED zamruga co sekunde tyle razy ile wynosi nastawa offsetu temperatury wylaczenia pompy.
     8. Po 5 sekundach zielona dioda LED zamruga co sekunde tyle razy ile wynosi nastawa histerezy temperatury zalaczenia pompy.
     9. Poczatkowa wartosc nastaw to 29 st C i histereza 3 st. C.
        Pompa pracuje bez regulacji temperaturowej przez pierwsze 80 minut od wlaczenia zasilania.
    10. Nastawy mo¿na zresetowac, gdy w trakcie wlaczenia zasilania przycisk KEY jest wcisniety.
        - Jesli chcesz zmienic temperature to zmien w if (srednieADC ..)  wartosc np 540 na inna
        - Jesli chcesz zmienic dlugosc poczatkowego wlaczenia pompy bez sterowania temperaturowego zmien wartosc funkcji minuty()
    11. Temperatura jest wskazywana przez diodê zielon¹ w zakresie 31-39 st C przez mruganie i powyzej 40 st C przez swiecenie ciagle.
        Cykl wskazywania temperatury trwa 10 sekund, tzn np. gdy jest 36 st C to zielona dioda œwieci przez 6s a nastepnie nie swieci przez 4 sekundy


    Wersja 2017_06 zmieniono  poczatkow¹ nastawê na 29 st C
 */
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "common.h"
#include "softUART/softuart.h"

// dioda LED zielona
#define LED_PIN (1<<PB2)            // definicja pinu do ktorego podlaczona jest zielona dioda
#define LED_ON PORTB &= ~LED_PIN    // makrodefinicja  zalaczenie diody
#define LED_OFF PORTB |= LED_PIN    // makrodefinicja  wylaczenie diody
#define LED_TOG PORTB ^= LED_PIN    // makrodefinicja  zmiana stanu diody

// pomocnicze makra do obs³ugi pompy
#define Pompa_PIN (1<<PB1)              // definicja pinu do ktorego podlaczona jest pompa
#define Pompa_ON PORTB |= Pompa_PIN    // makrodefinicja zalaczenie pompy
#define Pompa_OFF PORTB &=~ Pompa_PIN    // makrodefinicja wylaczenie pompy


void minuty( uint8_t min );              //minutnik
void srednia( uint16_t adc );
uint16_t pomiar( void );

uint8_t MM, SS, sekundyLED, LEDmruga;

//#define SR 8    // rozmar bufora do celów uœredniania  JKA zamienilem na osiem w funkcji


// pomocnicze zmienne i makra

uint16_t SrednieADC, odczytADCdlaLED;



#define KEY (1<<PB4)

uint8_t key_lock;
uint8_t i;


// dane w pamiêci Flash oraz ich inicjalizacja ustawieñ fabrycznych w pamiêci Flash
const TCFG pgm_cfg PROGMEM = {
    T_OFFSET,
    T_HISTEREZA,
};

TCFG eem_cfg EEMEM;            // dane w pamiêci EEPROM
TCFG ram_cfg;                  // dane w pamiêci RAM

void sekundy( uint16_t sec );              //ustawiamy ilosc sekund jako mnoznik 50, np 3 minuty to 50*60*3 = 9000
uint16_t licznik = 0;

int main( void ) {

    PORTB |= KEY;  // podci¹gamy linie klawiszy do VCC

    DDRB |= ( 1 << PB2 );          // kierunek pinu wyjsciowy PB2-dioda zielona
    PORTB |= ( 1 << PB2 );         // dioda LED zgaszona, tzn stan wysoki bo podpiete do VDD


    check_and_load_defaults();    //sprawdza poprawnosc zapisanych danych w EEPROM

    uart_puts( "Witaj\r\n" );

    if ( !( PINB & KEY ) )   {
        load_defaults();   //gdy przy zalaczaniu jest wcisniety przycisk to laduje ustawienia domyslne

        for ( i = 0;i < 10;i++ ) {          //wskazanie swietlne w wykonanym resecie
            LED_TOG;
            _delay_ms( 500 );
        }
    }

    LED_ON;

    //######## konfiguracja timera ATtiny45 (niestety 8 bitowego)- uwaga jest inaczej niz w ATmega ##############
//  TIMSK |= (1 << OCIE0A);  // Zezwolenie na przerwania CTC (compare Match) -wylaczone bo uzywamy flagi: patrz sekundnik
    TCCR0A |= ( 1 << WGM01 );  // Ustawia timer w tryb CTC
    TCCR0B |= ( 1 << CS00 ) | ( 1 << CS02 );     // Ustawia preskaler 1024, czyli Fcpu/1024  daje 7 812.5 Hz
    OCR0A = 155;           // Ustawia wartoœæ po¿¹dan¹ na 50,08 Hz, zliczanie do tej wartosci, przepelnienie i ponowne liczenie od zera


//    while(1) {

    while ( licznik <= 1500 ) {          //czas okolo 30 sekund
        if ( TIFR & ( 1 << OCF0A ) ) {   //wyzwolona flaga w rejestrze timera CTC
            licznik++;
            TIFR = ( 1 << OCF0A ) ;  //kasowanie flagi timera CTC , ma "=" a nie "|="
        }


        if ( !key_lock && !( PINB & KEY ) ) {
            key_lock = 1;

            // reakcja na PRESS (wciniêcie przycisku)
            ram_cfg.t_offset++;            //zwiekszamy o 1 st. C
            if ( ram_cfg.t_offset == 15 )  ram_cfg.t_offset = 0; //jesli zwiekszylismy o 15 st C to zerujemy


            LED_OFF;
            _delay_ms( 200 );
            LED_ON;
            _delay_ms( 1000 );
            LED_OFF;


        } else if ( key_lock && ( PINB & KEY ) ) key_lock = 0;

    }

    licznik = 0;

    for ( i = 0;i < 10;i++ ) {            //zamrugamy kilka razy
        LED_TOG;
        _delay_ms( 300 );
    }

    LED_ON;
    while ( licznik <= 1500 ) {           //czas okolo 30 sekund
        if ( TIFR & ( 1 << OCF0A ) ) {   //wyzwolona flaga w rejestrze timera CTC
            licznik++;
            TIFR = ( 1 << OCF0A ) ;  //kasowanie flagi timera CTC , ma "=" a nie "|="
        }


        if ( !key_lock && !( PINB & KEY ) ) {
            key_lock = 1;

            // reakcja na PRESS (wciniêcie przycisku)
            ram_cfg.t_histereza ++;            //zwiekszamy o 1 st. C

            LED_OFF;
            _delay_ms( 200 );
            LED_ON;
            _delay_ms( 1000 );
            LED_OFF;


        } else if ( key_lock && ( PINB & KEY ) ) key_lock = 0;
    }
    LED_OFF;

    for ( i = 0;i < 10;i++ ) {            //zamrugamy kilka razy
        LED_TOG;
        _delay_ms( 300 );
    }
    LED_OFF;

    copy_ram_to_eem();
    uart_putlong( ram_cfg.t_offset, 10 );
    uart_puts( "\n\r" );
    uart_putlong( ram_cfg.t_histereza, 10 );
    uart_puts( "\n\r" );
    uart_puts( "Koniec.\r\n " );




    _delay_ms( 9000 );
    for ( i = 0;i < ram_cfg.t_offset;i++ ) {             //zamrugamy kilka razy
        LED_ON;
        _delay_ms( 1000 );
        LED_OFF;
        _delay_ms( 1000 );
    }


    _delay_ms( 5000 );
    for ( i = 0;i < ram_cfg.t_histereza;i++ ) {             //zamrugamy kilka razy
        LED_ON;
        _delay_ms( 1000 );
        LED_OFF;
        _delay_ms( 1000 );
    }

// **********************POLACZENIE ************************

//########### I/O ###########
    DDRB |= ( 1 << PB1 );   // wyjscie pompy
    DDRB &= ~( 1 << PB3 );   // wejscie czujnika NTC


    // inicjalizacja ADC
    ADCSRA |= ( 1 << ADEN );  // wlaczenie ADC
    ADCSRA |= ( 1 << ADIE ); // ADC Interrupt Enable. ADC Conversion Complete Interrupt is activated.
    ADCSRA |= ( 1 << ADPS2 ) | ( 1 << ADPS1 ) | ( 1 << ADPS0 ); // preskaler = 128 dla 8MHz
//  ADMUX |= REF_VCC;    // ustawiamy napiecie odniesienia 5V   wiec nic nie nalezy podawac bo jest fabrycznie 0.
// patrz nizej **pomiar**    ADMUX |=  (1<<MUX1)|(1<<MUX0);     // (PB3) wybrane wejscie do pomiaru Single Ended Input. (lub patrz technika Kardasia Pomiar)

    _delay_ms( 1000 );
    Pompa_ON;    // pompa zalaczona
    minuty( 80 );   //czas wlaczenia pompy bez sterowania temperaturowego



    while ( 1 ) {          //g³ówna pêtla programu


        SS++;    /* zwiêksz licznik sekund */
        sekundyLED++;           //zwieksz licznik sekund wyswietlania temperatury przez diode LED
        uart_puts( "sec: " );
        uart_putlong( SS, 10 );
//                    uart_puts("\n\n\r");

        pomiar();
        uart_puts( "   ADC: " );
        uart_putlong( ADC, 10 );
//                   uart_puts("\n\n\r");

        srednia( ADC );
        uart_puts( "   Sr: " );
        uart_putlong( SrednieADC, 10 );
        uart_puts( "\n\n\r" );


        if ( SS > 59 ) {
            SS = 0; /* jeœli iloœæ sekund > 59 - wyzeruj */
            MM++;
            uart_puts( "min: " );
            uart_putlong( MM, 10 );
            uart_puts( "\n\n\r" );
        }

        if ( sekundyLED > 9 ) {
            sekundyLED = 0; /* jeœli iloœæ sekund wyswietlania LED > 9 - wyzeruj */
            odczytADCdlaLED = SrednieADC;

       //########### wskazywanie dioda temperatury 31-39 st C przez mruganie i powyzej 40 st C przez swiecenie ciagle

         if ((odczytADCdlaLED >= 577) && (odczytADCdlaLED <=586)) LEDmruga = 1;   // temperatura = 31 st C lub wiecej
         if ((odczytADCdlaLED >= 587) && (odczytADCdlaLED <=597)) LEDmruga = 2;   // temperatura = 32 st C lub wiecej
         if ((odczytADCdlaLED >= 598) && (odczytADCdlaLED <=607)) LEDmruga = 3;   // temperatura = 33 st C lub wiecej
         if ((odczytADCdlaLED >= 608) && (odczytADCdlaLED <=617)) LEDmruga = 4;   // temperatura = 34 st C lub wiecej
         if ((odczytADCdlaLED >= 618) && (odczytADCdlaLED <=627)) LEDmruga = 5;   // temperatura = 35 st C lub wiecej
         if ((odczytADCdlaLED >= 628) && (odczytADCdlaLED <=637)) LEDmruga = 6;   // temperatura = 36 st C lub wiecej
         if ((odczytADCdlaLED >= 638) && (odczytADCdlaLED <=646)) LEDmruga = 7;   // temperatura = 37 st C lub wiecej
         if ((odczytADCdlaLED >= 647) && (odczytADCdlaLED <=656)) LEDmruga = 8;   // temperatura = 38 st C lub wiecej
         if ((odczytADCdlaLED >= 657) && (odczytADCdlaLED <=665)) LEDmruga = 9;   // temperatura = 39 st C lub wiecej
         if ( odczytADCdlaLED >= 666) LEDmruga = 10;  // temperatura = 40 st C lub wiecej , swieci ciagle


        }


        if (LEDmruga > 0)
        {
          LED_ON;
          LEDmruga--;
        }
        else
        {
         LED_OFF;
        }

        //              if(ADC >= 557)    // temperatura zalaczenia = 29 st C
 //       if ( SrednieADC >= ( 520 + 10*ram_cfg.t_offset ) )    // temperatura zalaczenia = 26 st C    wartoœæ wyniku uœredniona
        if ( SrednieADC >= ( 556 + 10*ram_cfg.t_offset ) )    // temperatura zalaczenia = 29 st C    wartoœæ wyniku uœredniona
            // 523 = 26,00 st C
            // 534 = 27,00 st C  dla rezystora 10K    V_REFF_VCC
            // 556 = 29,00 st C
            // 680 = 41,00 st C
            // 671 = 40,03 st C
            // 662 = 39,07 st C
            // 652 = 38,02 st C
            // 642 = 37,00 st C
            // 632 = 35,99 st C
            // 622 = 35,00 st C
            // 577 = 31,00 st C

            // 755 = 50,01 st C
            // 791 = 55,12 st C
            // 822 = 60,14 st C
        {
            Pompa_ON;    // pompa zalaczona
            minuty( 3 ); // w³¹czamy opóŸnienie w regulacji temperaturowej pompy
        }

         //    if(SrednieADC <= 540 - 40)    //    histereza (28 - 4  temperatura wylaczenia = 24 st C  wartoœæ wyniku uœredniona
 //       if ( SrednieADC <= ( 520 + 10*( ram_cfg.t_offset - ram_cfg.t_histereza ) ) )
         if ( SrednieADC <= ( 556 + 10*( ram_cfg.t_offset - ram_cfg.t_histereza ) ) )  // 29 - 3 czyli temp. wylaczenia = 26 st C

        {
            Pompa_OFF;             //pompa wylaczona
        }



        _delay_ms( 1000 );  //sekunda

    }
}




// ***TO JEST ZBEDNE ********
//  void sekundy(uint16_t sec){
//     licznik = 0;
//      while(licznik <= sec) {
//                if (TIFR & (1 << OCF0A)){   ////wyzwolona flaga w rejestrze timera CTC
//                    licznik++;
//                    TIFR = (1 << OCF0A) ;    //kasowanie flagi timera CTC , ma "=" a nie "|="
//                    }
//            }
//       }


//KONIEC *************************



/*

*/
// do³¹czanie systemowych plików nag³ówkowych






// pomiar ADC
uint16_t pomiar( void ) {

//  ADMUX = (ADMUX & 0b11111000) | kanal;     //tak bylo u Kardasia
    ADMUX |= ( 1 << MUX1 ) | ( 1 << MUX0 );     // (PB3) wybrane wejscie do pomiaru Single Ended Input
    ADCSRA |= ( 1 << ADSC );  // start konwersji

    while ( ADCSRA & ( 1 << ADSC ) );

    return ADCW;            // to jest udogodnienie w kompilatorze avr-gcc, ze zwraca wynik juz 16 bitowy
}




void minuty( uint8_t min ) {
    SS = 0;
    MM = 0;

    while ( MM != min ) {

        SS++;    // zwiêksz licznik sekund
        uart_puts( "s minutnika: " );
        uart_putlong( SS, 10 );
//                    uart_puts("\n\n\r");
        pomiar();             //dodano pomiar i usrednianie bo gdy skonczy sie wykonywac ta funkcja to
        srednia( ADC );       // wartosc srednia pomiaru byla by poczatkowo bledna
        uart_puts( "   Sr: " );
        uart_putlong( SrednieADC, 10 );
        uart_puts( "\n\n\r" );

        if ( SS > 59 ) {
            SS = 0; // jeœli iloœæ sekund > 59 - wyzeruj
            MM++;
            uart_puts( "m minutnika: " );
            uart_putlong( MM, 10 );
            uart_puts( "\n\n\r" );
        }
        _delay_ms( 1000 );       //sekunda

    }
}


// usrednianie ADC  dla osmiu pomiarow
void srednia( uint16_t adc ) {

    static uint16_t sr[8];
    static uint8_t idx;
    uint32_t sr1 = 0;
    uint8_t i;

    sr[ idx++ & ( 8-1 )] = adc;

    for ( i = 0; i < 8; i++ ) sr1 += sr[i];
    sr1 /= 8;

    SrednieADC = sr1;
}

//KONIEC *************************