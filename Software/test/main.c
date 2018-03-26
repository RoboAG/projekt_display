/*******************************************************************************
*                                                                              *
* Version: 1.0.0                                                               *
* Datum: 14.09.10                                                              *
* Autor: Peter Weissig                                                         *
*                                                                              *
*******************************************************************************/


//*********************************<Eingebundene Dateien>***********************
// Standard Dateien
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

// Meine Eigenen Dateien
#include "display.h"

//*********************************<Typen>**************************************

//*********************************<Makros>*************************************
#define Motor(x)  (x ? (PORTB|= 0x01) : (PORTB&= ~0x01))
#define Cursor(x) (x ? (PORTD|= 0x20) : (PORTD&= ~0x20))
#define Leds(x) {PORTC = x & 0x3F; PORTD = (x & 0xC0) | (PORTD & 0x3F);}

//*********************************<Methoden>***********************************
void Init_Hardware(void);
void Blinken(void);
void test_UART(void);

//*********************************[Init_Hardware]******************************
void Init_Hardware(){
  DDRB  = 0x03; // Pin 0   ==> Relay
                // Pin 1   ==> PWM für IR-UART (OC1A)
                // Pin 2-4 ==> SS, MOSI, MISO, SCK
  DDRC  = 0x3F; // Pin 0-5 ==> Pixel 0-5
  DDRD  = 0xE2; // Pin 0-1 ==> RxD, TxD
                // Pin 2   ==> Optokoppler für Interrupt (INT0)
                // Pin 5   ==> Pixel Cursor
                // Pin 6-7 ==> Pixel 6-7

  // Timer1 ==> PWM für IR-UART
  TCCR1A = 0x40; // Toggle Output A; CTC bis OCR1A
  TCCR1B = 0x09; // CTC bis OCR1A; Prescaler: 1

  OCR1AH =   0; // 36kHz
  OCR1AL = 111; // 36kHz

  TCNT1H = 0x00; // Timer 1 zurücksetzen
  TCNT1L = 0x00; // Timer 1 zurücksetzen

  TIMSK = 0x00; // Keine Interrupts
}

//*********************************[Blinken]************************************
void Blinken(){
  uint8_t speed = 10;
  uint16_t dauer = 25;

  while (1) {
    speed++;
    if (speed > 7) {speed = 0; dauer = 25;}
    if (speed < 4) {dauer = dauer *2;} else {dauer = dauer / 2;}

    Leds(0x01); mdelay(dauer);
    Leds(0x02); mdelay(dauer);
    Leds(0x04); mdelay(dauer);
    Leds(0x08); mdelay(dauer);
    Leds(0x10); mdelay(dauer);
    Leds(0x20); mdelay(dauer);
    Leds(0x40); mdelay(dauer);
    Leds(0x80); mdelay(dauer);

    Leds(0x80); mdelay(dauer);
    Leds(0x40); mdelay(dauer);
    Leds(0x20); mdelay(dauer);
    Leds(0x10); mdelay(dauer);
    Leds(0x08); mdelay(dauer);
    Leds(0x04); mdelay(dauer);
    Leds(0x02); mdelay(dauer);
    Leds(0x01); mdelay(dauer);
  }
}

//*********************************[test_UART]**********************************
void test_UART(void){
  uint8_t a = 0;
  uint8_t b = 0;

  while (1) {
    serout0(a);
    mdelay(50);
    if (serstat0()) {
      while (serstat0()) {b = serinp0();}
      if (a == b) {
        Leds(a);
        mdelay(50);
      } else {
        Cursor(1);
        Leds(a); mdelay(500); Leds(b); mdelay(500);
        Leds(a); mdelay(500); Leds(b); mdelay(500);
        Leds(a); mdelay(500); Leds(b); mdelay(500);
        Leds(a); mdelay(500); Leds(b); mdelay(500);
        Leds(a); mdelay(500); Leds(b); mdelay(500);
        Cursor(0);
      }
    } else {
      Leds(0);
      Cursor(1); mdelay(100); Cursor(0); mdelay(100);
      Cursor(1); mdelay(100); Cursor(0); mdelay(100);
      Cursor(1); mdelay(100); Cursor(0); mdelay(100);
      Cursor(1); mdelay(100); Cursor(0); mdelay(100);
      Cursor(1); mdelay(100); Cursor(0); mdelay(100);
    }

    a++;
  }
}

//*********************************[Main]***************************************
int main (void) {
  Init_System();
  Init_Hardware();
  sei();

  test_UART();
  while (1) {mdelay (10);}
  return (0);
}
