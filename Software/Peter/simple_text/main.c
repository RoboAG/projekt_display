/*******************************************************************************
*                                                                              *
* Version: 1.0.0                                                               *
* Datum: 24.09.10                                                              *
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
struct TBild {
  uint8_t daten[128];
  uint8_t cursorA,cursorE; // 0..128 Position auf dem Display ; A = Anfang, E = Ende
};

struct TDisplay {
  volatile uint16_t dauer; // Dauer für eine Umdrehung
  volatile uint16_t zeit;  // Zeitpunkt des letzten Interrupts
  volatile uint8_t  sek;   // Wird auf 0 Runtergezählt
  volatile uint8_t  pos;   // Nr. der momentanen Spalte (0..127);
  volatile uint8_t  bild;  // Nr. des momentanen Bildes (0..1)
};

struct TEffekt {
  uint8_t  speedin;  // Einblendgeschwindigkeit in 10ms je Spalte
  uint8_t  speedout; // Ausblendgeschwindigkeit in 10ms je Spalte
  uint8_t  ein,estay,eout; // Ein-,Warte-& Ausblendeffekt
  int16_t  pos;      // Position des Textes (128 - Nicht sichtbar); Alt: Nr. des momentanen Zeichens & dessen Pixelreihe
  uint8_t  anz;      // Wie oft der Effekt noch wiederholt wird
  uint16_t zeit;     // Zeitpunkt der letzten Änderung
  uint16_t zeit_warten;// Zeit, wie lange der Text so dasteht in 100ms
};

struct TText {
  uint8_t  text[64];// Anzuzeigender Text; erstes Byte ist Längenbyte
  uint16_t tposs;   // Anfang des Aktuellen Textblocks im EEPROM
  uint16_t tpose;   // Ende   des Aktuellen Textblocks im EEPROM
  struct TEffekt effekt;
};


//*********************************<Methoden>***********************************
void Init_Hardware(void);
static inline void Variable_laden(void);
void Init_Variable(void);
ISR(INT0_vect);
ISR(TIMER1_COMPA_vect);
ISR(TIMER1_COMPB_vect);
ISR(TIMER2_COMP_vect);

void Startsequenz(void);
void Blinken(void);
void test_UART(void);
void test_Optokoppler(void);
void Bild_Left(void);
void Bild_Right(void);
void Bild_Load(void);
void Bild_Effekt(void);

//*********************************<Konstanten>*********************************
const PROGMEM uint8_t ASCII_flash[97][6] = {
  /* */ {0x50,0x00,0x00,0x00,0x00,0x00},/*!*/ {0x21,0x4F,0x00,0x00,0x00,0x00},
  /*"*/ {0x43,0x07,0x00,0x07,0x00,0x00},/*#*/ {0x65,0x14,0x7F,0x14,0x7F,0x14},
  /*$*/ {0x65,0x24,0x2A,0x7F,0x2A,0x12},/*%*/ {0x65,0x23,0x13,0x08,0x64,0x62},
  /*&*/ {0x65,0x36,0x49,0x55,0x22,0x50},/*'*/ {0x32,0x05,0x03,0x00,0x00,0x00},
  /*(*/ {0x43,0x1C,0x22,0x41,0x00,0x00},/*)*/ {0x43,0x41,0x22,0x1C,0x00,0x00},
  /***/ {0x65,0x14,0x08,0x3E,0x08,0x14},/*+*/ {0x65,0x08,0x08,0x3E,0x08,0x08},
  /*,*/ {0x32,0x50,0x30,0x00,0x00,0x00},/*-*/ {0x65,0x08,0x08,0x08,0x08,0x08},
  /*.*/ {0x32,0x60,0x60,0x00,0x00,0x00},/*/*/ {0x65,0x20,0x10,0x08,0x04,0x02},

  /*0*/ {0x65,0x3E,0x51,0x49,0x45,0x3E},/*1*/ {0x64,0x00,0x42,0x7F,0x40,0x00},
  /*2*/ {0x65,0x42,0x61,0x51,0x49,0x06},/*3*/ {0x65,0x21,0x41,0x45,0x4B,0x31},
  /*4*/ {0x65,0x18,0x14,0x12,0x7F,0x10},/*5*/ {0x65,0x27,0x45,0x45,0x45,0x39},
  /*6*/ {0x65,0x3C,0x4A,0x49,0x49,0x30},/*7*/ {0x65,0x01,0x71,0x09,0x05,0x03},
  /*8*/ {0x65,0x36,0x49,0x49,0x49,0x36},/*9*/ {0x65,0x06,0x49,0x49,0x29,0x1E},
  /*:*/ {0x32,0x36,0x36,0x00,0x00,0x00},/*;*/ {0x32,0x56,0x36,0x00,0x00,0x00},
  /*<*/ {0x54,0x08,0x14,0x22,0x41,0x00},/*=*/ {0x65,0x14,0x14,0x14,0x14,0x14},
  /*>*/ {0x54,0x41,0x22,0x14,0x08,0x00},/*?*/ {0x65,0x02,0x01,0x51,0x09,0x06},

  /*@*/ {0x65,0x32,0x49,0x79,0x41,0x3E},/*A*/ {0x65,0x7E,0x11,0x11,0x11,0x7E},
  /*B*/ {0x65,0x7F,0x49,0x49,0x49,0x36},/*C*/ {0x65,0x7E,0x41,0x41,0x41,0x22},
  /*D*/ {0x65,0x7F,0x41,0x41,0x22,0x1C},/*E*/ {0x65,0x7F,0x49,0x49,0x49,0x41},
  /*F*/ {0x65,0x7F,0x09,0x09,0x09,0x01},/*G*/ {0x65,0x3E,0x41,0x49,0x49,0x7A},
  /*H*/ {0x65,0x7F,0x08,0x08,0x08,0x7F},/*I*/ {0x43,0x41,0x7F,0x41,0x00,0x00},
  /*J*/ {0x65,0x20,0x40,0x41,0x3F,0x01},/*K*/ {0x65,0x7F,0x08,0x14,0x22,0x41},
  /*L*/ {0x65,0x7F,0x40,0x40,0x40,0x40},/*M*/ {0x65,0x7F,0x02,0x0C,0x02,0x7F},
  /*N*/ {0x65,0x7F,0x04,0x08,0x10,0x7F},/*O*/ {0x65,0x3E,0x41,0x41,0x41,0x3E},

  /*P*/ {0x65,0x7F,0x09,0x09,0x09,0x06},/*Q*/ {0x65,0x3E,0x41,0x51,0x21,0x5E},
  /*R*/ {0x65,0x7F,0x09,0x19,0x29,0x46},/*S*/ {0x65,0x46,0x49,0x49,0x49,0x31},
  /*T*/ {0x65,0x01,0x01,0x7F,0x01,0x01},/*U*/ {0x65,0x3F,0x40,0x40,0x40,0x3F},
  /*V*/ {0x65,0x1F,0x20,0x40,0x20,0x1F},/*W*/ {0x65,0x3F,0x40,0x38,0x40,0x3F},
  /*X*/ {0x65,0x63,0x14,0x08,0x14,0x63},/*Y*/ {0x65,0x07,0x08,0x70,0x08,0x07},
  /*Z*/ {0x65,0x61,0x51,0x49,0x45,0x43},/*[*/ {0x32,0x7F,0x41,0x00,0x00,0x00},
  /*\*/ {0x65,0x02,0x04,0x08,0x10,0x20},/*]*/ {0x32,0x41,0x7F,0x00,0x00,0x00},
  /*^*/ {0x65,0x04,0x02,0x01,0x02,0x04},/*_*/ {0x65,0x40,0x40,0x40,0x40,0x40},

  /*`*/ {0x43,0x01,0x02,0x04,0x00,0x00},/*a*/ {0x65,0x20,0x54,0x54,0x54,0x78},
  /*b*/ {0x65,0x7F,0x48,0x44,0x44,0x38},/*c*/ {0x65,0x38,0x44,0x44,0x44,0x20},
  /*d*/ {0x65,0x38,0x44,0x44,0x48,0x7F},/*e*/ {0x65,0x38,0x54,0x54,0x54,0x18},
  /*f*/ {0x65,0x08,0x7E,0x09,0x01,0x02},/*g*/ {0x65,0x0C,0x52,0x52,0x52,0x3C},
  /*h*/ {0x65,0x7F,0x08,0x04,0x04,0x78},/*i*/ {0x43,0x44,0x7D,0x40,0x00,0x00},
  /*j*/ {0x54,0x20,0x40,0x44,0x3D,0x00},/*k*/ {0x54,0x7F,0x10,0x28,0x44,0x00},
  /*l*/ {0x43,0x41,0x7F,0x40,0x00,0x00},/*m*/ {0x65,0x7C,0x04,0x18,0x04,0x78},
  /*n*/ {0x65,0x7C,0x08,0x04,0x04,0x78},/*o*/ {0x65,0x38,0x44,0x44,0x44,0x38},

  /*p*/ {0x65,0x7C,0x14,0x14,0x14,0x08},/*q*/ {0x65,0x08,0x14,0x14,0x18,0x7C},
  /*r*/ {0x65,0x7C,0x08,0x04,0x04,0x08},/*s*/ {0x65,0x18,0x54,0x54,0x54,0x20},
  /*t*/ {0x65,0x04,0x3F,0x44,0x40,0x20},/*u*/ {0x65,0x3C,0x40,0x40,0x20,0x7C},
  /*v*/ {0x65,0x1C,0x20,0x40,0x20,0x1C},/*w*/ {0x65,0x3C,0x40,0x30,0x40,0x3C},
  /*x*/ {0x65,0x44,0x28,0x10,0x28,0x44},/*y*/ {0x65,0x0C,0x50,0x50,0x50,0x3C},
  /*z*/ {0x65,0x44,0x64,0x54,0x4C,0x44},/*{*/ {0x43,0x08,0x36,0x41,0x00,0x00},
  /*|*/ {0x21,0x7F,0x00,0x00,0x00,0x00},/*}*/ {0x43,0x41,0x36,0x08,0x00,0x00},
  /*~*/ {0x65,0x0C,0x02,0x0C,0x10,0x0C},/*ß*/ {0x65,0x3E,0x15,0x15,0x15,0x0A},

  /*|*/ {0x65,0x7F,0x7F,0x7F,0x7F,0x7F}

};

const PROGMEM uint8_t Standardtext[] = "   ~ Willkommen ~";
// 1. Byte( Bit 0-3: Anzahl der Pixel zur Anzeige; Bit 4-7: Anzahl der Pixel bis zum nächsten Zeichen)
// 2-6. Byte: Pixel von vorn nach hinten

//*********************************<Variable>***********************************
const uint8_t ASCII[97][6]; // 9/x   6  582
struct TBild bild0, bild1;  //  2x 130  260
struct TDisplay display;    //       7    7
struct TText text;          //      80   80
uint16_t   pos_IR;          //       2    2
                            // UART 36   36
                            //          ---
                            // Alles    96/ ==> 58 Bytes für Frame, Stack und Variable

//*********************************<Makros>*************************************
#define Motor(x)  (x ? (PORTB|= 0x01) : (PORTB&= ~0x01))
#define Cursor(x) (x ? (PORTD|= 0x20) : (PORTD&= ~0x20))
#define Leds(x) {PORTC = x & 0x3F; PORTD = (x & 0xC0) | (PORTD & 0x3F);}
#define Bild_Switch() (display.bild ? (display.bild = 0) : (display.bild = 1))

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

/*
  // Timer1 ==> PWM für IR-UART
  TCCR1A = 0x40; // Toggle Output A; CTC bis OCR1A
  TCCR1B = 0x09; // CTC bis OCR1A; Prescaler: 1

  OCR1AH =   0; // 36kHz
  OCR1AL = 111; // 36kHz

  TCNT1H = 0x00; // Timer 1 zurücksetzen
  TCNT1L = 0x00; // Timer 1 zurücksetzen

  TIMSK = 0x00; // Keine Interrupts
 */

  // Timer1 ==> Zeitmessung für Leds
  TCCR1A = 0x40; // Toggle OCR1A; Normal bis 0xFFFF
  TCCR1B = 0x03; // Normal bis 0xFFFF; Prescaler: 64

  TCNT1H = 0x00; // Timer 1 zurücksetzen
  TCNT1L = 0x00; // Timer 1 zurücksetzen

  TIMSK =  0x18; // Interrupts: OCR1A ==> Als PWM für IR-Led; OCR1B ==> Zum Ändern der Leds

  // Timer2 ==> 10 Millisekundentakt
  TCCR2 = 0x0F; // Keine Ausgänge; Top bei OCR2(77); Prescaler: 1024
  TCNT2 = 0x00; // Timer 2 zurücksetzen

  OCR2  = 77;

  TIMSK|= 0x80; // Interrupt für OCR2 ==> 10 Milisekundentakt

  // Externe Interrupts ==> Optokoppler
  MCUCR = 0x02; // Falling Edge für Int0

  GICR  = 0x40; // Int0 einschalten ==> Optokoppler
}

//*********************************[Variable_laden]*****************************
static inline void Variable_laden(){
  uint8_t a,b;
  uint8_t *data_flash;
  uint8_t *data_ram;

  // ASCII - Tabelle
  data_flash = (uint8_t *) ASCII_flash;
  data_ram = (uint8_t *) ASCII;
  a = 0;
  for (a = 0;a < 97; a++) {for (b = 0;b < 6; b++) {
      *data_ram++ = (uint8_t) pgm_read_byte (data_flash++);
  }}

  // Text laden
  data_flash = (uint8_t *) Standardtext;
  data_ram = text.text + 1;
  for (a = 0;a < 63; a++) {
      b = (uint8_t) pgm_read_byte (data_flash++);
      *(data_ram++) = b;
      if (!b) {
        text.text[0] = a;
        break;
      }
  }

}

//*********************************[Init_Variable]******************************
void Init_Variable(){

  pos_IR = 0x0000; // letzter Zeitpunkt fürs Toggeln des IR_PWMs; immer + 7

  display.bild  =     0; // Erstes Bild ist bild0
  display.dauer =     0; // Umlaufzeit zw. 3125 (40 Hz) und 25000 (5 Hz); 0 = keine Drehung messbar; 1..10 = Anzahl der Sinnvollen Messungen
  display.zeit  =     0; // Erster Umlauf bei 0
  display.pos   =     0; // Erste Spalte
  display.sek   =     0; // Noch keine Zählzeit verstrichen

  bild0.cursorA =   150; // Keinen Cursor anzeigen
  bild0.cursorE =   150; // Keinen Cursor anzeigen

  bild1.cursorA =   150; // Keinen Cursor anzeigen
  bild1.cursorE =   150; // Keinen Cursor anzeigen

  text.effekt.speedin  =  5; // 50ms je Spalte
  text.effekt.ein      =  1; // Über rechts einfliegen
  text.effekt.estay    =  1; // Still stehen
  text.effekt.zeit_warten =  20; // 5s Still stehen
  text.effekt.speedout =  5; // 50ms je Spalte
  text.effekt.eout     =  1; // Über links rausfliegen
  text.effekt.pos      =  0; // Text steht vorm Einblenden;alt: Als nächstes kommt das erste Zeichen
  text.effekt.zeit     =  0; // Letzte Änderung bei 0,00 Sekunden

  Variable_laden();
}

//*********************************[ISR(INT0_vect)]*****************************
ISR(INT0_vect) {
  // Optokoppler zur Rundensynkronisation
  uint16_t now;
  uint16_t temp;

  now = TCNT1L & 0xFF;
  now|= TCNT1H << 8;

  temp = now - display.zeit;
  display.zeit = now;

  //Leds(LED++);
  display.pos = 1;

  if ((temp > 3250) && (temp < 25000)) {
    if (display.dauer > 3250) { // eigentlich 6250 - für test runtergesetzt
      if (display.dauer < temp - 100) {display.dauer+= 100;}
      if (display.dauer > temp + 100) {display.dauer-= 100;}
      if (display.dauer < temp -  10) {display.dauer+=  10;}
      if (display.dauer > temp +  10) {display.dauer-=  10;}

      // Für Leds
     if (display.bild) { Leds(bild1.daten[48]);} else { Leds(bild0.daten[48]);}

      temp = display.dauer >> 7;
      temp+= display.zeit;

      OCR1BH = temp >> 8;
      OCR1BL = temp & 0xFF;

      TIFR  |= 0x08; // Interruptflag für OCR1B löschen

    } else {
      if (++display.dauer > 10) {display.dauer = temp;}
    }
  } else {
    Leds(0xF0);
//    display.dauer = 0;
  }

}

//*********************************[ISR(TIMER1_COMPA_vect)]*********************
ISR(TIMER1_COMPA_vect) {
  // PWM Signal für IR-Led
  uint16_t temp;

  temp = TCNT1L & 0xFF;
  temp|= TCNT1H << 8;

  temp+= 7;

  OCR1AH = temp >> 8;
  OCR1AL = temp & 0xFF;
}

//*********************************[ISR(TIMER1_COMPB_vect)]*********************
ISR(TIMER1_COMPB_vect) {
  // Für Leds
  uint16_t temp;
  uint8_t pos;

  display.pos++;

  pos = (48 + display.pos) & 0x7F;
  if (display.bild) { Leds(bild1.daten[pos]);} else { Leds(bild0.daten[pos]);}

  temp = (display.dauer >> 7) * (uint16_t) display.pos + (((display.dauer & 0x7F) * (uint16_t)display.pos) >> 7);
  temp+= display.zeit;

  OCR1BH = temp >> 8;
  OCR1BL = temp & 0xFF;
}

//*********************************[ISR(TIMER2_COMP_vect)]**********************
ISR(TIMER2_COMP_vect){
  if (display.sek) {display.sek--;}
}

//*********************************[Startsequenz]*******************************
void Startsequenz(){
  uint8_t Led = 0x00;
  uint8_t a,b;

  Leds(0x00);

  for (b = 0x80; b; b = b >> 1) {
    for (a = 0; a < 11; a++) {
     Led^= b; Leds(Led); mdelay(100);
    }
  }
  Motor(1);
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

// //*********************************[test_UART]**********************************
// void test_UART(void){
//   uint8_t a = 0;
//   uint8_t b = 0;
//
//   while (1) {
//     serout0(a);
//     mdelay(50);
//     if (serstat0()) {
//       while (serstat0()) {b = serinp0();}
//       if (a == b) {
//         Leds(a);
//         mdelay(50);
//       } else {
//         Cursor(1);
//         Leds(a); mdelay(500); Leds(b); mdelay(500);
//         Leds(a); mdelay(500); Leds(b); mdelay(500);
//         Leds(a); mdelay(500); Leds(b); mdelay(500);
//         Leds(a); mdelay(500); Leds(b); mdelay(500);
//         Leds(a); mdelay(500); Leds(b); mdelay(500);
//         Cursor(0);
//       }
//     } else {
//       Leds(0);
//       Cursor(1); mdelay(100); Cursor(0); mdelay(100);
//       Cursor(1); mdelay(100); Cursor(0); mdelay(100);
//       Cursor(1); mdelay(100); Cursor(0); mdelay(100);
//       Cursor(1); mdelay(100); Cursor(0); mdelay(100);
//       Cursor(1); mdelay(100); Cursor(0); mdelay(100);
//     }
//
//     a++;
//   }
// }

//*********************************[test_Optokoppler]***************************
void test_Optokoppler(void){
  uint8_t a = 0;

  for (a =   0; a <  64; a++  ) {bild0.daten[a] =    a;}
  for (a =  64; a < 128; a+= 2) {bild0.daten[a] = 0xFF;}
  for (a =  65; a < 128; a+= 4) {bild0.daten[a] = 0x0F;}
  for (a =  67; a < 128; a+= 4) {bild0.daten[a] = 0xF0;}

  Motor(1);

  while (1) {mdelay (10);}

}

//*********************************[Bild_Bstabe]********************************
uint8_t Bild_Bstabe(uint8_t Bstabe, int16_t x /*; int8_t y*/) {
  uint8_t b,l_s;
  Bstabe-= 32;
  if (Bstabe > 96) { Bstabe = 96; } // "|"
  l_s = ASCII[Bstabe][0] & 0x0F;

  for (b = 1; b <= l_s; b++) {
    if ((x >= 0) && (x <= 128)) {
      if (display.bild) { bild0.daten[x] = ASCII[Bstabe][b];} else { bild1.daten[x] = ASCII[Bstabe][b];}
    }
    x++;
  }

  return(ASCII[Bstabe][0] >> 4);

}

//*********************************[Bild_Left]**********************************
void Bild_Left(){
  uint8_t b;

  if (display.bild) {
    for (b = 0; b < 127; b++) {
      bild0.daten[b] = bild1.daten[b + 1];
    }
    bild0.daten[127] = bild1.daten[0];
  } else {
    for (b = 0; b < 127; b++) {
      bild1.daten[b] = bild0.daten[b + 1];
    }
    bild1.daten[127] = bild0.daten[0];
  }

}

//*********************************[Bild_Right]*********************************
void Bild_Right() {
  uint8_t b;

  if (display.bild) {
    for (b = 0; b < 127; b++) {
      bild0.daten[b + 1] = bild1.daten[b];
    }
    bild0.daten[0] = bild1.daten[127];
  } else {
    for (b = 0; b < 127; b++) {
      bild1.daten[b + 1] = bild0.daten[b];
    }
    bild1.daten[0] = bild0.daten[127];
  }
}

//*********************************[Bild_Load]**********************************
void Bild_Load() {
  uint8_t a;
  int16_t posb = text.effekt.pos; // Position auf dem display;

  if (display.bild) {
    for (a =   0; a < 128; a++  ) {bild0.daten[a] = 0;} // Bild zurücksetzen
  } else {
    for (a =   0; a < 128; a++  ) {bild1.daten[a] = 0;} // Bild zurücksetzen
  }

  for (a =   1; a <= text.text[0]; a++) {
    posb+= (uint16_t) Bild_Bstabe(text.text[a],posb);
  }

}

//*********************************[Bild_Effekt]********************************
void Bild_Effekt() {
  uint8_t warte;
  while (1) {
    // Einblenden
    text.effekt.pos = 128;
    do {
      display.sek = text.effekt.speedin;
      Bild_Load();
      while(display.sek) { mdelay(1);}
      Bild_Switch();
    } while (--text.effekt.pos > 0);

    // Links drehen
    for (warte = 0; warte < 255; warte++) {
      display.sek = text.effekt.speedin;
      Bild_Left();
      while (display.sek) {mdelay(1);}
      Bild_Switch();
    }

    // Warten
    warte = text.effekt.zeit_warten;
    while (warte--) {
      display.sek = 10;
      while (display.sek) {mdelay(1);}
    }

    // Rechts drehen
    for (warte = 0; warte < 255; warte++) {
      display.sek = text.effekt.speedout;
      Bild_Right();
      while (display.sek) {mdelay(1);}
      Bild_Switch();
    }

    // Ausblenden
    text.effekt.pos = 0;
    do {
      display.sek = text.effekt.speedout;
      Bild_Load();
      while(display.sek) { mdelay(1);}
      Bild_Switch();
    } while (++text.effekt.pos < 128);
  }

}
//*********************************[Main]***************************************
int main (void) {
  Init_System();
  Init_Hardware();
  Init_Variable();
  sei();

  Startsequenz();
  //Blinken();
  //test_Optokoppler();

  Bild_Effekt();

  return (0);
}
