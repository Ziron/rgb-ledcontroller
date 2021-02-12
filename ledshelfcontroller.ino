/*
 * ----------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <sebastian@gustavsson.tech> wrote this file. As long as you 
 * retain this notice you can do whatever you want with this stuff. 
 * If we meet some day, and you think this stuff is worth it, 
 * you can buy me a beer in return.            Sebastian Gustavsson
 * ----------------------------------------------------------------
 */

// ESP_TX - A0 - PC0
// ESP_RX - A1 - PC1

// G1 - A2 -- PC2
// R1 - A3 -- PC3
// B1 - A4 -- PC4
// G2 - A5 -- PC5

// R2 - D12 - PB4
// B2 - D11 - PB3
// G3 - D10 - PB2
// R3 - D9 -- PB1
// B3 - D8 -- PB0

// G4 - D7 -- PD7
// R4 - D6 -- PD6
// B4 - D5 -- PD5
// G5 - D4 -- PD4
// R5 - D3 -- PD3
// B5 - D2 -- PD2

// Serial RX/TX is PD0/1

//--------------------
#define G1_ARR C_STATE
#define G1_BIT (1 << 2)

#define R1_ARR C_STATE
#define R1_BIT (1 << 3)

#define B1_ARR C_STATE
#define B1_BIT (1 << 4)
//--------------------
#define G2_ARR C_STATE
#define G2_BIT (1 << 5)

#define R2_ARR B_STATE
#define R2_BIT (1 << 4)

#define B2_ARR B_STATE
#define B2_BIT (1 << 3)
//--------------------
#define G3_ARR B_STATE
#define G3_BIT (1 << 2)

#define R3_ARR B_STATE
#define R3_BIT (1 << 1)

#define B3_ARR B_STATE
#define B3_BIT (1 << 0)
//--------------------
#define G4_ARR D_STATE
#define G4_BIT (1 << 7)

#define R4_ARR D_STATE
#define R4_BIT (1 << 6)

#define B4_ARR D_STATE
#define B4_BIT (1 << 5)
//--------------------
#define G5_ARR D_STATE
#define G5_BIT (1 << 4)

#define R5_ARR D_STATE
#define R5_BIT (1 << 3)

#define B5_ARR D_STATE
#define B5_BIT (1 << 2)
//--------------------

//#define SERIAL_DEBUG
#ifdef SERIAL_DEBUG
  #define espSerial Serial
#else
  #define espSerial Serial
  //#include <SoftwareSerial.h>
  //SoftwareSerial espSerial(A0, A1); // RX, TX
#endif

const uint8_t PWM_CYCLE = 255;

uint8_t pwm_i = 0;

volatile uint8_t B_STATE[PWM_CYCLE];
volatile uint8_t C_STATE[PWM_CYCLE];
volatile uint8_t D_STATE[PWM_CYCLE];

const uint8_t B_MASK = 0b00011111;
const uint8_t C_MASK = 0b00111100;
const uint8_t D_MASK = 0b11111100;

uint8_t R1_val = 0;
uint8_t G1_val = 0;
uint8_t B1_val = 0;

uint8_t R2_val = 0;
uint8_t G2_val = 0;
uint8_t B2_val = 0;

uint8_t R3_val = 0;
uint8_t G3_val = 0;
uint8_t B3_val = 0;

uint8_t R4_val = 0;
uint8_t G4_val = 0;
uint8_t B4_val = 0;

uint8_t R5_val = 0;
uint8_t G5_val = 0;
uint8_t B5_val = 0;

uint8_t cur_val = 0;
uint8_t val_count = 0;
uint8_t char_count = 0;

ISR(TIMER2_COMPA_vect){
   //interrupt commands here

   PORTB = (PORTB & ~B_MASK) | B_STATE[pwm_i];
   PORTC = (PORTC & ~C_MASK) | C_STATE[pwm_i];
   PORTD = (PORTD & ~D_MASK) | D_STATE[pwm_i];

   if (++pwm_i >= PWM_CYCLE) {
      pwm_i = 0;
   }
}

void setVal(volatile uint8_t *arr, const uint8_t valBit, uint8_t &curVal, const uint8_t newVal) {
#ifdef SERIAL_DEBUG
  Serial.print("Change ");
  Serial.print(curVal);
  Serial.print(" to ");
  Serial.println(newVal);
#endif
  
  if (newVal == curVal) {
    return;
  } else if (newVal < curVal) {
    for (int i = curVal - 1; i >= newVal; i--) {
      arr[i] &= ~valBit;
    }
  } else {
    for (int i = curVal; i < newVal; i++) {
      arr[i] |= valBit;
    }
  }
  curVal = newVal;
}

uint8_t hexToInt(const char hexChar) {
  if (hexChar < 'A') {           
    return hexChar - '0';         // is digit
  } else if (hexChar < 'a') {   
    return (hexChar - 'A') + 10;  // is upper case
  } else {                      
    return (hexChar - 'a') + 10;  // is lower case
  }
}

void setup() {
  // put your setup code here, to run once:
  
  cli();//stop interrupts

  // Setup outputs
  DDRB = B_MASK;
  DDRC = C_MASK;
  DDRD = D_MASK;


  //set timer2 interrupt at (120*255)Hz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for (120*255)Hz increments
  OCR2A = 64;// = (16*10^6) / ((120*255)*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS21);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
  
  sei();//allow interrupts
  
  espSerial.begin(115200); // Each update requires (5*3*2+1) = 31 chars * 10 = 310 bits. 19200 baud should be enough for just under 62 fps (60 is goal)
}

void loop() {
  // put your main code here, to run repeatedly:

  if (espSerial.available())
  {
    char c = espSerial.read();

    if (c == ';') { // Use ';' for separating commands
      cur_val = 0;
      val_count = 0;
      char_count = 0;
    } else if (('0' <= c && c <= '9') || ('A' <= c && c <= 'F') || ('a' <= c && c <= 'f')) { // Is hex digit
      cur_val = (cur_val << 4) |  hexToInt(c);

      if (++char_count >= 2) {

        switch(val_count++) {
          case 0: { // R1
            setVal(R1_ARR, R1_BIT, R1_val, cur_val);
            break;
          }
          case 1: { // G1
            setVal(G1_ARR, G1_BIT, G1_val, cur_val);
            break;
          }
          case 2: { // B1
            setVal(B1_ARR, B1_BIT, B1_val, cur_val);
            break;
          }

          case 3: { // R2
            setVal(R2_ARR, R2_BIT, R2_val, cur_val);
            break;
          }
          case 4: { // G2
            setVal(G2_ARR, G2_BIT, G2_val, cur_val);
            break;
          }
          case 5: { // B2
            setVal(B2_ARR, B2_BIT, B2_val, cur_val);
            break;
          }

          case 6: { // R3
            setVal(R3_ARR, R3_BIT, R3_val, cur_val);
            break;
          }
          case 7: { // G3
            setVal(G3_ARR, G3_BIT, G3_val, cur_val);
            break;
          }
          case 8: { // B3
            setVal(B3_ARR, B3_BIT, B3_val, cur_val);
            break;
          }

          case 9: { // R4
            setVal(R4_ARR, R4_BIT, R4_val, cur_val);
            break;
          }
          case 10: { // G4
            setVal(G4_ARR, G4_BIT, G4_val, cur_val);
            break;
          }
          case 11: { // B4
            setVal(B4_ARR, B4_BIT, B4_val, cur_val);
            break;
          }

          case 12: { // R5
            setVal(R5_ARR, R5_BIT, R5_val, cur_val);
            break;
          }
          case 13: { // G5
            setVal(G5_ARR, G5_BIT, G5_val, cur_val);
            break;
          }
          case 14: { // B5
            setVal(B5_ARR, B5_BIT, B5_val, cur_val);
            break;
          }

          default: {
            #ifdef SERIAL_DEBUG
            Serial.println("Got too many values!");
            #endif
            break;
          }
        }
        
        cur_val = 0;
        char_count = 0;
      }
    }
  }
}
