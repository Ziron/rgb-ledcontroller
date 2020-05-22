#include <SoftwareSerial.h>

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

SoftwareSerial espSerial(A0, A1); // RX, TX

const int PWM_CYCLE = 256;

int pwm_i = 0;

volatile char B_STATE[PWM_CYCLE];
volatile char C_STATE[PWM_CYCLE];
volatile char D_STATE[PWM_CYCLE];

const char B_MASK = 0b00011111;
const char C_MASK = 0b00111100;
const char D_MASK = 0b11111100;

ISR(TIMER2_COMPA_vect){
   //interrupt commands here

   PORTB = (PORTB & ~B_MASK) | B_STATE[pwm_i];
   PORTC = (PORTC & ~C_MASK) | C_STATE[pwm_i];
   PORTD = (PORTD & ~D_MASK) | D_STATE[pwm_i];

   if (++pwm_i >= PWM_CYCLE) {
      pwm_i = 0;
   }
}

int hexToInt(char hexChar) {
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


  //set timer2 interrupt at (115*256)Hz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for (115*256)Hz increments
  OCR2A = 67;// = (16*10^6) / ((115*256)*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS21);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
  
  sei();//allow interrupts
  
  espSerial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  if (espSerial.available()) {
    char c = espSerial.read();
  }

}
