#include <LiquidCrystal.h>
#include <dht.h>
#include <Servo.h>
#include "RTClib.h"
Servo myservo;

RTC_DS1307 RTC;

//motor
//digital pin 8 is direction (go high to turn on)
//digital pin 13 is enable (go high to enable)

// temp/humidity input on digital pin #9
#define DHT_PIN 9

// for led screen;
// K to GND
// A to +5V thru 220ohms
// d4-d7 digital pins 5-2 respectively
// d0-d3 unconnected
// E to pin #52
// RW to GND
// RS to pin #53
// v0 to potentiometer output, +5v and gnd to inputs
// vdd to +5v
// vss to gnd
const int rs = 53, en = 52, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//registers
// PORT B:
// arduino pin#13, bit#7: water sensor vcc
// arduino pin#12, bit#6: red    LED
// arduino pin#11, bit#5: yellow LED
// arduino pin#10, bit#4, green  LED
// arduino pin#50, bit#3, blue   LED
// arduino pin#51, bit#2, temp/humidity vcc
// arduino pin#52, bit#1, LED screen enable
// arduino pin#53, bit#0, LED screen RS
volatile unsigned char* pin_b = (unsigned char*) 0x23;
volatile unsigned char* ddr_b = (unsigned char*) 0x24;
volatile unsigned char* port_b = (unsigned char*) 0x25;

// PORT K:
// arduino analog pin A#8, bit#0: toggle button input
// arduino analog pin A#9, bit#1: output fan
volatile unsigned char* pin_k = (unsigned char*) 0x106;
volatile unsigned char* ddr_k = (unsigned char*) 0x107;
volatile unsigned char* port_k = (unsigned char*) 0x108;

//PORT H:
//arduino digital pin 7, bit #4, green LED
volatile unsigned char* port_h = (unsigned char*) 0x102;
volatile unsigned char* ddr_h = (unsigned char*) 0x101;

// water level sensor input on analog in #0
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;
volatile unsigned char* my_DIDR0 = (unsigned char*) 0x7E;
volatile unsigned char* my_DDRF = (unsigned char*) 0x30;

// timer registers (for button debouncing)
// button on analog in #8
volatile unsigned char *myTCCR1A  = (unsigned char*) 0x80;
volatile unsigned char *myTCCR1B  = (unsigned char*) 0x81;
volatile unsigned char *myTCCR1C  = (unsigned char*) 0x82;
volatile unsigned char *myTIMSK1  = (unsigned char*) 0x6F;
volatile unsigned char *myTIFR1   = (unsigned char*) 0x36;
volatile unsigned int  *myTCNT1   = (unsigned int* ) 0x84;
unsigned int currentTicks = 65535;
unsigned int buttonTicks = 4000; // about 250ms with prescaler at 1024
unsigned int timer_running = 0;

//Settings
float tempSet = 26; // temp threshold
float minLevel = 150; // minimum water level
int angleSet = 0;
int potpin = 0;
int val;

// Variables in system
float temp = 0, humidity = 0;
int lastState = -1, state = 0, waterLevel = 0, vAngle;
bool enabled = false, clockTick = false;
dht DHT;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  RTC.begin();

  if (! RTC.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }
  
  myservo.attach(10);
  
  // Pb2-7 outputs
  *ddr_b |= 0b11111100;

  // Pk toggle button input, bit #1 output
  *ddr_k |= 0b00000010;

  //sets PH4 as an output for the green LED
  *ddr_h |= (0x01 << 4); 

  // initialize adc for sensors
  adc_init();

  setup_timer_regs();
  
  lcd.begin(16, 2);

  currentTicks = buttonTicks;
  *myTCCR1B |= 0x05;
  timer_running = 1;

  *port_k |= 0b00000010;

  *ddr_h |= (0x01 << 5); 
  *ddr_b |= (0x01 << 7);

  RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
}
void loop() {
  

  //Setup Led indication.
  LEDS();
  //Vent();

  //Check for state change and save info if changed.
  if(lastState != state){
    Save();
  }
 
  //Idle State
  if(state == 1){
    //Check for state changes due to these conditions
    Temp();
    Display();
    WaterLevel();
    Vent();
  }
  
  //Running State
  else if(state == 2){
    //Check for state changes due to these conditions
    Temp();
    Motor();
    Display();
    WaterLevel();
  }
  
  //Error State
  else if(state == 3){
    // turn off temp sensor on state change to error, as we aren't using it
    if (state != lastState){
      *port_b &= 0b11111011;
    }
    //Check for state changes due to these conditions
    Motor();
    Display();
    WaterLevel();
  }
  
  //Default: Disabled State
  else{
    Motor();
    Display();
  }

  Button();
}

void Temp(){
  // Check temp and change state accordingly
  *port_b |= 0b00000100;
  int chk = DHT.read11(DHT_PIN);
  if(DHT.temperature != -999){ // temp defaults to -999 when it's not ready to be read it seems
    // update temp and humidity variables
    temp = DHT.temperature;
    humidity = DHT.humidity;
    
    // set state if temp is below predefined temp requirement
    if(state == 1){
      if(tempSet < temp){
        state = 2; 
      }
    }
    else{
      if(tempSet >= temp){
        state = 1;
      }
    }
  }
}

void WaterLevel(){
  // turn on water sensor
  *port_b |= 0b10000000;

  // read & set waterlevel
  waterLevel = adc_read(0);

  // change to error state if below water level,
  // otherwise, if in error state and water level > minlevel, change to idle
  if(waterLevel < minLevel){
    state = 3;
  }
  else if(waterLevel >= minLevel && state == 3)
  {
    state = 1;
  }
}

void Motor(){
  //if(lastState != state){
    //Serial.println(state);
  if(state == 3 || state == 0){
    // turn motor off
    //*port_k &= 0b11111101;
   *port_b &= 0b01111111;
   *port_h &= 0b11011111;

  }
  else{
    // turn motor on
    //*port_b |= (0x01 << 7);
    //*port_k |= 0b00000010;
    
    *port_b |= (0x01 << 7);
    *port_h |= (0x01 << 5);
  }
//}
}

void Display(){
  
  if(state == 3){
    // Print Error msg and water level to screen.
    lcd.setCursor(0,0);
    lcd.print("Error:");
    lcd.setCursor(0,1);
    lcd.print("Water too low.");
    
    lcd.print(waterLevel);
    
  }
  else if(state >= 1){
    
    // Print Temp at col 0, row 0.
    lcd.setCursor(0,0);
    lcd.print("Temp: ");
    lcd.print(temp);
    // Print humidity at col 0, row 1.
    lcd.setCursor(0,1);
    lcd.print("Humidity: ");
    lcd.print(humidity);
  }
  else
  {
    // turn off LCD E
    *port_b &= 0b11111101;
  }
}

void LEDS(){
  if(lastState != state){
    *port_b &= 0b10000111;
    *port_h &= 0x00;
    //Serial.println(state);
    if(state == 1){
      // Light the Green LED.
      *port_h |= (0x01 << 4);
    }
    else if(state == 2){
      // Light the Blue LED.
      *port_b |= 0b00001000;
    }
    else if(state == 3){
      // Light the Red LED.
      *port_b |= 0b01000000;
    }
    else{
      // Light the Yellow LED.
      *port_b |= 0b00100000;
    }
  }
}

void Vent(){
  //check for change in address to see if angle has changed vAngle and angleSet might need to be hex values.
  // Set vent angle
 /* if(vAngle != angleSet){
    //Change vent angle somehow.
  }
  angleSet = vAngle;*/
  myservo.attach(10);

  unsigned int angle = adc_read(1);
  //Serial.println(value);
  int position = map(angle, 0, 1023, 0, 180);
  myservo.write(position);// move servos to center position -> 90°
}

bool wasPressed = false;
void Button() // for push button, toggle
{
  if(clockTick && wasPressed)
  {
    if(enabled)
    {
      enabled = false;
      state = 0;
    }
    else
    {
      enabled = true;
      state = 1;
    }
  }
  
  
  // if button is pressed
  if(*pin_k & 0b00000001 == 1)
  { 
  
    wasPressed = true;
    // reset timer so it doesnt bounce back and forth
    currentTicks = buttonTicks;
    *myTCNT1 = (unsigned int) (65536 - currentTicks);
  }
  // else if button is not pressed
  else if (clockTick)
  {
    wasPressed = false;
  }
  clockTick = false;
}

void Save(){
  if(lastState != state)
  {
    lcd.clear();
  if(state == 2)
    {
      Serial.print("Motor on at: ");
      writeTime();
    }
    else if (lastState == 2)
    {
      Serial.print("Motor off at: ");
      writeTime();
    }
  }
  // Save information on the state change with file writing.
  lastState = state;
  
}


void adc_init()
{
   // setup the A register
  *my_ADCSRA |= 0x85;
  *my_ADCSRA &= 0x87;
  // setup the B register
  *my_ADCSRB &= (0x01 << 6);
  // setup the MUX Register
  //*my_ADMUX |= (0x01 << 7); //sets MSB bit to 1, REFS1
  *my_ADMUX &= 0x9C; //sets REFS0 and ADLAR for right justification
  //disabling digital input for all pins
  *my_DIDR0 |= 0x81; //disables all digital input pins for buffer
}

unsigned int adc_read(unsigned char adc_channel_num)
{  
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX &= 0xE0;

  // clear the channel selection bits (MUX 5) //MUX 5 is in the ADCSRB register
  *my_ADCSRB &= 0x77;
  
 if (adc_channel_num == 0)
  {
  // set the channel selection bits
  *my_ADMUX = *my_ADMUX | 0b01000000;
  }
  else if (adc_channel_num == 1)
  {
    *my_ADMUX = *my_ADMUX | 0b01000001;
  }
  
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;

  // wait for the conversion to complete
  while (( * my_ADCSRA & 0x40) != 0); 

  // return the result in the ADC data register
  return *my_ADC_DATA; 
}

// Timer setup function
void setup_timer_regs()
{
  // setup the timer control registers
  *myTCCR1A= 0x00;
  *myTCCR1B= 0X00;
  *myTCCR1C= 0x00;
  // reset the TOV flag
  *myTIFR1 |= 0x01;
  // enable the TOV interrupt
  *myTIMSK1 |= 0x01;
}

// TIMER OVERFLOW ISR
ISR(TIMER1_OVF_vect)
{

  // Stop the Timer
  *myTCCR1B &= 0xF8;
  
  // Load the Count
  *myTCNT1 = (unsigned int) (65536 - currentTicks);

  // Start the Timer
  *myTCCR1B |= 0x05;

  // if it's not the STOP amount
  if(currentTicks != 65535)
  {
    // XOR to toggle PB6
    clockTick = true;
  }
}

void writeTime()
{
  DateTime now = RTC.now();
    
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(' ');
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
}
