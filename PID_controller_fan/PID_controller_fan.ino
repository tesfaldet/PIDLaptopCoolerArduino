/*

 This sketch use PID controller to control PC fan speed and display result on a LCD.
 Setpoint for speed is read from a potentiometer. Data also send thru serial to PC 
 running a real-time graph under Processing software.
 
 Author - Bro_Az August 2010.
 
 The circuit:
 * LCD RS pin to digital pin 7
 * LCD Enable pin to digital pin 6
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * ends to +5V and ground
 * 10k pot wiper to LCD VO pin (pin 3)
 
 */

#include <LiquidCrystal.h>

// If this is defined it prints out the FPS that we can send a
// complete set of data over the serial port.
//#define CHECK_FPS

int tachPin = 8;   // fan tachometer connected to digital pin 8
int fanPin = 11;  // fan drive connected to digital pin 11
int sensorPin = A0; // analog input 0 for the potentiometer
int FETPin = 9;

int sensorValue;  // variable to store the value coming from the sensor
unsigned long duration; // time duration between tachometer pulses
int max_rpm=2000; // max rpm at 100% PWM output
int setpoint; // setpoint in %
int speed; // fan speed in %
float rpm; // fan speed in RPM

int error; 
int last_error=0;
int diff;
float integ=0;
float kp=1.0; // proportional gain
float ki=0.5; // integral gain
float kd=0.05; // derivative gain
int out; // PID controller output

boolean auto_manual=1; // setpoint manual or from pot
int manual_setpoint=77;
int sampletime=40; // Number of sample to average-out sampling values
unsigned long val; // Working variable to hold the accumulated value
boolean beginning = true; // to kickstart the fan at the start of the process

unsigned int an0=0; // Variables for sending data to PC
unsigned int an1=0;
unsigned int an2=0;
unsigned int an3=0;
unsigned int an4=0;
unsigned int an5=0;
unsigned int offset=2;


// initialize the LCD library routine with the numbers of the interface pins
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

void setup() {
  lcd.begin(16, 2); // set up the LCD's number of rows and columns:
  //digitalWrite(tachPin, HIGH);  // turn on pull-up resistor 
  pinMode(fanPin, OUTPUT);
  pinMode(FETPin, OUTPUT);
  Serial.begin(9600); // Serial port baud rate
}

void loop() {

  unsigned int startTag = 0xDEAD;  // Analog port maxes at 1023 so this is a safe termination value
  // for sending values to PC.
#ifdef CHECK_FPS  
  unsigned long startTime, endTime;
  startTime = millis();
#endif

  val=0;

  for (int i=0;i<sampletime;i++) {
    sensorValue = analogRead(sensorPin); // Read the value from the pot. AnalogRead values go from 0 to 1023.
    val=val+sensorValue;
  }
  sensorValue=val/sampletime;

  if (auto_manual) {
    setpoint=float(sensorValue)*100/1024;  // convert to percentage
  }
  else {
    setpoint=manual_setpoint;
  }

  duration=0;
  
  // skip at beginning since fan isn't spinning, which will take pulseIn 1 second to timeout each time (no tach signal yet)
  if (!beginning) {
    for (int i = 0; i < sampletime; i++) { // Average out pulseIn time reading in microseconds
      duration = duration+pulseIn(tachPin, HIGH);
    }
  }
  beginning = false;

  rpm=600000000/float(duration); // in RPM
  speed=rpm/max_rpm*100; // convert to percentage
  error=setpoint-speed;
  diff=error-last_error;
  integ=integ+error;
  out=kp*error+ki*integ+kd*diff;

  if (out<0) {
    out=0;
    integ=0;
  } //min limit, anti-reset wind-up
  if (out>255) {
    out=255;
    integ=integ-error;
  } //max limit, anti-reset wind-up
  
  analogWrite(fanPin, out);  // analogWrite values from 0 to 255, PWM output
  analogWrite(FETPin, 255); // keep MOSFET gate closed

  
  lcd.setCursor(0, 0); // Set for first line
  lcd.print(int(rpm)); // displays RPM value
  lcd.print(" RPM CO=");
  lcd.print(out);// displays Controller Output value
  lcd.print("   ");

  lcd.setCursor(0, 1); //set for second line
  lcd.print("SP=");
  lcd.print(setpoint);// displays setpoint value
  lcd.print("% ");
  lcd.print("PV=");// displays PV value
  lcd.print(speed);
  lcd.print("%   ");     

  if (auto_manual) { // if set-point in Auto, display "A"
    lcd.setCursor(15, 1);
    lcd.print("A");
  }
  else {
    lcd.setCursor(15, 1);// else display "M" for Manual
    lcd.print("M");
  }
  
  
  // scaling for graph on PC
  an0=setpoint*9+offset; // (0% - 99%) * 9 + offset
  an1=speed*9+offset; // (0% - 99%) * 9 + offset
  an2=out*3+offset; // (0 - 255) * 3 + offset

  Serial.write((unsigned byte*)&startTag, 2); // Send serial data to PC
  Serial.write((unsigned byte*)&an0, 2);
  Serial.write((unsigned byte*)&an1, 2);
  Serial.write((unsigned byte*)&an2, 2);
  Serial.write((unsigned byte*)&an3, 2);
  Serial.write((unsigned byte*)&an4, 2);
  Serial.write((unsigned byte*)&an5, 2);
  

#ifdef CHECK_FPS  
  endTime = millis();
  
  //Serial.print(" - FPS: ");
  //Serial.println(1.f / (endTime-startTime) * 1000);
  
  Serial.print(" - RPM: ");
  Serial.println(rpm);
  
  Serial.print(" - setpoint: ");
  Serial.println(setpoint);
  
  Serial.print(" - output: ");
  Serial.println(out);
  
  Serial.println("");
#endif
}

