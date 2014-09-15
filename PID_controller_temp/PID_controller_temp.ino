/*

 This sketch uses a PID controller to control the PC temperature and displays the result on an LCD.
 Setpoint for temperature is read from a potentiometer. Data is also sent through serial to PC 
 running a real-time graph under Processing software.
 
 */

#include <LiquidCrystal.h>
#include <LM35.h>
#include <PID_v1.h>

// If this is defined it prints out the data that should be sent to the PC
// via Processing software
//#define CHECK_DATA

#define CRITICAL 30.00 // define critical temperature (degrees celsius)

/* CONTROL DEVICES AND SENSORS */
int tachPin = 8;   // fan tachometer connected to digital pin 8
int fanPin = 11;  // fan drive connected to digital pin 11
int potPin = 0; // analog input 0 for the potentiometer
int FETPin = 9; // FET conected to digital pin 9 
                // (MOSFET IS OPTIONAL IF WANT TO CONTROL FAN SPEED VIA PWM ON POWER SOURCE)
int tempPin = 5; // temp sensor connected to analog pin 5
int redPin = 13; // red anode for RGB LED
int grnPin = 12; // green anode
int bluPin = 10; // blue anode
LM35Sensor sensor;

/* MISC. VARS */
int potValue;  // variable to store the value coming from the pot (for setting wanted temperature)
unsigned long duration; // time duration between tachometer pulses
double setpoint; // setpoint in degrees celsius
double temperature; // current temp speed in degrees celsius
float rpm; // fan speed in RPM
boolean beginning = true; // to kickstart the fan at the start of the process

/* PID CONTROL */
double aggKp=40, aggKi=2, aggKd=10; // aggressive tuning
double consKp=20, consKi=1, consKd=5;  // conservative tuning
double out; // PID controller output (0 - 255)
double error;
PID myPID(&temperature, &out, &setpoint, consKp, consKi, consKd, REVERSE); // Initialize PID
boolean auto_manual=1; // setpoint auto or from manually from pot (1 - manual, 0 - auto)
int auto_setpoint=20;
int sampletime=40; // Number of sample to average-out sampling values
unsigned long val; // Working variable to hold the accumulated value

/* vars to send to PC Processing software graph */
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
  pinMode(fanPin, OUTPUT);
  pinMode(FETPin, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(grnPin, OUTPUT);
  pinMode(bluPin, OUTPUT);
  myPID.SetMode(AUTOMATIC);
  Serial.begin(9600); // Serial port baud rate
}

void loop() {

  unsigned int startTag = 0xDEAD;  // Analog port maxes at 1023 so this is a safe termination value
  // for sending values to PC.

  val=0;

  if (auto_manual) {
    for (int i=0;i<sampletime;i++) {
      potValue = analogRead(potPin); // Read the value from the pot. AnalogRead values go from 0 to 1023.
      val=val+potValue;
    }
    potValue=val/sampletime;  // averaging out pot value over sample time

    // convert to percentage (0% - 30% translates to 0 - 30 degrees celsius)
    setpoint=float(potValue)*30/1024;
  } 
  else {
    setpoint=auto_setpoint; // in degrees celsius
  }

  duration=0;

  // skip at beginning since fan isn't spinning, which will take pulseIn 1 second to 
  // timeout each time (no tach signal yet)
  if (!beginning) {
    for (int i = 0; i < sampletime; i++) { // Average out pulseIn time reading in microseconds
      duration = duration+pulseIn(tachPin, HIGH);
    }
  }
  beginning = false;

  rpm=600000000/float(duration); // in RPM
  sensor.read(tempPin); // reading from the temperature sensor
  temperature = sensor.getCelsius(); // getting the temperature in degrees celsius
  error=abs(setpoint-temperature); // distance away from setpoint 

  // compute PID value
  if (error<1) {
    // close to setpoint, be conservative
    myPID.SetTunings(consKp, consKi, consKd);
  } 
  else {
    // far from setpoint, be aggressive
    myPID.SetTunings(aggKp, aggKi, aggKd);
  }
  myPID.Compute();

  if (temperature < CRITICAL) {
    if (temperature < (int) (setpoint - 2)) { // well below setpoint temperature, shine blue 
      digitalWrite(redPin, HIGH);
      digitalWrite(grnPin, HIGH);
      digitalWrite(bluPin, LOW);
    } 
    // optimal zone, shine green
    else if (temperature >= (int) (setpoint - 2) && temperature <= (int) (setpoint + 2)) { 
      digitalWrite(redPin, HIGH);
      digitalWrite(grnPin, LOW);
      digitalWrite(bluPin, HIGH);
    } 
    else {  // near critical zone, shine red
      digitalWrite(redPin, LOW);
      digitalWrite(grnPin, HIGH);
      digitalWrite(bluPin, HIGH);
    }
    analogWrite(fanPin, out);  // analogWrite values from 0 to 255, PWM output
  } 
  else { // if critical, fan on full blast and flash LED blinking red
    int pinSetVal = LOW;
    for (int i = 0; i < 5; i++) {
      digitalWrite(redPin, pinSetVal);
      digitalWrite(grnPin, HIGH);
      digitalWrite(bluPin, HIGH);
      if (pinSetVal == LOW) pinSetVal = HIGH;
      else pinSetVal = LOW;
      delay(100);
    }
    out = 255;
    analogWrite(fanPin, out);
  }
  analogWrite(FETPin, 255); // keep MOSFET gate closed

  lcd.setCursor(0, 0); // Set for first line
  lcd.print(int(rpm)); // displays RPM value
  lcd.print(" RPM CO:");
  lcd.print((int)out);// displays Controller Output value
  lcd.print("   ");

  lcd.setCursor(0, 1); //set for second line
  lcd.print("SP:");
  lcd.print((int)setpoint);// displays setpoint value
  lcd.print("C ");
  lcd.print("Tmp:");// displays current temperature
  lcd.print((int)temperature);
  lcd.print("C   ");     

  if (auto_manual) { // if set-point in Auto, display "A"
    lcd.setCursor(15, 1);
    lcd.print("M");
  } 
  else {
    lcd.setCursor(15, 1);// else display "M" for Manual
    lcd.print("A");
  }


  // scaling for graph on PC
  an0=setpoint*9+offset;
  an1=temperature*9+offset; 
  an2=out+offset;

  Serial.write((unsigned byte*)&startTag, 2); // Send serial data to PC
  Serial.write((unsigned byte*)&an0, 2);
  Serial.write((unsigned byte*)&an1, 2);
  Serial.write((unsigned byte*)&an2, 2);
  Serial.write((unsigned byte*)&an3, 2);
  Serial.write((unsigned byte*)&an4, 2);
  Serial.write((unsigned byte*)&an5, 2);

  // for testing purposes
#ifdef CHECK_DATA  
  Serial.print(" - RPM: ");
  Serial.println(rpm);

  Serial.print(" - setpoint: ");
  Serial.println(setpoint);

  Serial.print(" - output: ");
  Serial.println(out);

  Serial.println("");
#endif
}



