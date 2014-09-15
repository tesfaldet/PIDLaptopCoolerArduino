int pwmFETPin = 5;
int pwmFanPin = 11;
volatile int counter = 0;
volatile int result = 0;

void setup() {
  cli();  // clear interrupts (no interrupts allowed)
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 15624; //(16*10^6) / (1024 * 1) -1
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12) | (1 >> CS10);
  TIMSK1 |= (1 << OCIE1A);
  sei(); // set interrupts (interrupts allowed)
  pinMode(pwmFETPin, OUTPUT);
  pinMode(pwmFanPin, OUTPUT);
  attachInterrupt(0, rpm, RISING);
  Serial.begin(9600);
}

ISR(TIMER1_COMPA_vect) {
  result = (counter / 2) * 60;
  counter = 0;
}

void rpm() {
  counter++;
}

int x = 0;

void loop() {
  analogWrite(pwmFETPin, 255);  // controls MOSFET gate
  if (x < 1000) {
    analogWrite(pwmFanPin, 55);  // controls fan PWM input
  } else {
    analogWrite(pwmFanPin, 255);
  }
  Serial.print(x);
  Serial.print('_');
  Serial.println(result);
  x++;
}

