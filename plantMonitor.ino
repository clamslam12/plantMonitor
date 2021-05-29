
#include <avr/io.h>
#include <avr/interrupt.h>

//Using Overflow Timer1 Interrupt; max prescaler is 1024 so max time is 4.2 secs
//InterruptTime= (1/clockspeed) x(PrescalerValue)x(OverflowValue)
//for a timer1 to interrupt of about every 4.2 second using overflow
//For timer1 (16 bits, max count 65535), the prescaler = (16x10^6 * 1 second) / (2^16) = 244.14; ~=256
int led13 = 13;
int led12 = 12;
int button2 = 2;
//button/led for fan direction
int button4 = 4;
int led7 = 7;
// variables will change:
int ledState = LOW;     // the current state of LED
int lastButtonState;    // the previous state of button
int currentButtonState; // the current state of button

//potentiometer
//duty cycle in arduino is represented by a 8 bit value: 0 - 255
//0 --> 0%
//127 -->50%
//255 --> 100%
#define pot A0

//l293d and dc motor
int enA = 11;
int in1 = 10;
int in2 = 9;

//thermistor
int ThermistorPin = A1;
int Vo;
float R1 = 10000;
float logR2, R2, T;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

//active buzzer
int buzzer = 8;//the pin of the active buzzer


void setup() {
  // put your setup code here, to run once:
  //init LEDs as output and turn them off
  pinMode(led13, OUTPUT);
  pinMode(led12, OUTPUT);
  pinMode(led7, OUTPUT);
  //enable fan direction button 
  pinMode(button4, INPUT_PULLUP);
  currentButtonState = digitalRead(button4);
  digitalWrite(led13, LOW);
  digitalWrite(led12, LOW);
  pinMode(buzzer,OUTPUT);//initialize the buzzer pin as an output
  //enable pull-up resistor for overflow timer1 interrupt button
  digitalWrite(button2, HIGH);
  //potentiometer input
  pinMode(pot,INPUT);
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  Serial.begin(9600);

  //using timer1 overflow interrupt
  cli(); //noInterrupts(); // disable system interrupt
  //initialize timer1 (16 bits, max count 65535); set timer control register to 0
  TCCR1A = 0;
  TCCR1B = 0;
  //using 1024 prescaler
  TCCR1B |= 0b00000101;
  //enable timer1 overflow interrupt; 
  TIMSK1 |= 0b00000001;
  
  //external interrupt
  //Enable INT0(pin2) and INT1(pin3) (EIMSK)
  EIMSK |= (1 << INT0);
//  EIMSK |= (1 << INT1);

  //set edge to low
  // Trigger INT0 on low edge(00)
   EICRA |= 0b00000000;
   
  sei(); //interrupts(); //set system interrupt
  
}
//ISR routine to execute if interrupt for timer1
ISR (TIMER1_OVF_vect) {
  //Do something
  //delay(1000);  //Will not work
  //read thermistor and convert to temp reading
  digitalWrite(led13, HIGH);
  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  T = T - 273.15;
  T = (T * 9.0)/ 5.0 + 32.0;
  Serial.print("Temperature: "); 
  Serial.print(T);
  Serial.println(" F"); 
  //read photoresistor
  int photoVal = analogRead(A2); 
  Serial.print("Photoresistor value : ");
  Serial.println(photoVal);
  Serial.println("******************************");
  while(T > 80.0 && photoVal > 350){
    
    //turn on buzzer/red led
    digitalWrite(buzzer,HIGH);
    digitalWrite(led13, HIGH);
    //get reading from thermistor and convert to F
    Vo = analogRead(ThermistorPin);
    R2 = R1 * (1023.0 / (float)Vo - 1.0);
    logR2 = log(R2);
    T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
    T = T - 273.15;
    T = (T * 9.0)/ 5.0 + 32.0;
    Serial.print("Temperature: "); 
    Serial.print(T);
    Serial.println(" F"); 
    if(ledState == HIGH){
      //move fan clockwise
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
     }else{
      //move fan counter-clockwise to exhaust
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
     }
    //read potentiometer and adjust speed
    float val = analogRead(pot);
    float duty = map(val,0,1023,0,255);
    analogWrite(enA, duty); //here's how to generate PWM signal from Digital arduino pin
    Serial.print("Fan Speed Duty Cycle: ");
    Serial.println(duty);
    //read photoresistor
    photoVal = analogRead(A2); 
    Serial.print("Photoresistor value : ");
    Serial.println(photoVal);
    Serial.println("******************************");
  }
  // Turn off motors/red led/buzzer/servo
  
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(led13, LOW);
  digitalWrite(buzzer,LOW);
}
//external interrupt
ISR(INT0_vect)
{
  cli();
  while(digitalRead(button2) == LOW){
    //turn on led/buzzer/servo
    
    digitalWrite(led12, HIGH);
    digitalWrite(buzzer,HIGH);
     
     if(ledState == HIGH){
      //move fan clockwise
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
     }else{
      //move fan counter-clockwise to exhaust
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
     }
    
    //read potentiometer and adjust fan speed
    float val = analogRead(pot);
    float duty = map(val,0,1023,0,255);
    //turn on dc motor/fan
    analogWrite(enA, duty); //here's how to generate PWM signal from Digital arduino pin
    Serial.print("Fan Speed Duty Cycle: ");
    Serial.println(duty);
    Serial.println("******************************");
    
  }
  //turn off led/buzzer/servo/dc motor
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(led12, LOW);
  digitalWrite(buzzer,LOW);
  sei();
}
void loop() {
   // put your main code here, to run repeatedly:
  lastButtonState = currentButtonState;      // save the last state
  currentButtonState = digitalRead(button4); // read new state
  if(lastButtonState == HIGH && currentButtonState == LOW) {
    Serial.println("The button is pressed");

    // toggle state of LED
    ledState = !ledState;

    // control LED arccoding to the toggled state
    digitalWrite(led7, ledState); 
  }
}
