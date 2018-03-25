/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob
*/

#include <Servo.h>
#include <PID_v1.h>

Servo myservo;  // create servo object to control a servo

int potpin = 0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin
int swi=7;
int THpin=5;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

unsigned long duration,PWM,throtle;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,2,5,1,P_ON_M, DIRECT);

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600);
  pinMode(swi,INPUT);
  pinMode(THpin,INPUT);

  Setpoint = 100;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
  val = map(val, 0, 1023, 700, 2300);// scale it to use it with the servo (value between 0 and 180)
  throtle= pulseIn(THpin,HIGH);
  duration = pulseIn(swi,HIGH);
  throtle = map(duration,1000,2000,700,2300);
  duration = map(duration,1000,2000,700,2300); 
  
  
  
  if (duration<=1300){
      PWM = throtle ;// sets the servo position according to the scaled value
      myservo.writeMicroseconds(PWM);
}
   else if (duration>=1700){
      PWM = val;
      myservo.writeMicroseconds(PWM);
   }



   
Serial.print(val);
Serial.print("    ");
Serial.print(duration);
Serial.print("    ");
Serial.print(throtle);
Serial.print("    ");
Serial.println(PWM);
  
  delay(100);                           // waits for the servo to get there
}