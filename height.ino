/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob
*/

#include <Servo.h>
#include <PID_v1.h>
#include <Wire.h>
#include <LIDARLite.h>

//IMU libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();


imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

LIDARLite lidarLite;
int cal_cnt = 0;
int a[6],j=0,l=0;
float avg;
float curr_height;
float height_err;
float height_thresh = 2;

Servo myservo;  // create servo object to control a servo

//int potpin = 0;  // analog pin used to connect the potentiometer
//int val;    // variable to read the value from the analog pin
int swi=7;
int THpin=5;
double Kp=1,Ki=1,Kd=1;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

unsigned long duration,PWM,throtle,SIG;
unsigned long huv_throtle = 1465;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,Kp,Ki,Kd,DIRECT);

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600);
  pinMode(swi,INPUT);
  pinMode(THpin,INPUT);
  lidarLite.begin(0, true);
  lidarLite.configure(0);
  
 
  //initialize IMU
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
   delay(1000);
 
  bno.setExtCrystalUse(true);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  //val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
  //val = map(val, 0, 1023, 700, 2300);// scale it to use it with the servo (value between 0 and 180)
  throtle= pulseIn(THpin,HIGH);
  duration = pulseIn(swi,HIGH);
  //throtle = map(throtle,1000,1900,700,2300);
  //duration = map(duration,1000,2000,700,2300); 
  
  
  
  if (duration<=1300){
      SIG = throtle ;// sets the servo position according to the scaled value
      myservo.writeMicroseconds(PWM);
}
   else if (duration>=1700){
      SIG = PWM;
      myservo.writeMicroseconds(PWM);
   }

Serial.print(duration);
Serial.print("    ");
Serial.print(throtle);
Serial.print("    ");
Serial.print(PWM);
Serial.print("    ");
Serial.println(SIG);
  
  delay(200);// waits for the servo to get there


  curr_height = getheight();
  height_err = Setpoint - curr_height;

  if((height_err > height_thresh)||((height_err)*(-1.0)>height_thresh)){

      if(height_err > 0){
        myPID.SetControllerDirection(DIRECT);
        myPID.Compute();
        PWM = huv_throtle + Output;
      }
      else{
        myPID.SetControllerDirection(REVERSE);
        myPID.Compute();
        PWM = huv_throtle - Output;
      }
  } 
 
   /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}

float getheight()
{
int dist, sum=0;

  // At the beginning of every 100 readings,
  // take a measurement with receiver bias correction
  if ( cal_cnt == 0 ) {
    dist = lidarLite.distance();      // With bias correction
  } else {
    dist = lidarLite.distance(false); // Without bias correction
  }

  // Increment reading counter
  cal_cnt++;
  cal_cnt = cal_cnt % 100;

  for (j=6;j>0;j--){
    a[j]= a[j-1];
  }
  a[0]=dist;

  for(l=0;l<5;l++)
  {
  sum = sum + a[l];
  }
  avg = sum/5;
 
 // correct for tilt
   imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  avg = -avg * gravity.z / 9.8032
  return(avg);
}

 
long SetpointF()
{
  if (throtle > 1500){
    Setpoint = 500;
  }
  else {
    Setpoint = 200;
  }
  return (Setpoint);
}  
