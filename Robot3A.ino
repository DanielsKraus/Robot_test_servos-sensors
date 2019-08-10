/*
Test of robots various systems. Such as servos,compass,distance
sensor and hit sensor.
*/

#include <Servo.h> 
#include <Wire.h>
#include <HMC5883L.h>

Servo servoLeft; // servo left
Servo servoRight; // servo right
HMC5883L compass; // compass

const int trigPin = 7; // pin for trig
const int echoPin = 8; // pin for echo

const int buttonLeft = 2; // button left
const int buttonRight = 3; // button right

const int SDA = 4; // compass SDA
const int SCL = 5; // compass SCL

long duration;
int distance;
int ccw;  // delay is set for degrees going counter clock wise
int cw;   // delay is set for degrees going clock wise
int pos_1 = 0;  // servo position left
int pos_2 = 0;  // servo position right
int buttonState1 = 0;
int buttonState2 = 0;
int objectLeft = 0;  // if hits object on the left changes
int objectRight = 0;  // if hits object on the right changes

///////////////////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);      
  pinMode(buttonLeft, INPUT);
  pinMode(buttonRight, INPUT);
  servoLeft.attach(9);
  servoRight.attach(10);
  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }
  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);
  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);
  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0);
}

//////////////////////////////////////////////////////////////////
void loop()
{
  ping();
  hitSensor1();
  hitSensor2();
  compass1();
  
}

///////////////////////////////////////////////////////////////////
void servo_left()
{
  servoLeft.write(pos_1);
  delay(cw);
}
void servo_right()
{
  servoRight.write(pos_2);
  delay(ccw);
}
void hitSensor1()
{
  buttonState1 = digitalRead(buttonLeft);
  if(buttonState1 == HIGH)
  {
    Serial.println(objectLeft);
    Serial.println("left sensor");
  }
}
void hitSensor2()
{
  buttonState2 = digitalRead(buttonRight);
  if(buttonState2 == HIGH)
  {
    Serial.println(objectRight);
    Serial.println("right sensor");
  }
}

void compass1()
{
  Vector norm = compass.readNormalize();
  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;
  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }
  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }
  // Convert to degrees
  float headingDegrees = heading * 180/M_PI; 

  Serial.print(" Heading = ");
  Serial.print(heading);
  Serial.print(" Degress = ");
  Serial.print(headingDegrees);
  Serial.println();
  delay(100);
}
void ping()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH); 
  distance = duration*0.034/2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);
}
