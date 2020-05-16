
/*
  Arduino ROS node for JetsonCar project
  The Arduino controls a TRAXXAS Rally Car
  MIT License
  JetsonHacks (2016)
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include <WProgram.h>
#endif

#include <stdlib.h>
//#include <stdbool.h>

#include <Servo.h>

#define USB_USBCON
#include <ros.h>

#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;
// These are general bounds for the steering servo and the
// TRAXXAS Electronic Speed Controller (ESC)
const int minSteering = 75 ;
const int maxSteering = 105;
const int minThrottle = 0 ;
const int maxThrottle = 150 ;

//pin for DC motor
//DIR = Direction
//PWM = velocity
//A is left, B is right
//f for front and b for back
const int DIRA_f = 4;
const int PWMA_f = 3;

const int DIRB_f = 2;
const int PWMB_f = 5;

const int DIRA_b = 7;
const int PWMA_b = 6;

const int DIRB_b = 13;
const int PWMB_b = 11;

Servo steeringServo;

int usonic_stop = 0;
int usonic_0 = 1;
int usonic_1 = 1;

geometry_msgs::Twist zeroTwist;


std_msgs::Int32 str_msg;
ros::Publisher chatter("chatter", &str_msg);

// Arduino 'map' funtion for floating point
double fmap (double toMap, double in_min, double in_max, double out_min, double out_max) {
  return (toMap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void driveCallback ( const geometry_msgs::Twist&  twistMsg);

void defaultDrive()
{
  pinMode(DIRA_f, OUTPUT);
  pinMode(DIRB_f, OUTPUT);
  pinMode(DIRA_b, OUTPUT);
  pinMode(DIRB_b, OUTPUT);
 
  //Initiate DC motor
  //lft wheel initiate
  
  digitalWrite(DIRA_f, LOW);
  digitalWrite(DIRA_b, LOW);
  //right wheel initiate
  
  digitalWrite(DIRB_f, HIGH);
  digitalWrite(DIRB_b, HIGH);
  
  analogWrite(PWMA_f, 0);
  analogWrite(PWMB_f, 0);
  analogWrite(PWMA_b, 0);
  analogWrite(PWMB_b, 0);

  return;
}

void driveWrite(int _escCommand, int _direction){
  //_direction is fwd if 1, bwd if -1, 0 is to stop
  if (_direction==1){
    //left forward
    digitalWrite(DIRA_f, LOW);
    analogWrite(PWMA_f, _escCommand);
    digitalWrite(DIRA_b, LOW);
    analogWrite(PWMA_b, _escCommand);
    //right forward
    digitalWrite(DIRB_f, HIGH);
    analogWrite(PWMB_f, _escCommand);
    digitalWrite(DIRB_b, HIGH);
    analogWrite(PWMB_b, _escCommand);
    }
  else if (_direction==-1){
    //left forward
    digitalWrite(DIRA_f, HIGH);
    analogWrite(PWMA_f, -_escCommand);
    digitalWrite(DIRA_b, HIGH);
    analogWrite(PWMA_b, -_escCommand);
    //right backward
    digitalWrite(DIRB_f, LOW);
    analogWrite(PWMB_f, -_escCommand);
    digitalWrite(DIRB_b, LOW);
    analogWrite(PWMB_b, -_escCommand);
  }
  else{
    //stop
    digitalWrite(DIRA_f, HIGH);
    analogWrite(PWMA_f, 0);
    digitalWrite(DIRA_b, HIGH);
    analogWrite(PWMA_b, 0);
    digitalWrite(DIRB_f, LOW);
    analogWrite(PWMB_f, 0);
    digitalWrite(DIRB_b, LOW);
    analogWrite(PWMB_b, 0);
  }
  
}

int ustop(int us_0, int us_1){
  int us_stop = (us_0 || us_1);
  if (us_stop){
    driveCallback(zeroTwist);
  }
  else {
    //defaultDrive();
    //driveCallback();
    
  }
  return us_stop;
}


void driveCallback ( const geometry_msgs::Twist&  twistMsg)
{
  //usonic_stop = ustop(usonic_0, usonic_1);
  
  //60 is supposed to be the mid point
  int steeringMid = 75;
  int steeringAngle = fmap(-twistMsg.angular.z, 0.0, 1.0, minSteering, maxSteering) ;
  // The following could be useful for debugging
  //str_msg.data = steeringAngle ;
  //chatter.publish(&str_msg);
  // Check to make sure steeringAngle is within car range
  if (steeringAngle > steeringMid) {
    steeringAngle = steeringAngle;
  }
  if (steeringAngle < steeringMid) {
    steeringAngle = steeringAngle;
  }

  steeringServo.write(steeringAngle);
  
  str_msg.data = steeringAngle;
  chatter.publish(&str_msg);
  // ESC forward is between 0.5 and 1.0
  int escCommand ;

  if(usonic_0 <= 0 || usonic_1 <= 0){
    usonic_stop = 1;
    twistMsg = zeroTwist;
  }
  else{
    usonic_stop = 0;
  }

  if(!usonic_stop){
    escCommand = 100 * (twistMsg.linear.x);
  }
  
  
  if (usonic_stop){
    escCommand=0;
    //stop when usonic signal is below threshold
    driveWrite(escCommand, 0);
   
  }
  else if (escCommand > 0 && !usonic_stop) {
    //lft forward
    driveWrite(escCommand, 1);
  }
  else if (escCommand < 0 && !usonic_stop) {
    //lft backward
    driveWrite(escCommand, -1);
  }
  
  else {
    driveWrite(escCommand, 0);
  }
  
  
}

void stopCallback_0 (const std_msgs:: Int32 _usonic_signal){
  int usonic_signal = _usonic_signal.data;
  
  if (usonic_signal < 500){
    //usonic_0--;
    usonic_0 = 1;
    ustop(usonic_0,usonic_1);
    
    //driveCallback(zeroTwist);
  }
  else {
    //usonic_0 = 1;
    usonic_0 = 0;
    ustop(usonic_0,usonic_1);
    //defaultDrive();
  }
  
 }


void stopCallback_1 (const std_msgs:: Int32 _usonic_signal){
  int usonic_signal = _usonic_signal.data;
  
  if (usonic_signal < 500){
    //usonic_1--;
    usonic_1 = 1;
    ustop(usonic_0,usonic_1);
    //driveCallback(zeroTwist);
  }
  else {
    //usonic_1 = 1;
    usonic_1 = 0;
    ustop(usonic_0,usonic_1);
    //defaultDrive();
  }
  
 }


ros::Subscriber<geometry_msgs::Twist> driveSubscriber("/cmd_vel", &driveCallback) ;
ros::Subscriber<std_msgs:: Int32> usonicSubscriber_0("usonic_data_0", &stopCallback_0);
ros::Subscriber<std_msgs:: Int32> usonicSubscriber_1("usonic_data_1", &stopCallback_1);
//ros::Subscriber<std_msgs:: Int32> usonicSubscriber_2("usonic_data_2", &stopCallback_1);
// ros::Subscirber<std_msgs:: String> usonicSubscriber_3("usonic_data_3", &stopCallback);

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(57600) ;
  nh.initNode();
  // This can be useful for debugging purposes
  nh.advertise(chatter);
  // Subscribe to the steering and throttle messages


  nh.subscribe(driveSubscriber);
  nh.subscribe(usonicSubscriber_0);
  nh.subscribe(usonicSubscriber_1);

  // Attach the servos to actual pins
  steeringServo.attach(9); // Steering servo is attached to pin 9

  
  // Initialize Steering and ESC setting
  // Steering centered is 90, throttle at neutral is 90
  // default dc motor is 0
  steeringServo.write(90) ;
  defaultDrive();
  delay(100) ;

}

void loop() {
  nh.spinOnce();
  delay(1);
}
