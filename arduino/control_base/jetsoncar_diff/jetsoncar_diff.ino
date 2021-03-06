/*
  Arduino ROS node for JetsonCar project
  The Arduino controls a TRAXXAS Rally Car
  MIT License
  JetsonHacks (2016)
  differential drive robot code based on https://github.com/Reinbert/ros_diffdrive_robot/blob/master/ros_diffdrive_robot.ino
  wheel intensity calculation
  taken from https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010#1e59
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include <WProgram.h>
#endif

#include <stdlib.h>
//#include <stdbool.h>
#include <math.h>

#define USB_USBCON
#include <ros.h>

#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;

//pin for DC motor
//DIR = Direction
//PWM = velocity
//A is left, B is right
//f for front and b for back

const int DIRA_f = 4;
const int PWMA_f = 8;
const int DIRA_b = 6;
const int PWMA_b = 10;


const int DIRB_f = 7;
const int PWMB_f = 9;
const int DIRB_b = 13;
const int PWMB_b = 11;

int usonic_stop = 0;
int usonic_0 = 1;
int usonic_1 = 1;

geometry_msgs::Twist zeroTwist;

//pwm constant initiate
const int MIN_PWM = 37;
const int MAX_PWM = 125;

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

float Pmap(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}

void pwm_log(int l, int r, float fl, float fr, float x, float z){
  char str1[50], str2[50];
  char tmp1[20], tmp2[20], tmp3[20], tmp4[20];
  dtostrf(fl, 8, 3, tmp1);
  dtostrf(fr, 8, 3, tmp2);
  dtostrf(x, 8, 4, tmp3);
  dtostrf(z, 8, 4, tmp4);
  //sprintf(str1, "PWM: %d %d    lr: %s %s\n", l, r, tmp1, tmp2);
  sprintf(str2, "twist(x, z): %s %s\n", tmp3, tmp4);
  //nh.loginfo(str1);
  nh.loginfo(str2);
}

void driveCallback(const geometry_msgs::Twist &twistMsg)
{
  // Cap values at [-1 .. 1]
  
  float x = max(min(twistMsg.linear.x, 1.0f), -1.0f);
  float z = max(min(twistMsg.angular.z, 1.0f), -1.0f);
  
  
  x = ceil(10000*x)/10000;
  z = ceil(10000*z)/10000;
  
  // Calculate the intensity of left and right wheels. Simple version.
  /*
  float l = (twistMsg.linear.x - twistMsg.angular.z) / 2;
  float r = (twistMsg.linear.x + twistMsg.angular.z) / 2;
  */
  float l = (x - z) / 2;
  float r = (x + z) / 2;
  
  // Then map those values to PWM intensities. MAX_PWM = full speed, while MIN_PWM = the minimal amount of power at which the motors begin moving.
  uint16_t lPwm = Pmap(fabs(l), MIN_PWM, MAX_PWM);
  uint16_t rPwm = Pmap(fabs(r), MIN_PWM, MAX_PWM);

  // Set direction pins and PWM
  
  if (l==0 && r==0){
    defaultDrive();
  }
  else if ((l<0 && r<0) && x<0){
    digitalWrite(DIRA_f, l<0);
    digitalWrite(DIRA_b, l<0);
    digitalWrite(DIRB_f, r>0);
    digitalWrite(DIRB_b, r>0);
    analogWrite(PWMA_f, rPwm);
    analogWrite(PWMA_b, rPwm);
    analogWrite(PWMB_f, lPwm);
    analogWrite(PWMB_b, lPwm); 
    }
   else if (!(l*r>0) && x<0){
    digitalWrite(DIRA_f, l>0);
    digitalWrite(DIRA_b, l>0);
    digitalWrite(DIRB_f, r<0);
    digitalWrite(DIRB_b, r<0);
    analogWrite(PWMA_f, rPwm);
    analogWrite(PWMA_b, rPwm);
    analogWrite(PWMB_f, lPwm);
    analogWrite(PWMB_b, lPwm); 
    }
  else{
    digitalWrite(DIRA_f, l<0);
    digitalWrite(DIRA_b, l<0);
    digitalWrite(DIRB_f, r>0);
    digitalWrite(DIRB_b, r>0);
    analogWrite(PWMA_f, lPwm);
    analogWrite(PWMA_b, lPwm);
    analogWrite(PWMB_f, rPwm);
    analogWrite(PWMB_b, rPwm); 
    } 
    
  
  //pwm debugging code
  //pwm_log(int(lPwm), int(rPwm), l, r, x, z);
       
}

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
  //drive setup
  pinMode(13, OUTPUT);
  Serial.begin(57600) ;
  nh.initNode();
  // Subscribe to the throttle messages
  nh.subscribe(driveSubscriber);
  nh.subscribe(usonicSubscriber_0);
  nh.subscribe(usonicSubscriber_1);
  defaultDrive();
  //wheel encoder setup
  
  delay(100) ;

}

void loop() {
  nh.spinOnce();
  delay(1);
}
