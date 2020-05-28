#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include <WProgram.h>
#endif

#include "diff_control.h"
#include "wheel_encoder.h"
#include <stdlib.h>

#define USB_USBCON
#include <ros.h>

ros::NodeHandle  nh;

ros::Subscriber<geometry_msgs::Twist> driveSubscriber("/cmd_vel", &driveCallback) ;
ros::Subscriber<std_msgs:: Int32> usonicSubscriber_0("usonic_data_0", &stopCallback_0);
ros::Subscriber<std_msgs:: Int32> usonicSubscriber_1("usonic_data_1", &stopCallback_1);
ros::Publisher pub("pos", &vector3); 

//pwm logging function
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

//encoder logging function
void encoder_log(const geometry_msgs::Vector3 vector3){
  
  char log_msg_x[10], log_msg_y[10];
  dtostrf(vector3.x, 6, 2, x);
  dtostrf(vector3.y, 6, 2, y);

  sprintf(x,"rotation_x =%s", log_msg_x);
  sprintf(y,"rotation_y =%s", log_msg_y);
  
  nh.loginfo(log_msg_x);
  nh.loginfo(log_msg_y);

}


void encoder_pub() {

  rot_tot_x = rot[0] + cnt[0]/50;
  rot_tot_y = rot[1] + cnt[1]/50;
  vector3.x = rot_tot_x/19;
  vector3.y = rot_tot_y/19;
  pub.publish(&vector3);
  //Serial.print(cnt[0]);
  //Serial.print(',');
  //Serial.print(rot[0]);
  //Serial.print(',');
  //Serial.println(rot_tot_x);

  //loggin encoder
  //encoder_log(vector3);
}

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
  
  pinMode (outA_x,INPUT_PULLUP);
  pinMode (outB_x,INPUT_PULLUP);
  pinMode (outZ_x,INPUT_PULLUP);
  pinMode (outA_y,INPUT_PULLUP);
  pinMode (outB_y,INPUT_PULLUP);
  pinMode (outZ_y,INPUT_PULLUP);
  attachInterrupt(0, encoderCountx, FALLING);
  attachInterrupt(5, encoderResetx, FALLING);
  attachInterrupt(4, encoderCounty, FALLING);
  attachInterrupt(2, encoderResety, FALLING);
  nh.advertise(pub);
  
  delay(100) ;

}

void loop() {
  encoder_pub();
  nh.spinOnce();
  delay(1);
}
