
#include <ros.h>
#include <geometry_msgs/Vector3.h>

//ros::NodeHandle nh;
geometry_msgs::Vector3 vector3;
ros::Publisher pub("pos", &vector3); 

#define outA_x 2
#define outB_x 3
#define outZ_x 18
#define outA_y 19
#define outB_y 20
#define outZ_y 21


volatile float cnt[2] = {0,0};
volatile signed long rot[2] = {0,0};
volatile signed int state[2] = {1,1};
float rot_tot_x = 0;
float rot_tot_y = 0;
char x[8];
char y[8];

//encoder count & reset function

void encoderCountx() { 
    if(digitalRead(outA_x)!= digitalRead(outB_x)){
      cnt[0]+=1;
      state[0]=1; 
    }
    else{
      cnt[0]+=-1;
      state[0]=-1;
    }
}
void encoderResetx() {
    cnt[0] = 0;
    if(state[0]==1){
      rot[0]+=1;
    }
    else{
      rot[0]+=-1;
    }
} 

void encoderCounty() { 
    if(digitalRead(outA_y)!= digitalRead(outB_y)){
      cnt[1]+=1;
      state[1]=1; 
    }
    else{
      cnt[1]+=-1;
      state[1]=-1;
    }
}
void encoderResety() {
    cnt[1] = 0;
    if(state[1]==1){
      rot[1]+=1;
    }
    else{
      rot[1]+=-1;
    }
}
//encoder logging and publishing function
void encoder_log(){
  
  char log_msg_x[10], log_msg_y[10];
  dtostrf(rot_tot_x, 6, 2, x);
  dtostrf(rot_tot_y, 6, 2, y);

  sprintf(x,"rotation_x =%s", log_msg_x);
  sprintf(y,"rotation_y =%s", log_msg_y);
  
  nh.loginfo(log_msg_x);
  nh.loginfo(log_msg_y);

}

void encoder_pub() {

  rot_tot_x = rot[0] + cnt[0]/50;
  rot_tot_y = rot[1] + cnt[1]/50;
  vector3.x = rot_tot_x;
  vector3.y = rot_tot_y;
  pub.publish(&vector3);
  //Serial.print(cnt[0]);
  //Serial.print(',');
  //Serial.print(rot[0]);
  //Serial.print(',');
  //Serial.println(rot_tot_x);
  
  encoder_log();
}

void setup() {

    Serial.begin(115200);
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
    
    nh.initNode();
    nh.advertise(pub);

}
 
void loop() {
    encoder_pub();    
    nh.spinOnce();
    delay(10);
} 
