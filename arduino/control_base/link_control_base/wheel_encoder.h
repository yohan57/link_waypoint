#include <ros.h>
#include <geometry_msgs/Vector3.h>
#define outA_x 2
#define outB_x 3
#define outZ_x 18
#define outA_y 19
#define outB_y 20
#define outZ_y 21

//ros::NodeHandle nh;
geometry_msgs::Vector3 vector3;

volatile float cnt[2] = {0,0};
volatile signed long rot[2] = {0,0};
volatile signed int state[2] = {1,1};
float rot_tot_x = 0;
float rot_tot_y = 0;
char x[8];
char y[8];

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
