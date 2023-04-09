#include <ros.h>
#include <ArduinoHardware.h>
#include <geometry_msgs/Twist.h>

/***********************************************************************
 * Macro
 **********************************************************************/
#define EN_L 12
#define IN1_L 7
#define IN2_L 8

#define EN_R 11
#define IN1_R 5
#define IN2_R 6

/***********************************************************************
 * Global variables
 **********************************************************************/
double w_r = 0.0;
double w_l = 0.0;

//wheel_rad is the wheel radius ,wheel_sep is
double wheel_rad = 0.03;
double wheel_sep = 0.32;

double speed_ang = 0.0;
double speed_lin = 0.0;

/***********************************************************************
 * Function
 **********************************************************************/
void messageCb( const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = -msg.linear.x;
  w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
}

// ROS
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

/***********************************************************************
 * prottype
 **********************************************************************/
void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);

void setup(){
  Motors_init();
  nh.initNode();
  nh.subscribe(sub);
}

void loop(){
  MotorL(-w_l * 30);
  MotorR(-w_r*  30);
  nh.spinOnce();
}

/***********************************************************************
 * Function
 **********************************************************************/
void Motors_init(){
    pinMode(EN_L, OUTPUT);
    pinMode(EN_R, OUTPUT);
    pinMode(IN1_L, OUTPUT);
    pinMode(IN2_L, OUTPUT);
    pinMode(IN1_R, OUTPUT);
    pinMode(IN2_R, OUTPUT);
    digitalWrite(EN_L, LOW);
    digitalWrite(EN_R, LOW);
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, LOW);
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, LOW);
}

void MotorL(int Pulse_Width1){
 if (Pulse_Width1 > 0){
     if (Pulse_Width1 > 240) {
        Pulse_Width1 = 240;
     }

     analogWrite(EN_L, Pulse_Width1);
     digitalWrite(IN1_L, LOW);
     digitalWrite(IN2_L, HIGH);
     /*digitalWrite(IN1_L, LOW);
     digitalWrite(IN2_L, HIGH);*/
 }

 if (Pulse_Width1 < 0){
     Pulse_Width1=abs(Pulse_Width1);
     if (Pulse_Width1 > 240) {
        Pulse_Width1 = 240;
     }

     analogWrite(EN_L, Pulse_Width1);
     digitalWrite(IN1_L, HIGH);
     digitalWrite(IN2_L, LOW);
     /*digitalWrite(IN1_L, LOW);
     digitalWrite(IN2_L, HIGH);*/
 }

 if (Pulse_Width1 == 0){
     analogWrite(EN_L, Pulse_Width1);
     digitalWrite(IN1_L, LOW);
     digitalWrite(IN2_L, LOW);
 }
}

void MotorR(int Pulse_Width2){
 if (Pulse_Width2 > 0){
     if (Pulse_Width2 > 240) {
        Pulse_Width2 = 240;
     }

     analogWrite(EN_R, Pulse_Width2);
     digitalWrite(IN1_R, LOW);
     digitalWrite(IN2_R, HIGH);
     /*digitalWrite(IN1_R, HIGH);
     digitalWrite(IN2_R, LOW);*/
 }

 if (Pulse_Width2 < 0){
     Pulse_Width2=abs(Pulse_Width2);
     if (Pulse_Width2 > 240) {
        Pulse_Width2 = 240;
     }
     
     analogWrite(EN_R, Pulse_Width2);
     digitalWrite(IN1_R, HIGH);
     digitalWrite(IN2_R, LOW);
     /*digitalWrite(IN1_R, LOW);
     digitalWrite(IN2_R, HIGH);*/
 }

 if (Pulse_Width2 == 0){
     analogWrite(EN_R, Pulse_Width2);
     digitalWrite(IN1_R, LOW);
     digitalWrite(IN2_R, LOW);
 }
}