#include <math.h>
#include <Braccio.h>
#include <Servo.h>
Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;
//length of the links
const double L1 =71.5; // Length of link 1
const double L2 = 125; // Length of link.2
const double L3 = 125; // Length of link.3
//function for inverse kinematics to find angles
void inversekinematics(double X, double Y, double& A,
double& B, double& C) {
double theta = 0;
double theta1 = atan2(X, Y) * 180.0 / M_PI;
double theta2 = atan2(Y, X) * 180.0 / M_PI;
double theta3= 180.0 - theta;
double lc=L1*cos(theta);
double ld=L1*sin(theta);
double x = X - lc;
double y = Y + ld;
double theta4=atan2(y,x)* 180.0 / M_PI;
double theta5=atan2(x,y)* 180.0 / M_PI;
Source code :
double la=sqrt(X*X+Y*Y);
double le=sqrt(x*x+y*y);
double theta6 = acos((le*le + L2*L2 - L3*L3)/(2*le*L2)) * 180.0 / M_PI;
double theta7 = acos((L3*L3 + le*le - L2*L2)/(2*L3*le)) * 180.0 / M_PI;
double theta8 = acos((L2*L2 + L3*L3 - le*le)/(2*L2*L3)) * 180.0 / M_PI;
A = 180.0 - theta4 + theta7;
B = theta8 - 90.0;
C = theta6 - theta5 + theta3 - 90.0;
}
//initializing the braccio
void setup(){
Braccio.begin();
Serial.begin(9600);
}
//loop for movement of the robotic arm
void loop(){
double dx = 200;
double dy = 150;
double TA,TB,TC;
inversekinematics(dx,dy,TA,TB,TC);
Serial.print("Theta 1: ");
Serial.print(TA);
Serial.print("Theta 2: ");
Serial.print(TB);
Serial.print("Theta 3: ");
Serial.print(TC);
Braccio.ServoMovement(20, 0, TA, TB, TC, 0 , 73);
delay(1000);
}
