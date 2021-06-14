#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Vector3.h>
#include <sstream>
#include <iostream>
#include <pubsub/custom.h>
#include <geometry_msgs/PoseStamped.h>
#include <kinova_msgs/KinovaPose.h>
#include <math.h>
#include <pubsub/keystateMsg.h>
#include <kinova_msgs/PoseVelocity.h>
#include <pubsub/finger.h>
#include "std_msgs/Float32.h"


const float R = 5;
float X, Y, Z, RX, RY, RZ;   // pose
float px=0, py=-0.4, pz=0.5, prx=90, pry=0, prz=0, dist=0.2, rxp, ryp, rzp; // temperary pose
ros::Publisher posevelGoal, chatter_pub, gripGoal;  //publishers
int cmd_x, cmd_y, cmd_z, cmd_r, cmd_a, cmd_s; // buttens
int state; //turn or move
float cdist = 0.07;

pubsub::custom msg;

void home(){
  msg.x = 0;
  msg.y = -0.45;
  msg.z = 0.5;
  msg.rx = 90;
  msg.ry = 0;
  msg.rz = 0;
}

void calculater(float x, float y, float z, float rx, float ry, float rz, float di){
float a, b, c, d=0, e=0 , f=1, ka, kb, kc;

std::cout<<"rx: "<<rx<< std::endl;
std::cout<<"ry: "<<ry<< std::endl;
std::cout<<"rz: "<<rz<< std::endl;

ka = d*(cos(ry)*cos(rz))+e*(cos(ry)*(-sin(rz)))+f*sin(ry);
kb = d*(((-sin(rx))*(-sin(ry)))*cos(rz)+cos(rx)*sin(rz))+e*(((-sin(rx))*(-sin(ry)))*(-sin(rz))+cos(rx)*cos(rz))+f*((-sin(rx))*cos(ry));
kc = d*((cos(rx)*(-sin(ry)))*cos(rz)+sin(rx)*sin(rz))+e*((cos(rx)*(-sin(ry)))*(-sin(rz))+sin(rx)*cos(rz))+e*(cos(rx)*cos(ry));

//ka = (d*cos(ry)+(e*sin(rx)+f*cos(rx))*sin(ry))*cos(rz)+(e*cos(rx)+f*(-sin(rx)))*(-sin(rz));
//kb = (d*cos(ry)+(e*sin(rx)+f*cos(rx))*sin(ry))*sin(rz)+(e*cos(rx)+f*(-sin(rx)))*cos(rz);
//kc = d*(-sin(ry))+(e*sin(rx)+f*cos(rx))*cos(ry);

a=x+(di-cdist)*ka;
b=y+(di-cdist)*kb;
c=z+(di-cdist)*kc;

std::cout<<"x + "<<ka<< std::endl;
std::cout<<"y + "<<kb<< std::endl;
std::cout<<"z + "<<kc<< std::endl;

X = a;
Y = b;
Z = c;
RX = prx;
RY = pry;
RZ = prz;
}


void poseVel(float x, float y, float z){ //for publishing velocity
  kinova_msgs::PoseVelocity poseV;
  poseV.twist_linear_x = x;
  poseV.twist_linear_y = y;
  poseV.twist_linear_z = z;
  posevelGoal.publish(poseV);
}

void fingControl(int x){
  pubsub::finger grip;
  grip.x = x;
  gripGoal.publish(grip);
}

void automation(){
  calculater(px, py, pz, rxp, ryp, rzp, dist);
  msg.x = X;
  msg.y = Y;
  msg.z = Z;
  msg.rx = RX;
  msg.ry = RY;
  msg.rz = RZ;
  std::cout << msg << std::endl; //print pose
  chatter_pub.publish(msg);
  sleep(4);
  fingControl(100);
  sleep(1);
  home();
  chatter_pub.publish(msg);
}

void cb_dist(const std_msgs::Float32::ConstPtr & di){
  dist = di->data;
  std::cout <<"distens: "<<dist<< std::endl;
}

void ControllerCallback(const pubsub::keystateMsg::ConstPtr & msg){ //receiving keystate
  cmd_x = msg->x;
  cmd_y = msg->y;
  cmd_z = msg->z;
  cmd_r = msg->r;
  cmd_a = msg->a;
  cmd_s = msg->s;
}


void callback(const kinova_msgs::KinovaPose::ConstPtr & pose){ //j2n6s300_driver/out/cartesian_command
float x;
px = pose->X;
py = pose->Y;
pz = pose->Z;

rxp = pose->ThetaX;
ryp = pose->ThetaY;
rzp = pose->ThetaZ;

x = pose->ThetaX;
prx = ((x)/(2*M_PI))*360;

x = pose->ThetaY;
pry = ((x)/(2*M_PI))*360;

x = pose->ThetaZ;
prz = ((x)/(2*M_PI))*360;
}
 

int main(int argc, char **argv){

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;
  ros::NodeHandle nh;
  ros::NodeHandle vel;
  
  ros::Subscriber sub = n.subscribe("j2n6s300_driver/out/cartesian_command",1, callback);
  ros::Subscriber sub2 = vel.subscribe("direction", 1, ControllerCallback);
  ros::Subscriber sub3 = nh.subscribe("CamDist", 1, cb_dist);

  chatter_pub = n.advertise<pubsub::custom>("pose", 1);
  posevelGoal = vel.advertise<kinova_msgs::PoseVelocity>("/j2n6s300_driver/in/cartesian_velocity", 1);
  gripGoal = nh.advertise<pubsub::finger>("fingers",1);

  ros::Rate loop_rate(100);

  while (ros::ok())
  {

    switch(cmd_x) {
      case(-1): {
        if(state == 0){
          std::cout << "down" << std::endl;
          poseVel(0,0,-0.15);
        }
        if(state == 1){
          msg.x = px;
          msg.y = py;
          msg.z = pz;
          msg.rx = prx;
          msg.ry = pry;
          msg.rz = prz-R;
          std::cout <<"rotate -z"<< std::endl; //print pose
          chatter_pub.publish(msg);
        }
        break;
      }
      case(1): {
        if(state == 0){
          std::cout << "up" << '\n';
          poseVel(0,0,0.15);
        }
        if(state == 1){
          msg.x = px;
          msg.y = py;
          msg.z = pz;
          msg.rx = prx;
          msg.ry = pry;
          msg.rz = prz+R;
          std::cout <<"rotate +z"<< std::endl; //print pose
          chatter_pub.publish(msg);
        }
        break;
      }
    }

    switch(cmd_y) {
      case(-1): {
        if(state == 0){  
          std::cout << "right" << '\n';
          poseVel(-0.15,0,0);
        }
        if(state == 1){
          msg.x = px;
          msg.y = py;
          msg.z = pz;
          msg.rx = prx;
          msg.ry = pry+R;
          msg.rz = prz;
          std::cout <<"rotate +y"<< std::endl; //print pose
          chatter_pub.publish(msg);
        }
        break;
      }
      case(1): {
        if(state == 0){
          std::cout << "left" << '\n';
          poseVel(0.15,0,0);
        }
        if(state == 1){
          msg.x = px;
          msg.y = py;
          msg.z = pz;
          msg.rx = prx;
          msg.ry = pry-R;
          msg.rz = prz;
          std::cout <<"rotate -y"<< std::endl; //print pose
          chatter_pub.publish(msg);
        }
        break;
      }
    }

    switch(cmd_z) {
      case(-1): {
        if(state == 0){
          std::cout << "backward" << '\n';
          poseVel(0,0.15,0);
        }
        if(state == 1){
          msg.x = px;
          msg.y = py;
          msg.z = pz;
          msg.rx = prx+R;
          msg.ry = pry;
          msg.rz = prz;
          std::cout <<"rotate +x"<< std::endl; //print pose
          chatter_pub.publish(msg);
        }
        break;
      }
      case(1): {
        if(state == 0){
          std::cout << "forwards" << '\n';
          poseVel(0,-0.15,0);
        }
        if(state == 1){
          msg.x = px;
          msg.y = py;
          msg.z = pz;
          msg.rx = prx-R;
          msg.ry = pry;
          msg.rz = prz;
          std::cout <<"rotate -x"<< std::endl; //print pose
          chatter_pub.publish(msg);
        }
        break;
      }
    }
    switch(cmd_r) {
      case(-1): {
          std::cout << "Release" << '\n';
          fingControl(0);
        }
        break;
      case(1): {
          std::cout << "grap" << '\n';
          fingControl(100);
        break;
      }
    }
    switch(cmd_a) {
      case(1): {
        std::cout << "auto" << '\n';
        automation();
        break;
      }
      case(-1): {
        std::cout << "Home" << '\n';
        home();
        chatter_pub.publish(msg);
      }
    }
    switch(cmd_s){
      case(1): {
        std::cout << "move" << '\n';
        state = 0;
        break;
      }
      case(-1): {
        std::cout << "turn" << '\n';
        state = 1;
        break;
      }
    }

  ros::spinOnce();

  loop_rate.sleep();
  }
  ros::spin();
  return 0;
}
