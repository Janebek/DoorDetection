#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "quaternions.h"

#include "geometry_msgs/PoseStamped.h" //include posestamp head file

using namespace std;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("target_pose", 10); //initialize chatter
  ros::Rate loop_rate(10);

  //角度转四元数
  double setangle = 45;  //贾尼计算得到的角度　假设为45度
  struct xyzw qz = angletrans(setangle);
  cout << "四元数:\n" << "x:" << qz.x << endl << "y:" << qz.y << endl << "z:" << qz.z << endl << "w:" << qz.w << endl;
  
  
  //generate pose by ourselves.
  double positionX, positionY, positionZ;
  double orientationX, orientationY, orientationZ, orientationW;
  //We just make the robot has fixed orientation. Normally quaternion needs to be normalized, which means x^2 + y^2 + z^2 +w^2 = 1
  double fixedOrientation = 0.1;
  orientationX = qz.x;
  orientationY = qz.y;
  orientationZ = qz.z;
  orientationW = qz.w; 
  double count = 0.0;

 
  
  //struct xyz sangletrans(double angle,double a,double b,double c);
  //struct xyz qz2 = sangletrans(30,0,0,0);
  //cout << "四元数2:\n" << "w:" << qz2.w << endl;
  //print();
  //int a = 13, b = 130;
  //cout << "最大值为：" << max(a, b) << endl;//调用头文件中定义的函数
  

  while (ros::ok())
  {
    //std_msgs::String msg;
    //std::stringstream ss;
    
    //ss << "{\"name\":\"candice\",\"age\":\"18\",\"sex\":\"woman\"}" << count;
    
    //msg.data = ss.str();
    //ROS_INFO("%s", msg.data.c_str());
    //chatter_pub.publish(msg);
    //ros::spinOnce();
    //loop_rate.sleep();

    //We just make the position x,y,z all the same. The X,Y,Z increase linearly
    positionX = count;
    positionY = count;
    positionZ = count;
 
    geometry_msgs::PoseStamped msg; 
 
    //assign value to poseStamped
 
    //First assign value to "header".
    ros::Time currentTime = ros::Time::now();
    msg.header.stamp = currentTime;
 
    //Then assign value to "pose", which has member position and orientation
    //msg.pose.position.x = positionX;
    //msg.pose.position.y = positionY;
    //msg.pose.position.z = positionY;
    //msg.pose.position.x = 0;
    //msg.pose.position.y = 0;
    //msg.pose.position.z = 0;
 
    msg.pose.orientation.x = orientationX;
    msg.pose.orientation.y = orientationY;
    msg.pose.orientation.z = orientationZ;
    msg.pose.orientation.w = orientationW;
 
    ROS_INFO("we publish the robot's position and orientaion!!!"); 
    ROS_INFO("the position(x,y,z) is %f , %f, %f", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    ROS_INFO("the orientation(x,y,z,w) is %f , %f, %f, %f", msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
    ROS_INFO("the time we get the pose is %f",  msg.header.stamp.sec + 1e-9*msg.header.stamp.nsec);
 
    std::cout<<"\n \n"<<std::endl; //add two more blank row so that we can see the message more clearly
 
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
 
    ++count;
  }
  return 0;
}




