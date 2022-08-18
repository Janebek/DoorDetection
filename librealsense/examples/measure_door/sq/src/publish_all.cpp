#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

//#include "geometry_msgs/PoseStamped.h" //include posestamp head file
#include "geometry_msgs/PoseArray.h"

#include "quaternions.h"  //get quaternions(x, y, z, w)
#include "readPython.h"   //get position(x, y, z)

using namespace std;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "publisher");
  ros::NodeHandle n;
  //ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("target_pose", 10); //initialize chatter
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseArray>("target_pose", 10); //initialize chatter
  ros::Rate loop_rate(10);

  //角度转四元数
  double setangle = 45;  //贾尼计算得到的角度　假设为45度
  struct xyzw qz = angletrans(setangle);
  ROS_INFO("the quaternions(x,y,z,w) is %lf , %lf, %lf, %lf", qz.x, qz.y, qz.z, qz.w);
  
  //坐标转换
  struct position p_pub = getPosition();
  ROS_INFO("the position(x,y,z) is %f , %f, %f", p_pub.x, p_pub.y, p_pub.z);
  
  //generate pose by ourselves.
  float positionX, positionY, positionZ;
  double orientationX, orientationY, orientationZ, orientationW;
  //We just make the robot has fixed orientation. Normally quaternion needs to be normalized, which means x^2 + y^2 + z^2 +w^2 = 1
  double fixedOrientation = 0.1;
  orientationX = qz.x;
  orientationY = qz.y;
  orientationZ = qz.z;
  orientationW = qz.w; 

  //We just make the position x,y,z all the same. The X,Y,Z increase linearly
  positionX = p_pub.x;
  positionY = p_pub.y;
  positionZ = p_pub.z;

  while (ros::ok())
  {
 
    //geometry_msgs::PoseStamped msg;
    geometry_msgs::PoseArray msg; 

    geometry_msgs::Pose init_pose;
    init_pose.position.x = positionX;
    init_pose.position.y = positionY;
    init_pose.position.z = positionZ;

    init_pose.orientation.x = orientationX;
    init_pose.orientation.y = orientationY;
    init_pose.orientation.z = orientationZ;
    init_pose.orientation.w = orientationW;
    msg.poses.push_back(init_pose);

    geometry_msgs::Pose init_pose2;
    init_pose2.position.x = positionX+1;
    init_pose2.position.y = positionY+1;
    init_pose2.position.z = positionZ+1;

    init_pose2.orientation.x = orientationX;
    init_pose2.orientation.y = orientationY;
    init_pose2.orientation.z = orientationZ;
    init_pose2.orientation.w = orientationW;
    msg.poses.push_back(init_pose2);
 
    ROS_INFO("we publish the robot's position and orientaion!!!1111111111"); 
    ROS_INFO("the position(x,y,z) is %f , %f, %f", init_pose.position.x, init_pose.position.y, init_pose.position.z);
    ROS_INFO("the orientation(x,y,z,w) is %f , %f, %f, %f", init_pose.orientation.x, init_pose.orientation.y, init_pose.orientation.z, init_pose.orientation.w);

    ROS_INFO("we publish the robot's position and orientaion!!!2222222222"); 
    ROS_INFO("the position(x,y,z) is %f , %f, %f", init_pose2.position.x, init_pose2.position.y, init_pose2.position.z);
    ROS_INFO("the orientation(x,y,z,w) is %f , %f, %f, %f", init_pose2.orientation.x, init_pose2.orientation.y, init_pose2.orientation.z, init_pose2.orientation.w);
    //ROS_INFO("the time we get the pose is %f",  msg.header.stamp.sec + 1e-9*msg.header.stamp.nsec);
 
    std::cout<<"\n \n"<<std::endl; //add two more blank row so that we can see the message more clearly
 
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
 
  }
  return 0;
}




