#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h" //include posestamp head file
 
#include <cmath>//for sqrt() function
#include <iostream>
#include "readPython.h"

using namespace std; 
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_pub");
 
    ros::NodeHandle n;
 
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("target_pose", 10); //initialize chatter
 
    ros::Rate loop_rate(10);
    
    struct position p_pub = getPosition();
    ROS_INFO("the position(x,y,z) is %f , %f, %f", p_pub.x, p_pub.y, p_pub.z);
    
 
    //generate pose by ourselves.
    float positionX, positionY, positionZ;
    //int positionX = 0, positionY = 0, positionZ = 0;
    //double orientationX, orientationY, orientationZ, orientationW;
 
    //We just make the robot has fixed orientation. Normally quaternion needs to be normalized, which means x^2 + y^2 + z^2 +w^2 = 1
    //double fixedOrientation = 0.1;
    //orientationX = fixedOrientation ;
    //orientationY = fixedOrientation ;
    //orientationZ = fixedOrientation ;
    //orientationW = sqrt(1.0 - 3.0*fixedOrientation*fixedOrientation); 
 
    //double count = 0.0;
    while (ros::ok())
    {
        //We just make the position x,y,z all the same. The X,Y,Z increase linearly
        positionX = p_pub.x;
        positionY = p_pub.y;
        positionZ = p_pub.z;
        /*positionX = 1;
        positionY = 1;
        positionZ = 1;*/
 
        geometry_msgs::PoseStamped msg; 
 
        //assign value to poseStamped
 
            //First assign value to "header".
        ros::Time currentTime = ros::Time::now();
        msg.header.stamp = currentTime;
 
            //Then assign value to "pose", which has member position and orientation
        msg.pose.position.x = positionX;
        msg.pose.position.y = positionY;
        msg.pose.position.z = positionY;
 
        msg.pose.orientation.x = 0;
        msg.pose.orientation.y = 0;
        msg.pose.orientation.z = 0;
        msg.pose.orientation.w = 0;
 
        //ROS_INFO("we publish the robot's position and orientaion"); 
        ROS_INFO("we publish the robot's position"); 
        ROS_INFO("the position(x,y,z) is %f , %f, %f", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
        ROS_INFO("the orientation(x,y,z,w) is %f , %f, %f, %f", msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
        ROS_INFO("the time we get the pose is %f",  msg.header.stamp.sec + 1e-9*msg.header.stamp.nsec);
 
        std::cout<<"\n \n"<<std::endl; //add two more blank row so that we can see the message more clearly
 
        chatter_pub.publish(msg);
 
        ros::spinOnce();
 
        loop_rate.sleep();
 
        //++count;
    }
 
 
  return 0;
}
