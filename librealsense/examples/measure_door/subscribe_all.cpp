#include "ros/ros.h"
//#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h" 
 
//void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) //Note it is geometry_msgs::PoseStamped, not std_msgs::PoseStamped
void chatterCallback(const geometry_msgs::PoseArray msg)
{
    for(int i=0;i<msg.poses.size();i++){
        geometry_msgs::Pose pose = msg.poses[i];
        ROS_INFO("I heard the pose from the robot"+i); 
        ROS_INFO("the position(x,y,z) is %lf , %lf, %lf", pose.position.x, pose.position.y, pose.position.z);
        ROS_INFO("the orientation(x,y,z,w) is %f , %f, %f, %f", pose.orientation.x, pose.orientation.y, pose.orientation.z,pose.orientation.w);
    }
    //ROS_INFO("I heard the pose from the robot"); 
    //ROS_INFO("the position(x,y,z) is %lf , %lf, %lf", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    //ROS_INFO("the orientation(x,y,z,w) is %f , %f, %f, %f", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg[0]->pose.orientation.w);
    //ROS_INFO("the time we get the pose is %f",  msg->header.stamp.sec + 1e-9*msg->header.stamp.nsec);
 
    std::cout<<"\n \n"<<std::endl; //add two more blank row so that we can see the message more clearly
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "subscriber");
 
    ros::NodeHandle n;
 
    ros::Subscriber sub = n.subscribe("target_pose", 10, chatterCallback);
 
    ros::spin();
 
    return 0;
}
