

#include <ros/ros.h>


#include <Eigen/Dense>


#include <ct/core/core.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include "datadef.h"

Input input;




void joint_states_callback(const sensor_msgs::JointStateConstPtr& joint_states_msg)
{

    ROS_INFO("joint_space %f>>>>%d", joint_states_msg->position[0], i);
    for(int i=0;i < 12; i++)
    {
        input.joint_ang[i] = joint_states_msg->position[i];
        input.joint_vel[i] = joint_states_msg->velocity[i];
    }    
}

void wb_ImuCallback(const nav_msgs::Odometry& tf_msg)
{
    q0 = tf_msg.pose.pose.orientation.w;
    q1 = tf_msg.pose.pose.orientation.x;
    q2 = tf_msg.pose.pose.orientation.y;
    q3= tf_msg.pose.pose.orientation.z;
    buffer[1] = 0-atan2(2 * (q2*q3+q0*q1),1-2*(q1*q1-q2*q2));
    buffer[2] = 0-asin(-2*(q1*q3-q0*q2));
    buffer[3] = atan2(2*(q1*q2+q0*q3),1-2*(q2*q2+q3*q3));
    //ROS_INFO("RPY:%f %f %f",buffer[1]*180/3.14,buffer[2]*180/3.14,buffer[3]*180/3.14);
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "node1");
    ros::NodeHandle n;
    ros::Subscriber joint_states_sub = n.subscribe(
        "/pegasus2_model/joint_states", 1, joint_states_callback);
    //ros::Subscriber sub_lf_foot_bumper = n.subscribe(
      //  "/pegasus2_model/lf_foot_bumper", 1000, &lf_foot_bumper_callback);
    ros::spin();

}








