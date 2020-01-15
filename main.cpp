

#include <ros/ros.h>


#include <Eigen/Dense>


#include <ct/core/core.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include "datadef.h"

Input input;




void joint_states_callback(const sensor_msgs::JointStateConstPtr & joint_states_msg)
{

    ROS_INFO("joint_space %f>>>>%d", joint_states_msg->position[0], i);
    for(int i=0;i < 12; i++)
    {
        input.joint_ang[i] = joint_states_msg->position[i];
        input.joint_vel[i] = joint_states_msg->velocity[i];
    }    
}

void ImuCallback(const sensor_msgs::ImuConstPtr imu)
{
    input.base_ang_vl = imu->angular_velocity;
    input.base_lin_acc = imu->linear_acceleration;
    input.base_quat = imu->orientation;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "node1");
    ros::NodeHandle n;
    ros::Subscriber joint_states_sub = n.subscribe(
        "/pegasus2_model/joint_states", 1, joint_states_callback);
    ros::Subscriber base_state_sub = n.subscribe(
        "/pegasus2_model/imu", 1, ImuCallback
    );
    //ros::Subscriber sub_lf_foot_bumper = n.subscribe(
      //  "/pegasus2_model/lf_foot_bumper", 1000, &lf_foot_bumper_callback);
    ros::spin();

}








