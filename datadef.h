
#include <ros/ros.h>




class Input{
public:
    double joint_ang[12];
    double joint_vel[12];
    geometry_msgs::Vector3 base_ang_vl;
    geometry_msgs::Vector3 base_lin_acc;
    geometry_msgs::Quaternion base_quat;

};






