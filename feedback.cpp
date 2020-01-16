#include <ros/ros.h>
#include <ct/core/core.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <ct/core/examples/CustomController.h>


class Input{
public:
    double joint_ang[12];
    double joint_vel[12];
    geometry_msgs::Vector3 base_ang_vl;
    geometry_msgs::Vector3 base_lin_acc;
    geometry_msgs::Quaternion base_quat;

};

class MyFirstController : public ct::core::Controller<36, 12>
{
public:
    static const size_t state_dim = 36;    // two states
    static const size_t control_dim = 12;  // one control action
    MyFirstController(const ct::core::ControlVector<control_dim>& uff,  // feedforward control
                     const double * kp,                                              // P gain
                     const double * kd                                               // D gain
    )
            : uff_(uff), kp_HAA_(kp[0]), kp_HFE_(kp[1]), kp_KFE_(kp[2]), kd_HAA_(kd[0]), kd_HFE_(kd[1]), kd_KFE_(kd[2])
    {
    }
    ~MyFirstController() {}
    MyFirstController(const MyFirstController& other) : uff_(other.uff_),
    kp_HAA_(other.kp_HAA_),  kp_HFE_(other.kp_HFE_),  kp_KFE_(other.kp_KFE_),
    kd_HAA_(other.kd_HAA_), kd_HFE_(other.kd_HFE_), kd_KFE_(other.kd_KFE_)
    {}
    MyFirstController* clone() const override
    {
        return new MyFirstController(*this);  // calls copy constructor
    }
    void computeControl(const ct::core::StateVector<state_dim>& state,
                        const double& t,
                        ct::core::ControlVector<control_dim>& controlAction) override
    {
        controlAction = uff_;                                 // apply feedforward control
        for(int i=0;i<4;i++)
        {
            controlAction(i*3) -= state(i*3+6)*kp_HAA_ + state(i*3+24)*kd_HAA_;
            controlAction(i*3+1) -= state(i*3+7)*kp_HFE_ + state(i*3+25)*kd_HFE_;
            controlAction(i*3+2) -= state(i*3+8)*kp_KFE_ + state(i*3+26)*kd_KFE_;
        }
    }
private:
    ct::core::ControlVector<control_dim> uff_;
    
    
    double kp_HAA_;
    double kp_HFE_;
    double kp_KFE_;
    double kd_HAA_;
    double kd_HFE_;
    double kd_KFE_;
};

Input input_act;
Input input_des;



void state_update( ct::core::StateVector<MyFirstController::state_dim> & state)
{
    for(int i=0;i<12;i++)
    {
        state(i) = input_act.joint_ang[i];
        state(i+24) = input_act.joint_vel[i];
    }
}


void joint_states_callback(const sensor_msgs::JointStateConstPtr & joint_states_msg);
void ImuCallback(const sensor_msgs::ImuConstPtr imu);

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "node2");
    ROS_INFO("node2");
    ros::NodeHandle n;
    int cnt = 0;
    ct::core::ControlVector<12> tau_joint_pd;
    ct::core::StateVector<36> state_vector;
    ros::Subscriber joint_states_sub = n.subscribe(
        "/pegasus2_model/joint_states", 1, joint_states_callback);
    ros::Subscriber base_state_sub = n.subscribe(
        "/pegasus2_model/imu", 1, ImuCallback
    );


        // a damped oscillator has two states, position and velocity
    const size_t state_dim = 36;      // = 2
    const size_t control_dim = 12;
    // create a state
    ct::core::StateVector<state_dim> x;
    // we initialize it at a point with unit deflection and zero velocity
    x << 0;
    
    // create our oscillator
   
    
    // create our controller
    double kp[3] = {10, 10, 10};
    double kd[3] = {1, 1, 1};
    ct::core::ControlVector<control_dim> uff;
    ct::core::ControlVector<control_dim> controlAction;
    uff << 0.0;
    std::shared_ptr<MyFirstController> controller(new MyFirstController(uff, kp, kd));

    

    // print the new state
    //std::cout << "state after integration: " << x.transpose() << std::endl;
    
    ros::Rate loop_rate(1000);
    
    while(ros::ok())
    {
        state_update(x);

        controller->computeControl(x, 0, controlAction);
        ros::spinOnce();
        std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl;
        std::cout << "oneloop end" << cnt << std::endl; 
        std::cout << "joint[1] = " << input_act.joint_ang[0] << std::endl;
        std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl;
        cnt ++;
        loop_rate.sleep();
        
    }
    return 0;




}


void joint_states_callback(const sensor_msgs::JointStateConstPtr & joint_states_msg)
{

    //ROS_INFO("joint_space %f", joint_states_msg->position[0]);
    for(int i=0;i < 12; i++)
    {
        input_act.joint_ang[i] = joint_states_msg->position[i];
        input_act.joint_vel[i] = joint_states_msg->velocity[i];
    }
}

void ImuCallback(const sensor_msgs::ImuConstPtr imu)
{
    //ROS_INFO("base state");
    input_act.base_ang_vl = imu->angular_velocity;
    input_act.base_lin_acc = imu->linear_acceleration;
    input_act.base_quat = imu->orientation;
}



