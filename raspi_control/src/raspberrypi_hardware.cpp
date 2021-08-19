#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <ros/package.h> 
#include <signal.h>
#include "std_srvs/Trigger.h"

bool is_on = false;
bool callbackOn(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
bool callbackOff(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
void onSigint(int);

bool setPower(bool on)
{
        std::ofstream ofs("/dev/rtmotoren0");
        if(not ofs.is_open())
                return false;

        ofs << (on ? '1' : '0') << std::endl;
        is_on = on;
        return true;
}

bool callbackOn(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
        if(not setPower(true))
                return false;

        response.message = "ON";
        response.success = true;
        return true;
}

bool callbackOff(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
        if(not setPower(false))
                return false;

        response.message = "OFF";
        response.success = true;
        return true;
}

void onSigint(int sig)
{
        setPower(false);
        exit(0);
}



class RaspberryPiHW : public hardware_interface::RobotHW
{
public:
    RaspberryPiHW(){
      ros::NodeHandle pnh;
      std::fill_n(pos,2,0.0);
      std::fill_n(vel,2,0.0);
      std::fill_n(eff,2,0.0);

      hardware_interface::JointStateHandle state_wheel_r_handle("wheel_r_joint",&pos[0],&vel[0],&eff[0]);
      jnt_state_interface.registerHandle(state_wheel_r_handle);

      hardware_interface::JointStateHandle state_wheel_l_handle("wheel_l_joint",&pos[1],&vel[1],&eff[1]);
      jnt_state_interface.registerHandle(state_wheel_l_handle);

      registerInterface(&jnt_state_interface);

      hardware_interface::JointHandle vel_wheel_r_handle(jnt_state_interface.getHandle("wheel_r_joint"),&cmd[0]);
      jnt_vel_interface.registerHandle(vel_wheel_r_handle);

      hardware_interface::JointHandle vel_wheel_l_handle(jnt_state_interface.getHandle("wheel_l_joint"),&cmd[1]);
      jnt_vel_interface.registerHandle(vel_wheel_l_handle);

      registerInterface(&jnt_vel_interface);

      pnh.getParam("/Raspimouse/diff_drive_controller/wheel_radius",wheel_radius_);
      ROS_INFO("wheel_radius=%f:",wheel_radius_);
    }
    ros::Time getTime() const { return ros::Time::now(); }
    ros:: Duration getDuration( ros::Time t) const { return ( ros::Time::now() -t); }

    void read(ros::Duration d);
    void write();

private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    double cmd[2];
    double pos[2];
    double vel[2];
    double eff[2];
    double wheel_radius_;
};

void RaspberryPiHW::read(ros::Duration d){
    pos[0] += vel[0]*d.sec;
    vel[0] = cmd[0];
    pos[1] += vel[1]*d.sec;
    vel[1] = cmd[1];
    ROS_INFO("cmd=%f %f %f %f",vel[0],vel[1],pos[0],pos[1]);
};

void RaspberryPiHW::write(){
int left,right;
    std::ofstream ofsL("/dev/rtmotor_raw_l0");
    std::ofstream ofsR("/dev/rtmotor_raw_r0");
    if( (not ofsL.is_open()) or (not ofsR.is_open()) ){
            ROS_ERROR("Cannot open /dev/rtmotor_raw_{l,r}0");
            return;
    }

    left = (int)round(cmd[1]/(2.0*3.14159*wheel_radius_/400.0)/1000*24);
    right = (int)round(cmd[0]/(2.0*3.14159*wheel_radius_/400.0)/1000*24);
    ROS_INFO("left=%d right=%d %f ",left,right,wheel_radius_);
    ofsL << left << std::endl;
    ofsR << right << std::endl;
};

int main(int argc,char **argv)
{
    ros::init(argc,argv,"RaspberryPiMouse");
    ros::NodeHandle nh,n;

    std::string onoff;
    onoff = "off";
    setPower(onoff == "on");

    signal(SIGINT, onSigint);

    ros::ServiceServer srv_on = n.advertiseService("motor_on", callbackOn);
    ros::ServiceServer srv_off = n.advertiseService("motor_off", callbackOff);

    RaspberryPiHW raspimouse;
    controller_manager::ControllerManager cm (&raspimouse,nh);
    
    ros::Rate loop_rate(10);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time t = ros::Time::now();
    ros::Duration  d = ros::Time::now() -t;

    while( ros::ok() ){
    d=ros::Time::now() -t;
    t=ros::Time::now();

    raspimouse.read(d );
    cm.update(t,d);
    raspimouse.write( );
    loop_rate.sleep();
    }

    spinner.stop();

    return 0;
}

