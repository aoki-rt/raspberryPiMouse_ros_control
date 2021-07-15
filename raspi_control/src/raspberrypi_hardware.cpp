#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class RaspberryPiHW : public hardware_interface::RobotHW
{
public:
    RaspberryPiHW(){
      ros::NodeHandle pnh("~");
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

      pnh.getParam("wheel_radius",wheel_radius_);
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
};

void RaspberryPiHW::write(){
    FILE *motor_l, *motor_r;
    char s_l[10],s_r[10];
    if((motor_l = fopen("/dev/rtmotor_l0","w")) != NULL &&
       (motor_r = fopen("/dev/rtmotor_r0","w")) != NULL ){
        sprintf(s_l , "%d\n", (int)(cmd[1]*2*3.14159*wheel_radius_/400) );
        sprintf(s_r , "%d\n", (int)(cmd[0]*2*3.14159*wheel_radius_/400) );       
        fputs(s_l , motor_l);
        fputs(s_r , motor_r);
    }
    fclose(motor_l);
    fclose(motor_r);
};

int main(int argc,char **argv)
{
    ros::init(argc,argv,"RaspberryPiMouse");
    ros::NodeHandle nh;

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

