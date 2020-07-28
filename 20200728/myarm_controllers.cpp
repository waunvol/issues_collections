#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <boost/thread.hpp>


class Arm_hwinterface : public hardware_interface::RobotHW
{
    private:
        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::VelocityJointInterface jnt_vel_interface;
        std::string sub_name[4]={"joint_state0", "joint_state1", "joint_state2", "joint_state3"};
        std::string pub_name[4]={"cmd1", "cmd2", "cmd3", "cmd4"};
        double cmd[4] = {0,0,0,0};
        double pos[4] = {0,0,0,0};
        double vel[4] = {0,0,0,0};
        double eff[4] = {2.0,2.0,2.0,2.0};
        double _pos[4] = {0,0,0,0};
        std::vector<float> _cmd = {0,0,0,0};

        ros::NodeHandle n;
        ros::Publisher pub_cmd[4];
        ros::Publisher cmd_pub;
        ros::Subscriber sub_pos[4];
        ros::Subscriber pos_sub;
        std_msgs::Float32 cmd_msg[4];  
        std_msgs::Float32 monitor; 
        std_msgs::Float32MultiArray cmdmsg;  

    public:
        void rec_callback(const geometry_msgs::Vector3::ConstPtr& msg)
        {
            int i, temp;
            i = msg->x;
            _pos[i] = msg->y;
        }

        void pos_callback(const std_msgs::Int16MultiArray::ConstPtr& msg0)
        {
            for(int i=0; i<4; i++)
            {
                _pos[i] = msg0->data[i];
            }
            // ROS_INFO("%f, %f, %f, %f", _pos[0], _pos[1], _pos[2], _pos[3]);
        }


        Arm_hwinterface()
        {
            // connect and register the joint state interface
            for(int i=0; i<4; i++)
            {
                std::string joint_name = "joint";
                std::string joint_num = std::to_string(i);
                joint_name.append(joint_num);
                hardware_interface::JointStateHandle jnt_state_handle_tmp(joint_name, &pos[i], &vel[i], &eff[i]);
                jnt_state_interface.registerHandle(jnt_state_handle_tmp);
            }
            registerInterface(&jnt_state_interface);

            // connect and register the joint velocity interface
            for(int i=0; i<4; i++)
            {
                std::string joint_name = "joint";
                std::string joint_num = std::to_string(i);
                joint_name.append(joint_num);
                hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(joint_name), &cmd[i]);
                jnt_vel_interface.registerHandle(vel_handle);
            }
            registerInterface(&jnt_vel_interface);


            cmd_pub = n.advertise<std_msgs::Float32MultiArray>("cmd", 1);
            pos_sub = n.subscribe("jointstates", 1, &Arm_hwinterface::pos_callback, this);


            for(int i=0; i<4; i++)
            {
                pub_cmd[i]=n.advertise<std_msgs::Float32>(pub_name[i], 1);

            }

        }

        void write()
        {
            // ROS_INFO("%lf, %lf, %lf, %lf", cmd[0], cmd[1], cmd[2], cmd[3]);
            for(int i=0; i<4; i++)
            {
                _cmd[i] = cmd[i];
            }
            // ROS_INFO("%lf, %lf, %lf, %lf", _cmd[0], _cmd[1], _cmd[2], _cmd[3]);



            cmdmsg.data = _cmd;
            cmd_pub.publish(cmdmsg);


            for(int i=0; i<4; i++)
            {
                vel[i] = cmd[i];
            }
        }

        void read()
        {
            /*mid value = 2000, 90' = 1000
              as5600: max=>4096
            */
            for(int i=1; i<4; i++)
            {
                pos[i] = (_pos[i] - 205)*0.0157;
            }
            pos[0] = 0;

            // ROS_INFO("%f, %f, %f, %f", pos[0], pos[1], pos[2], pos[3]);
            // ROS_INFO("%f, %f, %f, %f", _pos[0], _pos[1], _pos[2], _pos[3]);
            for(int i=0; i<4; i++)
            {
                monitor.data = pos[i];
                pub_cmd[i].publish(monitor);
            }




        }

        /*test funtion, change pos manually*/
        void change_pos(double num)
        {
            for(int i=0; i<4; i++)
            {
                pos[i] = num;
            }
        }

        

        ros::Time getTime() const {return ros::Time::now();}
        ros::Duration getPeriod() const {return ros::Duration(0.01);}

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_controllers");
    ros::NodeHandle nh;

    Arm_hwinterface bot;
    controller_manager::ControllerManager ctrl(&bot);	//class of controller，link to bot

	ros::AsyncSpinner spinner(3);
	spinner.start();
    ros::Rate rate(50);

	// ROS_INFO("HW has been launch!");
    double test = 0;
    double test_flag=0;

	while(ros::ok())
	{
        // if(test_flag==0)
        // {
        //     test += 0.01;
        //     if(test > 0.02)
        //         test_flag = 1;
        // }
        // else
        // {
        //     test -= 0.01;
        //     if(test < -0.02)
        //         test_flag = 0;
        // }
        // bot.change_pos(test);
        // ROS_INFO("%f", test);
        
        bot.read();
        ctrl.update(bot.getTime(), bot.getPeriod());

        bot.write();	
        rate.sleep();
	}
	spinner.stop();

	return 0;

    // nh.createTimer

}