//
// Created by sanket on 10/10/22.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "general_teleop/LegCommand.h"
#include "general_teleop/LegCommandArray.h"
#include "general_teleop/MotorCommand.h"

sensor_msgs::JointState jointStateMessage;
float hip0;
float thigh0;
float calf0;

float hip1;
float thigh1;
float calf1;

float hip2;
float thigh2;
float calf2;

float hip3;
float thigh3;
float calf3;

void jointControlCallback(const general_teleop::LegCommandArray::ConstPtr &msg)
{
  // ROS_INFO("Position 4 legs: FL: [%0.2f] [%0.2f] [%0.2f] FR: [%0.2f] [%0.2f] [%0.2f] RL: [%0.2f] [%0.2f] [%0.2f] RR: [%0.2f] [%0.2f] [%0.2f]",
  //                                         msg->leg_commands[0].motor_commands[0].pos_setpoint, 
  //                                         msg->leg_commands[0].motor_commands[1].pos_setpoint, 
  //                                         msg->leg_commands[0].motor_commands[2].pos_setpoint, 
                                          
  //                                         msg->leg_commands[1].motor_commands[0].pos_setpoint,
  //                                         msg->leg_commands[1].motor_commands[1].pos_setpoint,
  //                                         msg->leg_commands[1].motor_commands[2].pos_setpoint,
                                          

  //                                         msg->leg_commands[2].motor_commands[0].pos_setpoint,
  //                                         msg->leg_commands[2].motor_commands[1].pos_setpoint,
  //                                         msg->leg_commands[2].motor_commands[2].pos_setpoint,
                                                                        
  //                                         msg->leg_commands[3].motor_commands[0].pos_setpoint,
  //                                         msg->leg_commands[3].motor_commands[1].pos_setpoint,
  //                                         msg->leg_commands[3].motor_commands[2].pos_setpoint
  //                                         );
  
  
  hip0 = msg->leg_commands[0].motor_commands[0].pos_setpoint;
  thigh0 = msg->leg_commands[0].motor_commands[1].pos_setpoint;
  calf0 = msg->leg_commands[0].motor_commands[2].pos_setpoint;

  hip1 = msg->leg_commands[1].motor_commands[0].pos_setpoint;
  thigh1 = msg->leg_commands[1].motor_commands[1].pos_setpoint;
  calf1 = msg->leg_commands[1].motor_commands[2].pos_setpoint;

  hip2 = msg->leg_commands[2].motor_commands[0].pos_setpoint;
  thigh2 = msg->leg_commands[2].motor_commands[1].pos_setpoint;
  calf2 = msg->leg_commands[2].motor_commands[2].pos_setpoint;

  hip3 = msg->leg_commands[3].motor_commands[0].pos_setpoint;
  thigh3 = msg->leg_commands[3].motor_commands[1].pos_setpoint;
  calf3 = msg->leg_commands[3].motor_commands[2].pos_setpoint;


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;
  ros::Rate loopRate(1000);
  //memset(&jointStateMessage[0].header.seq, 0x00, 12*sizeof(jointStateMessage));
  ros::Subscriber jointControl_sub = n.subscribe("/robot_1/control/joint_command", 1000, jointControlCallback);
  ros::Publisher  jointControl_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1000);

  //ros::spin();

  jointStateMessage.name.resize(12);
  jointStateMessage.position.resize(12);
  jointStateMessage.velocity.resize(12);
  jointStateMessage.effort.resize(12);

 // FR_hip_joint, FR_thigh_joint, FR_calf_joint, FL_hip_joint, FL_thigh_joint, FL_calf_joint,
 // RR_hip_joint, RR_thigh_joint, RR_calf_joint, RL_hip_joint, RL_thigh_joint, RL_calf_joint

  jointStateMessage.name[0] = "FR_thigh_joint";
  jointStateMessage.name[1] = "FR_calf_joint";
  jointStateMessage.name[2] = "FL_thigh_joint";
  jointStateMessage.name[3] = "FL_calf_joint";
  jointStateMessage.name[4] = "RR_thigh_joint";
  jointStateMessage.name[5] = "RR_calf_joint";
  jointStateMessage.name[6] = "RL_thigh_joint";
  jointStateMessage.name[7] = "RL_calf_joint";
  jointStateMessage.name[8] = "FR_hip_joint";
  jointStateMessage.name[9] = "FL_hip_joint";
  jointStateMessage.name[10] = "RR_hip_joint";
  jointStateMessage.name[11] = "RL_hip_joint";

  //hip = 0.0; thigh = 0.0; calf = 0.0;

  while(ros::ok())
  {

    // hip += 0.01;
    // thigh += 0.01;
    // calf += 0.01;
    jointStateMessage.header.stamp.sec = ros::WallTime::now().toSec();
    jointStateMessage.header.stamp.nsec = ros::WallTime::now().toNSec();
    
    jointStateMessage.position[0] = thigh0;// - (3.14/2);
    jointStateMessage.position[1] = calf0;
    jointStateMessage.position[2] = thigh1;// - (3.14/2);

    jointStateMessage.position[3] = calf1;
    jointStateMessage.position[4] = thigh2;// - (3.14/2);
    jointStateMessage.position[5] = calf2;

    jointStateMessage.position[6] = thigh3; // - (3.14/2);
    jointStateMessage.position[7] = calf3;
    jointStateMessage.position[8] = hip0;

    jointStateMessage.position[9] = hip1;
    jointStateMessage.position[10] = hip2;
    jointStateMessage.position[11] = hip3;



    //jointControl_pub.publish(jointStateMessage);
    ros::spinOnce();
    loopRate.sleep();

  }

  return 0;
}
