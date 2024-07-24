//
// Created by sanket on 10/10/22.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "general_teleop/LegCommand.h"
#include "general_teleop/LegCommandArray.h"
#include "general_teleop/MotorCommand.h"


float hip[3];
float thigh[3];
float calf[3];

// void rivzjointControlCallback(const sensor_msgs::JointState::ConstPtr &msg)
// {

// ROS_INFO("Position 4 rviz: FR: [%.2f] [%.2f]  [%.2f] FL: [%.2f]  [%.2f] [%.2f] RR: [%.2f] [%.2f]  [%.2f] RL: [%.2f]  [%.2f] [%.2f] ",
//                                             msg->position[0],
//                                             msg->position[1],
//                                             msg->position[2]*-1.0,
//                                             msg->position[3],
//                                             msg->position[4],
//                                             msg->position[5]*-1.0,
//                                             msg->position[6],
//                                             msg->position[7],
//                                             msg->position[8]*-1.0,
//                                             msg->position[9],
//                                             msg->position[10],
//                                             msg->position[11]*-1.0
//                                               );
// }


void jointControlCallback(const general_teleop::LegCommandArray::ConstPtr &msg)
{
//   ROS_INFO("Position 4 legs: FL: [%0.2f] [%0.2f] [%0.2f] FR: [%0.2f] [%0.2f] [%0.2f] RL: [%0.2f] [%0.2f] [%0.2f] RR: [%0.2f] [%0.2f] [%0.2f]",
//                                           msg->leg_commands[0].motor_commands[0].pos_setpoint, 
//                                           msg->leg_commands[0].motor_commands[1].pos_setpoint, 
//                                           msg->leg_commands[0].motor_commands[2].pos_setpoint, 
                                          
//                                           msg->leg_commands[1].motor_commands[0].pos_setpoint,
//                                           msg->leg_commands[1].motor_commands[1].pos_setpoint,
//                                           msg->leg_commands[1].motor_commands[2].pos_setpoint,
                                          

//                                           msg->leg_commands[2].motor_commands[0].pos_setpoint,
//                                           msg->leg_commands[2].motor_commands[1].pos_setpoint,
//                                           msg->leg_commands[2].motor_commands[2].pos_setpoint,
                                                                        
//                                           msg->leg_commands[3].motor_commands[0].pos_setpoint,
//                                           msg->leg_commands[3].motor_commands[1].pos_setpoint,
//                                           msg->leg_commands[3].motor_commands[2].pos_setpoint
//                                           );

    hip[0]   = msg->leg_commands[0].motor_commands[0].pos_setpoint;
    thigh[0] = msg->leg_commands[0].motor_commands[1].pos_setpoint;
    calf[0]  = msg->leg_commands[0].motor_commands[2].pos_setpoint * -1.0;

    hip[1]   = msg->leg_commands[1].motor_commands[0].pos_setpoint;
    thigh[1] = msg->leg_commands[1].motor_commands[1].pos_setpoint;
    calf[1]  = msg->leg_commands[1].motor_commands[2].pos_setpoint * -1.0;

    hip[2]   = msg->leg_commands[2].motor_commands[0].pos_setpoint;
    thigh[2] = msg->leg_commands[2].motor_commands[1].pos_setpoint;
    calf[2]  = msg->leg_commands[2].motor_commands[2].pos_setpoint * -1.0;

    hip[3]   = msg->leg_commands[3].motor_commands[0].pos_setpoint;
    thigh[3] = msg->leg_commands[3].motor_commands[1].pos_setpoint;
    calf[3]  = msg->leg_commands[3].motor_commands[2].pos_setpoint * -1.0;

  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "testpos");
  ros::NodeHandle n;
  ros::Rate loopRate(100000);

sensor_msgs::JointState unitreejointStateMessage;

general_teleop::LegCommandArray quadLedCommandArray;


  ros::Subscriber jointControl_sub = n.subscribe("/robot_1/control/joint_command", 10000, jointControlCallback);
  
  //ros::Subscriber rivjointControl_sub = n.subscribe("/joint_states", 1000, rivzjointControlCallback);
  
  
  ros::Publisher  jointControl_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1000);

  //ros::spin();

  unitreejointStateMessage.name.resize(12);
  unitreejointStateMessage.position.resize(12);
  unitreejointStateMessage.velocity.resize(12);
  unitreejointStateMessage.effort.resize(12);

 // FR_hip_joint, FR_thigh_joint, FR_calf_joint, FL_hip_joint, FL_thigh_joint, FL_calf_joint,
 // RR_hip_joint, RR_thigh_joint, RR_calf_joint, RL_hip_joint, RL_thigh_joint, RL_calf_joint

//   jointStateMessage.name[0] = "FR_thigh_joint";
//   jointStateMessage.name[1] = "FR_calf_joint";
//   jointStateMessage.name[2] = "FL_thigh_joint";
//   jointStateMessage.name[3] = "FL_calf_joint";
//   jointStateMessage.name[4] = "RR_thigh_joint";
//   jointStateMessage.name[5] = "RR_calf_joint";
//   jointStateMessage.name[6] = "RL_thigh_joint";
//   jointStateMessage.name[7] = "RL_calf_joint";
//   jointStateMessage.name[8] = "FR_hip_joint";
//   jointStateMessage.name[9] = "FL_hip_joint";
//   jointStateMessage.name[10] = "RR_hip_joint";
//   jointStateMessage.name[11] = "RL_hip_joint";

    unitreejointStateMessage.name[0] = "FR_hip_joint";
    unitreejointStateMessage.name[1] = "FR_thigh_joint";
    unitreejointStateMessage.name[2] = "FR_calf_joint";
    unitreejointStateMessage.name[3] = "FL_hip_joint";
    unitreejointStateMessage.name[4] = "FL_thigh_joint";
    unitreejointStateMessage.name[5] = "FL_calf_joint";
    unitreejointStateMessage.name[6] = "RR_hip_joint";
    unitreejointStateMessage.name[7] = "RR_thigh_joint";
    unitreejointStateMessage.name[8] = "RR_calf_joint";
    unitreejointStateMessage.name[9] = "RL_hip_joint";
    unitreejointStateMessage.name[10] = "RL_thigh_joint";
    unitreejointStateMessage.name[11] = "RL_calf_joint";


  while(ros::ok())
  {

    unitreejointStateMessage.header.stamp.sec = ros::WallTime::now().toSec();
    unitreejointStateMessage.header.stamp.nsec = ros::WallTime::now().toNSec();
    
    unitreejointStateMessage.position[0] = hip[0];// - (3.14/2);
    unitreejointStateMessage.position[1] = thigh[0];
    unitreejointStateMessage.position[2] = calf[0];// - (3.14/2);

    unitreejointStateMessage.position[3] = hip[1];
    unitreejointStateMessage.position[4] = thigh[1];// - (3.14/2);
    unitreejointStateMessage.position[5] = calf[1];

    unitreejointStateMessage.position[6] = hip[2]; // - (3.14/2);
    unitreejointStateMessage.position[7] = thigh[2];
    unitreejointStateMessage.position[8] = calf[2];

    unitreejointStateMessage.position[9] = hip[3];
    unitreejointStateMessage.position[10] = thigh[3];
    unitreejointStateMessage.position[11] = calf[3];



    jointControl_pub.publish(unitreejointStateMessage);
    ros::spinOnce();
    loopRate.sleep();

  }

  return 0;
}
