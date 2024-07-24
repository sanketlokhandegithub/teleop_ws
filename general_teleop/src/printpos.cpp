#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "general_teleop/LegCommand.h"
#include "general_teleop/LegCommandArray.h"
#include "general_teleop/MotorCommand.h"

constexpr int hipJoint = 0;
constexpr int thighJoint = 1;
constexpr int calfJoint = 2;

constexpr int legFR = 0;
constexpr int legFL = 1;
constexpr int legRR = 2;
constexpr int legRL = 3;

general_teleop::MotorCommand motorCmd[4][3];

void getQuadSdkLegComArray(const general_teleop::LegCommandArray::ConstPtr &msg)
{
  

  for(int i=0; i<4; i++)
  {
    for (int j=0; j<3; j++)
    {
      motorCmd[i][j].pos_setpoint = msg->leg_commands[i].motor_commands[j].pos_setpoint;
      motorCmd[i][j].vel_setpoint = msg->leg_commands[i].motor_commands[j].vel_setpoint;
      motorCmd[i][j].kp = msg->leg_commands[i].motor_commands[j].kp;
      motorCmd[i][j].kd = msg->leg_commands[i].motor_commands[j].kd;
      motorCmd[i][j].torque_ff = msg->leg_commands[i].motor_commands[j].torque_ff;
      
      motorCmd[i][j].pos_component = msg->leg_commands[i].motor_commands[j].pos_component;
      motorCmd[i][j].vel_component = msg->leg_commands[i].motor_commands[j].vel_component;
      motorCmd[i][j].fb_component = msg->leg_commands[i].motor_commands[j].fb_component;
      motorCmd[i][j].fb_ratio = msg->leg_commands[i].motor_commands[j].fb_ratio;
      motorCmd[i][j].effort = msg->leg_commands[i].motor_commands[j].effort;
    }
  }  
}

float rvizPos[12];
float rvizVel[12];
float rvizEfforf[12];
void getRosToRealJointStates(const sensor_msgs::JointState::ConstPtr &msg)
{
  ROS_INFO("[]");
  for(int i=0; i<12; i++)
  {
    // rvizPos[i] = msg->position[i];
    // rvizVel[i] = msg->velocity[i];
    // rvizEfforf[i] = msg->effort[i];
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "printpos");
  ros::NodeHandle n;
  ros::Rate loopRate(100000);

  ros::Subscriber quadsdkjointControl_sub = n.subscribe("/robot_1/control/joint_command", 10000, getQuadSdkLegComArray);
  
  ros::Subscriber rivjointControl_sub = n.subscribe("/joint_states", 10000, getRosToRealJointStates);
  
  
  //ros::Publisher  jointControl_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1000);

  //ros::spin();

 // FR_hip_joint, FR_thigh_joint, FR_calf_joint, FL_hip_joint, FL_thigh_joint, FL_calf_joint,
 // RR_hip_joint, RR_thigh_joint, RR_calf_joint, RL_hip_joint, RL_thigh_joint, RL_calf_joint

    // unitreejointStateMessage.name[0] = "FR_hip_joint";
    // unitreejointStateMessage.name[1] = "FR_thigh_joint";
    // unitreejointStateMessage.name[2] = "FR_calf_joint";
    // unitreejointStateMessage.name[3] = "FL_hip_joint";
    // unitreejointStateMessage.name[4] = "FL_thigh_joint";
    // unitreejointStateMessage.name[5] = "FL_calf_joint";
    // unitreejointStateMessage.name[6] = "RR_hip_joint";
    // unitreejointStateMessage.name[7] = "RR_thigh_joint";
    // unitreejointStateMessage.name[8] = "RR_calf_joint";
    // unitreejointStateMessage.name[9] = "RL_hip_joint";
    // unitreejointStateMessage.name[10] = "RL_thigh_joint";
    // unitreejointStateMessage.name[11] = "RL_calf_joint";


  while(ros::ok())
  {

    //unitreejointStateMessage.header.stamp.sec = ros::WallTime::now().toSec();
    //unitreejointStateMessage.header.stamp.nsec = ros::WallTime::now().toNSec();
    

    //jointControl_pub.publish(unitreejointStateMessage);
   
    ROS_INFO("Front Right Leg Hip   [quadsdk] | [Rviz] : %.4f | %.4f" , motorCmd[legFR][hipJoint]);
    ROS_INFO("Front Right Leg Thigh [quadsdk] | [Rviz] : %.4f | %.4f" , motorCmd[legFR][thighJoint]);
    ROS_INFO("Front Right Leg Calf  [quadsdk] | [Rviz] : %.4f | %.4f" , motorCmd[legFR][calfJoint]);

    ros::spinOnce();
    loopRate.sleep();

  }

  return 0;
}
