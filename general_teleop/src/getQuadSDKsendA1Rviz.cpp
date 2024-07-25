#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "general_teleop/LegCommand.h"
#include "general_teleop/LegCommandArray.h"
#include "general_teleop/MotorCommand.h"



// FR_hip_joint, FR_thigh_joint, FR_calf_joint, FL_hip_joint, FL_thigh_joint, FL_calf_joint,
// RR_hip_joint, RR_thigh_joint, RR_calf_joint, RL_hip_joint, RL_thigh_joint, RL_calf_joint
general_teleop::MotorCommand unitreemotorcmd[4][3];
constexpr int unitreehipJoint = 0;
constexpr int unitreethighJoint = 1;
constexpr int unitreecalfJoint = 2;
constexpr int unitreeLegFR = 0;
constexpr int unitreeLegFL = 1;
constexpr int unitreeLegRR = 2;
constexpr int unitreeLegRL = 3;

// quadsdk
// FL-hip 8 -> FL-thigh -> FL-calf
// RL-hip 9 -> RL-thigh -> RL-calf
// FR-hip 10 -> FR-thigh -> FR-calf
// RR-hip 11 -> RR-thigh -> RR-calf
general_teleop::MotorCommand quadsdkmotorCmd[4][3];
constexpr int quadLegFL = 0;
constexpr int quadLegRL = 1;
constexpr int quadLegFR = 2;
constexpr int quadLegRR = 3;
constexpr int quadhipJoint = 0;
constexpr int quadthighJoint = 1;
constexpr int quadcalfJoint = 2;


void map_QuadSDK_to_UnitreeJointState()
{

  unitreemotorcmd[unitreeLegFL][unitreehipJoint].pos_setpoint = quadsdkmotorCmd[quadLegFL][quadhipJoint].pos_setpoint;
  unitreemotorcmd[unitreeLegFL][unitreethighJoint].pos_setpoint = quadsdkmotorCmd[quadLegFL][quadthighJoint].pos_setpoint;
  unitreemotorcmd[unitreeLegFL][unitreecalfJoint].pos_setpoint = quadsdkmotorCmd[quadLegFL][quadcalfJoint].pos_setpoint;

  unitreemotorcmd[unitreeLegRL][unitreehipJoint].pos_setpoint = quadsdkmotorCmd[quadLegRL][quadhipJoint].pos_setpoint;
  unitreemotorcmd[unitreeLegRL][unitreethighJoint].pos_setpoint = quadsdkmotorCmd[quadLegRL][quadthighJoint].pos_setpoint;
  unitreemotorcmd[unitreeLegRL][unitreecalfJoint].pos_setpoint = quadsdkmotorCmd[quadLegRL][quadcalfJoint].pos_setpoint;

  unitreemotorcmd[unitreeLegFR][unitreehipJoint].pos_setpoint = quadsdkmotorCmd[quadLegFR][quadhipJoint].pos_setpoint;
  unitreemotorcmd[unitreeLegFR][unitreethighJoint].pos_setpoint = quadsdkmotorCmd[quadLegFR][quadthighJoint].pos_setpoint;
  unitreemotorcmd[unitreeLegFR][unitreecalfJoint].pos_setpoint = quadsdkmotorCmd[quadLegFR][quadcalfJoint].pos_setpoint;

  unitreemotorcmd[unitreeLegRR][unitreehipJoint].pos_setpoint = quadsdkmotorCmd[quadLegRR][quadhipJoint].pos_setpoint;
  unitreemotorcmd[unitreeLegRR][unitreethighJoint].pos_setpoint = quadsdkmotorCmd[quadLegRR][quadthighJoint].pos_setpoint;
  unitreemotorcmd[unitreeLegRR][unitreecalfJoint].pos_setpoint = quadsdkmotorCmd[quadLegRR][quadcalfJoint].pos_setpoint;

  unitreemotorcmd[unitreeLegFL][unitreehipJoint].vel_setpoint = quadsdkmotorCmd[quadLegFL][quadhipJoint].vel_setpoint;
  unitreemotorcmd[unitreeLegFL][unitreethighJoint].vel_setpoint = quadsdkmotorCmd[quadLegFL][quadthighJoint].vel_setpoint;
  unitreemotorcmd[unitreeLegFL][unitreecalfJoint].vel_setpoint = quadsdkmotorCmd[quadLegFL][quadcalfJoint].vel_setpoint;

  unitreemotorcmd[unitreeLegRL][unitreehipJoint].vel_setpoint = quadsdkmotorCmd[quadLegRL][quadhipJoint].vel_setpoint;
  unitreemotorcmd[unitreeLegRL][unitreethighJoint].vel_setpoint = quadsdkmotorCmd[quadLegRL][quadthighJoint].vel_setpoint;
  unitreemotorcmd[unitreeLegRL][unitreecalfJoint].vel_setpoint = quadsdkmotorCmd[quadLegRL][quadcalfJoint].vel_setpoint;

  unitreemotorcmd[unitreeLegFR][unitreehipJoint].vel_setpoint = quadsdkmotorCmd[quadLegFR][quadhipJoint].vel_setpoint;
  unitreemotorcmd[unitreeLegFR][unitreethighJoint].vel_setpoint = quadsdkmotorCmd[quadLegFR][quadthighJoint].vel_setpoint;
  unitreemotorcmd[unitreeLegFR][unitreecalfJoint].vel_setpoint = quadsdkmotorCmd[quadLegFR][quadcalfJoint].vel_setpoint;

  unitreemotorcmd[unitreeLegRR][unitreehipJoint].vel_setpoint = quadsdkmotorCmd[quadLegRR][quadhipJoint].vel_setpoint;
  unitreemotorcmd[unitreeLegRR][unitreethighJoint].vel_setpoint = quadsdkmotorCmd[quadLegRR][quadthighJoint].vel_setpoint;
  unitreemotorcmd[unitreeLegRR][unitreecalfJoint].vel_setpoint = quadsdkmotorCmd[quadLegRR][quadcalfJoint].vel_setpoint;


  unitreemotorcmd[unitreeLegFL][unitreehipJoint].effort = quadsdkmotorCmd[quadLegFL][quadhipJoint].effort;
  unitreemotorcmd[unitreeLegFL][unitreethighJoint].effort = quadsdkmotorCmd[quadLegFL][quadthighJoint].effort;
  unitreemotorcmd[unitreeLegFL][unitreecalfJoint].effort = quadsdkmotorCmd[quadLegFL][quadcalfJoint].effort;

  unitreemotorcmd[unitreeLegRL][unitreehipJoint].effort = quadsdkmotorCmd[quadLegRL][quadhipJoint].effort;
  unitreemotorcmd[unitreeLegRL][unitreethighJoint].effort = quadsdkmotorCmd[quadLegRL][quadthighJoint].effort;
  unitreemotorcmd[unitreeLegRL][unitreecalfJoint].effort = quadsdkmotorCmd[quadLegRL][quadcalfJoint].effort;

  unitreemotorcmd[unitreeLegFR][unitreehipJoint].effort = quadsdkmotorCmd[quadLegFR][quadhipJoint].effort;
  unitreemotorcmd[unitreeLegFR][unitreethighJoint].effort = quadsdkmotorCmd[quadLegFR][quadthighJoint].effort;
  unitreemotorcmd[unitreeLegFR][unitreecalfJoint].effort = quadsdkmotorCmd[quadLegFR][quadcalfJoint].effort;

  unitreemotorcmd[unitreeLegRR][unitreehipJoint].effort = quadsdkmotorCmd[quadLegRR][quadhipJoint].effort;
  unitreemotorcmd[unitreeLegRR][unitreethighJoint].effort = quadsdkmotorCmd[quadLegRR][quadthighJoint].effort;
  unitreemotorcmd[unitreeLegRR][unitreecalfJoint].effort = quadsdkmotorCmd[quadLegRR][quadcalfJoint].effort;

}

void getQuadSdkLegComArray(const general_teleop::LegCommandArray::ConstPtr &msg)
{
  for(int i=0; i<4; i++)
  {
    for (int j=0; j<3; j++)
    {
      quadsdkmotorCmd[i][j].pos_setpoint = msg->leg_commands[i].motor_commands[j].pos_setpoint;
      quadsdkmotorCmd[i][j].vel_setpoint = msg->leg_commands[i].motor_commands[j].vel_setpoint;
      quadsdkmotorCmd[i][j].kp = msg->leg_commands[i].motor_commands[j].kp;
      quadsdkmotorCmd[i][j].kd = msg->leg_commands[i].motor_commands[j].kd;
      quadsdkmotorCmd[i][j].torque_ff = msg->leg_commands[i].motor_commands[j].torque_ff;
      
      quadsdkmotorCmd[i][j].pos_component = msg->leg_commands[i].motor_commands[j].pos_component;
      quadsdkmotorCmd[i][j].vel_component = msg->leg_commands[i].motor_commands[j].vel_component;
      quadsdkmotorCmd[i][j].fb_component = msg->leg_commands[i].motor_commands[j].fb_component;
      quadsdkmotorCmd[i][j].fb_ratio = msg->leg_commands[i].motor_commands[j].fb_ratio;
      quadsdkmotorCmd[i][j].effort = msg->leg_commands[i].motor_commands[j].effort;
    }
  }  
  map_QuadSDK_to_UnitreeJointState();
}

sensor_msgs::JointState sendJointStatetoR2R;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "printpos");
  ros::NodeHandle n;
  ros::Rate loopRate(100000);

  ros::Subscriber quadsdkjointControl_sub = n.subscribe("/robot_1/control/joint_command", 10000, getQuadSdkLegComArray);
 
  sendJointStatetoR2R.name.resize(12);
  sendJointStatetoR2R.effort.resize(12);
  sendJointStatetoR2R.velocity.resize(12);
  sendJointStatetoR2R.position.resize(12);
  
  ros::Publisher  jointControl_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 10000);

  //ros::spin();

  sendJointStatetoR2R.name[0] = "FR_hip_joint";
  sendJointStatetoR2R.name[1] = "FR_thigh_joint";
  sendJointStatetoR2R.name[2] = "FR_calf_joint";
  sendJointStatetoR2R.name[3] = "FL_hip_joint";
  sendJointStatetoR2R.name[4] = "FL_thigh_joint";
  sendJointStatetoR2R.name[5] = "FL_calf_joint";
  sendJointStatetoR2R.name[6] = "RR_hip_joint";
  sendJointStatetoR2R.name[7] = "RR_thigh_joint";
  sendJointStatetoR2R.name[8] = "RR_calf_joint";
  sendJointStatetoR2R.name[9] = "RL_hip_joint";
  sendJointStatetoR2R.name[10] = "RL_thigh_joint";
  sendJointStatetoR2R.name[11] = "RL_calf_joint";



  while(ros::ok())
  {

    sendJointStatetoR2R.header.stamp.sec = ros::WallTime::now().toSec();
    sendJointStatetoR2R.header.stamp.nsec = ros::WallTime::now().toNSec();

    sendJointStatetoR2R.position[0] =  unitreemotorcmd[unitreeLegFR][unitreehipJoint].pos_setpoint ;
    sendJointStatetoR2R.position[1] =  unitreemotorcmd[unitreeLegFR][unitreethighJoint].pos_setpoint;
    sendJointStatetoR2R.position[2] =  unitreemotorcmd[unitreeLegFR][unitreecalfJoint].pos_setpoint * -1.0;
    sendJointStatetoR2R.position[3] =  unitreemotorcmd[unitreeLegFL][unitreehipJoint].pos_setpoint ;
    sendJointStatetoR2R.position[4] =  unitreemotorcmd[unitreeLegFL][unitreethighJoint].pos_setpoint;
    sendJointStatetoR2R.position[5] =  unitreemotorcmd[unitreeLegFL][unitreecalfJoint].pos_setpoint * -1.0;
    sendJointStatetoR2R.position[6] =  unitreemotorcmd[unitreeLegRR][unitreehipJoint].pos_setpoint ;
    sendJointStatetoR2R.position[7] =  unitreemotorcmd[unitreeLegRR][unitreethighJoint].pos_setpoint;
    sendJointStatetoR2R.position[8] =  unitreemotorcmd[unitreeLegRR][unitreecalfJoint].pos_setpoint * -1.0;
    sendJointStatetoR2R.position[9] =  unitreemotorcmd[unitreeLegRL][unitreehipJoint].pos_setpoint ;
    sendJointStatetoR2R.position[10] = unitreemotorcmd[unitreeLegRL][unitreethighJoint].pos_setpoint; 
    sendJointStatetoR2R.position[11] = unitreemotorcmd[unitreeLegRL][unitreecalfJoint].pos_setpoint * -1.0;   

    jointControl_pub.publish(sendJointStatetoR2R);
   
    ROS_INFO("Front Left Leg Hip  : %.4f   %.4f   %.4f" , sendJointStatetoR2R.position[0], sendJointStatetoR2R.position[1],sendJointStatetoR2R.position[2]);
    //ROS_INFO("Front Right Leg Thigh [quadsdk] | [Rviz] : %.4f | %.4f" , quadsdkmotorCmd[legFR][thighJoint].pos_setpoint, rvizPos[1]);
    //ROS_INFO("Front Right Leg Calf  [quadsdk] | [Rviz] : %.4f | %.4f" , quadsdkmotorCmd[legFR][calfJoint].pos_setpoint, rvizPos[2]);

    ros::spinOnce();
    loopRate.sleep();

  }

  return 0;
}
