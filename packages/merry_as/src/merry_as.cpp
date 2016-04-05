/* 
 * merry_as 
 * an action server for easy commanding of merry the baxter robot
 *
 * Copyright (c) 2016, Luc Bettaieb / Wyatt Newman
 * BSD Licensed
 */

#include "merry_as/merry_as.h"

#include <vector>

baxter_core_msgs::JointCommand left_cmd;
double leftJointAngles [7];

MerryInterface::MerryInterface(ros::NodeHandle &nh) :
  as_(nh_, "merry_get_can", boost::bind(&merry_as::canGoalCb, this, _1), false)
{
  nh_ = nh;

  as_.start();

  left_cmd.mode = baxter_core_msgs::JointCommand::RAW_POSITION_MODE;
  left_cmd.names.push_back("left_s0");
  left_cmd.names.push_back("left_s1");
  left_cmd.names.push_back("left_e0");
  left_cmd.names.push_back("left_e1");
  left_cmd.names.push_back("left_w0");
  left_cmd.names.push_back("left_w1");
  left_cmd.names.push_back("left_w2");

  left_cmd.command.resize(7, 0.0);
}

MerryInterface::~MerryInterface()
{
}

void MerryInterface::canGoalCb(const actionlib::SimpleActionServer<cwru_action::merry_get_canAction>::GoalConstPtr& goal)
{
  // Here do interesting stuff.  Do IK for Merry's arm and find joint angles.  Make a new Vectorq7x1 and send the joint space command to the AS.
  
}

void MerryInterface::doneCb(const actionlib::SimpleClientGoalState& state,
                             const baxter_traj_streamer::trajResultConstPtr& result)
{
  ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
  ROS_INFO("got return val = %d; traj_id = %d", result->return_val, result->traj_id);
}

Vectorq7x1 MerryInterface::getLeftArmPose()
{
  ros::spinOnce();

  Vectorq7x1 pose;

  pose(2, 0) = leftJointAngles[0];
  pose(3, 0) = leftJointAngles[1];
  pose(0, 0) = leftJointAngles[2];
  pose(1, 0) = leftJointAngles[3];
  pose(4, 0) = leftJointAngles[4];
  pose(5, 0) = leftJointAngles[5];
  pose(6, 0) = leftJointAngles[6];

  return pose;
}


void MerryInterface::updateLeftJointAngles(const sensor_msgs::JointState& jointstate)
{
  for (uint i = 0; i < 7; i++)
  {
    leftJointAngles[i] = jointstate.position.at(i+2);
  }
}


bool MerryInterface::goToPose(Vectorq7x1 pose)
{
  uint g_count = 0;
  uint ans;
  Eigen::VectorXd q_in_vecxd;

  Vectorq7x1 q_vec_left_arm;

  std::vector<Eigen::VectorXd> des_path;
  des_path.clear();

  trajectory_msgs::JointTrajectory des_trajectory;

  ROS_DEBUG("Instantiating a traj streamer");
  Baxter_traj_streamer ts(&nh_);

  ROS_DEBUG("Warming up callbacks");  // Is this necessary?  Test with and without it.
  for (uint i = 0; i < 100; i++)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  ROS_DEBUG("Getting current pose.");

  for (uint i = 0; i < 7; i++)
  {
    q_vec_left_arm = getLeftArmPose();  // NEED TO MAKE WORK FOR RIGHT ARM
  }

  q_in_vecxd = q_vec_left_arm;
  des_path.push_back(q_in_vecxd);

  q_in_vecxd = pose;
  des_path.push_back(q_in_vecxd);

  ts.stuff_left_trajectory(des_path, des_trajectory);

  baxter_traj_streamer::trajGoal goal;
  goal.trajectory = des_trajectory;

  actionlib::SimpleActionClient<baxter_traj_streamer::trajAction> action_client("trajActionServer", true);

  ROS_DEBUG("Waiting for server: ");

  if (!action_client.waitForServer(ros::Duration(5.0)))
  {
    ROS_WARN("Could not connect to server.");
    return false;
  }

  ROS_DEBUG("Connected to action server");

  g_count++;
  goal.traj_id = g_count;

  ROS_DEBUG("Sending traj_id %d", g_count);

  action_client.sendGoal(goal, boost::bind(&BaxterInterface::doneCb, this, _1, _2));

  if (!action_client.waitForResult(ros::Duration(5.0)))
  {
    ROS_WARN("Giving up waiting on result for traj_id: %d", g_count);
    return false;
  }
  else
  {
    ROS_DEBUG("Finished before timeout!");
  }

  return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "merry_as");
  ROS_INFO("Instantiating a MerryInterface...");

  MerryInterface merry;

  ROS_INFO("...done!");

  ros::spin();
}
