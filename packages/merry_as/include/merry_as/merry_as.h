/* 
 * merry_as 
 * an action server for easy commanding of merry the baxter robot
 *
 * Copyright (c) 2016, Luc Bettaieb / Wyatt Newman
 * BSD Licensed
 */

#ifndef MERRY_AS_H
#define MERRY_AS_H

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <baxter_traj_streamer/baxter_traj_streamer.h>
#include <baxter_traj_streamer/trajAction.h>

#include <cwru_action/merry_get_can.h>

#include <baxter_core_msgs/JointCommand.h>

class MerryInterface
{
  public:
    MerryInterface(ros::NodeHandle &nh);
    virtual ~MerryInterface()

  private:
    ros::NodeHandle nh_;

    actionlib::SimpleActionServer<cwru_action::merry_get_canAction> as_;

    cwru_action::merry_get_canGoal goal_;
    cwru_action::merry_get_canResult result_
    cwru_action::merry_get_canFeedback feedback_;

    virtual void updateLeftJointAngles(const sensor_msgs::JointState& jointstate);

    virtual void doneCb(const actionlib::SimpleClientGoalState& state,
                        const baxter_traj_streamer::trajResultConstPtr& result);

    virtual void canGoalCb(const actionlib::SimpleActionServer<cwru_action::merry_get_canAction>::GoalConstPtr& goal)

  public:
    virtual Vectorq7x1 getLeftArmPose();

    virtual bool goToPose(Vectorq7x1 pose);
};

#endif  // MERRY_AS_H
