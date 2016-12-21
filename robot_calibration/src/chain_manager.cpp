/*
 * Copyright (C) 2014-2015 Fetch Robotics Inc.
 * Copyright (C) 2013-2014 Unbounded Robotics Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Author: Michael Ferguson

#include <robot_calibration/capture/chain_manager.h>

namespace robot_calibration
{

ChainManager::ChainManager(ros::NodeHandle& nh, double wait_time)
{
  nh.param<std::string>("moveit/planning_group", planning_group, "arm_group");
  nh.param<std::vector<std::string>>("moveit/joints", joint_names, std::vector<std::string>());

  move_group_.reset(new MoveGroupClient("move_group", true));
  if (!move_group_->waitForServer(ros::Duration(wait_time)))
  {
    ROS_WARN("Failed to connect to move_group");
  }

  // Parameter to set velocity scaling factor for move_group
  nh.param<double>("velocity_factor", velocity_factor_, 1.0);

  subscriber_ = nh.subscribe("/joint_states", 10, &ChainManager::stateCallback, this);
}

void ChainManager::stateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  if (msg->name.size() != msg->position.size())
  {
    ROS_ERROR("JointState Error: name array is not same size as position array.");
    return;
  }

  if (msg->position.size() != msg->velocity.size())
  {
    ROS_ERROR("JointState Error: position array is not same size as velocity array.");
    return;
  }

  boost::mutex::scoped_lock lock(state_mutex_);
  // Update each joint based on message
  for (size_t msg_j = 0; msg_j < msg->name.size(); msg_j++)
  {
    size_t state_j;
    for (state_j = 0; state_j < state_.name.size(); state_j++)
    {
      if (state_.name[state_j] == msg->name[msg_j])
      {
        state_.position[state_j] = msg->position[msg_j];
        state_.velocity[state_j] = msg->velocity[msg_j];
        break;
      }
    }
    if (state_j == state_.name.size())
    {
      // New joint
      state_.name.push_back(msg->name[msg_j]);
      state_.position.push_back(msg->position[msg_j]);
      state_.velocity.push_back(msg->velocity[msg_j]);
    }
  }
}

bool ChainManager::getState(sensor_msgs::JointState* state)
{
  boost::mutex::scoped_lock lock(state_mutex_);
  *state = state_;
  return true;  // TODO: this should actually return whether state is valid
}

double ChainManager::getPosition(const sensor_msgs::JointState& state, std::string joint_name) {
  for (unsigned int i = 0; i < state.name.size(); i++) {
    if (state.name[i] == joint_name) {
      return state.position[i];
    }
  }
  // joint not found
  ROS_WARN_STREAM("Moveit joint " << joint_name << " not found in commanded joint state. Check your bag file.");
  return 0.0;
}


bool ChainManager::moveToState(const sensor_msgs::JointState& state)
{
  // Call MoveIt
  moveit_msgs::MoveGroupGoal moveit_goal;
  moveit_goal.request.group_name = planning_group;
  moveit_goal.request.num_planning_attempts = 1;
  moveit_goal.request.allowed_planning_time = 5.0;

  moveit_msgs::Constraints c1;
  c1.joint_constraints.resize(joint_names.size());
  for (size_t c = 0; c < joint_names.size(); c++)
  {
    c1.joint_constraints[c].joint_name = joint_names[c];
    c1.joint_constraints[c].position = getPosition(state, joint_names[c]);
    c1.joint_constraints[c].tolerance_above = 0.01;
    c1.joint_constraints[c].tolerance_below = 0.01;
    c1.joint_constraints[c].weight = 1.0;
  }
  moveit_goal.request.goal_constraints.push_back(c1);

  // Reduce speed
  moveit_goal.request.max_velocity_scaling_factor = velocity_factor_;

  // All diffs
  moveit_goal.request.start_state.is_diff = true;
  moveit_goal.planning_options.planning_scene_diff.is_diff = true;
  moveit_goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

  moveit_goal.planning_options.plan_only = false;

  move_group_->sendGoal(moveit_goal);
  move_group_->waitForResult();
  MoveGroupResultPtr result = move_group_->getResult();
  if (result->error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_ERROR_STREAM("Moveit planning failed with error code: " << result->error_code.val);
    return false;
  }

  return true;
}

bool ChainManager::waitToSettle()
{
  //TODO: Rewrite this
  sensor_msgs::JointState state;

  // TODO: timeout?
  while (true)
  {
    getState(&state);
    bool settled = true;

    // For each joint in state message
    for (size_t j = 0; j < state.name.size(); ++j)
    {
      // Is this joint even a concern?
      if (fabs(state.velocity[j]) < 1)
        continue;

      for (size_t k = 0; k < joint_names.size(); ++k)
      {
        if (joint_names[k] == state.name[j])
        {
          settled = false;
          break;
        }
      }

      // If at least one joint is not settled, break out of this for loop
      if (!settled)
        break;
    }

    // If all joints are settled, break out of while loop
    if (settled)
      break;

    ros::spinOnce();
  }

  return true;
}

}  // namespace robot_calibration
