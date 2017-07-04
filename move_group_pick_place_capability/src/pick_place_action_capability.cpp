/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include "pick_place_action_capability.h"
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/plan_execution/plan_with_sensing.h>
#include <moveit/move_group_pick_place_capability/capability_names.h>

#include <eigen_conversions/eigen_msg.h>

move_group::MoveGroupAugmentedPickPlaceAction::MoveGroupAugmentedPickPlaceAction()
  : MoveGroupCapability("AugmentedPickPlaceAction"), pickup_state_(IDLE)
{
}

void move_group::MoveGroupAugmentedPickPlaceAction::initialize()
{
  pick_place_.reset(new pick_place::PickPlace(context_->planning_pipeline_));
  pick_place_->displayComputedMotionPlans(true);

  if (context_->debug_)
    pick_place_->displayProcessedGrasps(true);

  // register action client for grasp manager
  grasp_manager_client_.reset(new actionlib::SimpleActionClient<grasping_msgs::GenerateGraspsAction>(GRASP_MANAGER_ACTION, true));

  // start the pickup action server
  pickup_action_server_.reset(new actionlib::SimpleActionServer<augmented_manipulation_msgs::AugmentedPickupAction>(
      root_node_handle_, PICKUP_ACTION, boost::bind(&MoveGroupAugmentedPickPlaceAction::executePickupCallback, this, _1),
      false));
  pickup_action_server_->registerPreemptCallback(boost::bind(&MoveGroupAugmentedPickPlaceAction::preemptPickupCallback, this));
  pickup_action_server_->start();

  // start the place action server
  place_action_server_.reset(new actionlib::SimpleActionServer<moveit_msgs::PlaceAction>(
      root_node_handle_, PLACE_ACTION, boost::bind(&MoveGroupAugmentedPickPlaceAction::executePlaceCallback, this, _1), false));
  place_action_server_->registerPreemptCallback(boost::bind(&MoveGroupAugmentedPickPlaceAction::preemptPlaceCallback, this));
  place_action_server_->start();
}

void move_group::MoveGroupAugmentedPickPlaceAction::startPickupExecutionCallback()
{
  setPickupState(MONITOR);
}

void move_group::MoveGroupAugmentedPickPlaceAction::startPickupLookCallback()
{
  setPickupState(LOOK);
}

void move_group::MoveGroupAugmentedPickPlaceAction::startPlaceExecutionCallback()
{
  setPlaceState(MONITOR);
}

void move_group::MoveGroupAugmentedPickPlaceAction::startPlaceLookCallback()
{
  setPlaceState(LOOK);
}

void move_group::MoveGroupAugmentedPickPlaceAction::executePickupCallback_PlanOnly(const moveit_msgs::PickupGoalConstPtr& goal,
                                                                                   moveit_msgs::PickupResult& action_res)
{
  pick_place::PickPlanPtr plan;
  try
  {
    planning_scene_monitor::LockedPlanningSceneRO ps(context_->planning_scene_monitor_);
    plan = pick_place_->planPick(ps, *goal);
  }
  catch (std::exception& ex)
  {
    ROS_ERROR_NAMED("manipulation", "Pick&place threw an exception: %s", ex.what());
  }

  if (plan)
  {
    const std::vector<pick_place::ManipulationPlanPtr>& success = plan->getSuccessfulManipulationPlans();
    if (success.empty())
    {
      action_res.error_code = plan->getErrorCode();
    }
    else
    {
      const pick_place::ManipulationPlanPtr& result = success.back();
      convertToMsg(result->trajectories_, action_res.trajectory_start, action_res.trajectory_stages);
      action_res.trajectory_descriptions.resize(result->trajectories_.size());
      for (std::size_t i = 0; i < result->trajectories_.size(); ++i)
        action_res.trajectory_descriptions[i] = result->trajectories_[i].description_;
      if (result->id_ < goal->possible_grasps.size())
        action_res.grasp = goal->possible_grasps[result->id_];
      action_res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }
  }
  else
  {
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }
}

void move_group::MoveGroupAugmentedPickPlaceAction::executePlaceCallback_PlanOnly(const moveit_msgs::PlaceGoalConstPtr& goal,
                                                                         moveit_msgs::PlaceResult& action_res)
{
  pick_place::PlacePlanPtr plan;
  try
  {
    planning_scene_monitor::LockedPlanningSceneRO ps(context_->planning_scene_monitor_);
    plan = pick_place_->planPlace(ps, *goal);
  }
  catch (std::exception& ex)
  {
    ROS_ERROR_NAMED("manipulation", "Pick&place threw an exception: %s", ex.what());
  }

  if (plan)
  {
    const std::vector<pick_place::ManipulationPlanPtr>& success = plan->getSuccessfulManipulationPlans();
    if (success.empty())
    {
      action_res.error_code = plan->getErrorCode();
    }
    else
    {
      const pick_place::ManipulationPlanPtr& result = success.back();
      convertToMsg(result->trajectories_, action_res.trajectory_start, action_res.trajectory_stages);
      action_res.trajectory_descriptions.resize(result->trajectories_.size());
      for (std::size_t i = 0; i < result->trajectories_.size(); ++i)
        action_res.trajectory_descriptions[i] = result->trajectories_[i].description_;
      if (result->id_ < goal->place_locations.size())
        action_res.place_location = goal->place_locations[result->id_];
      action_res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }
  }
  else
  {
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }
}

bool move_group::MoveGroupAugmentedPickPlaceAction::planUsingPickPlace_Pickup(const moveit_msgs::PickupGoal& goal,
                                                                              moveit_msgs::PickupResult* action_res,
                                                                     plan_execution::ExecutableMotionPlan& plan)
{
  setPickupState(PLANNING);

  planning_scene_monitor::LockedPlanningSceneRO ps(plan.planning_scene_monitor_);

  pick_place::PickPlanPtr pick_plan;
  try
  {
    pick_plan = pick_place_->planPick(plan.planning_scene_, goal);
  }
  catch (std::exception& ex)
  {
    ROS_ERROR_NAMED("manipulation", "Pick&place threw an exception: %s", ex.what());
  }

  if (pick_plan)
  {
    const std::vector<pick_place::ManipulationPlanPtr>& success = pick_plan->getSuccessfulManipulationPlans();
    if (success.empty())
    {
      plan.error_code_ = pick_plan->getErrorCode();
    }
    else
    {
      const pick_place::ManipulationPlanPtr& result = success.back();
      plan.plan_components_ = result->trajectories_;
      if (result->id_ < goal.possible_grasps.size())
        action_res->grasp = goal.possible_grasps[result->id_];
      plan.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }
  }
  else
  {
    plan.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }

  return plan.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
}

bool move_group::MoveGroupAugmentedPickPlaceAction::planUsingPickPlace_Place(const moveit_msgs::PlaceGoal& goal,
                                                                    moveit_msgs::PlaceResult* action_res,
                                                                    plan_execution::ExecutableMotionPlan& plan)
{
  setPlaceState(PLANNING);

  planning_scene_monitor::LockedPlanningSceneRO ps(plan.planning_scene_monitor_);

  pick_place::PlacePlanPtr place_plan;
  try
  {
    place_plan = pick_place_->planPlace(plan.planning_scene_, goal);
  }
  catch (std::exception& ex)
  {
    ROS_ERROR_NAMED("manipulation", "Pick&place threw an exception: %s", ex.what());
  }

  if (place_plan)
  {
    const std::vector<pick_place::ManipulationPlanPtr>& success = place_plan->getSuccessfulManipulationPlans();
    if (success.empty())
    {
      plan.error_code_ = place_plan->getErrorCode();
    }
    else
    {
      const pick_place::ManipulationPlanPtr& result = success.back();
      plan.plan_components_ = result->trajectories_;
      if (result->id_ < goal.place_locations.size())
        action_res->place_location = goal.place_locations[result->id_];
      plan.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }
  }
  else
  {
    plan.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }

  return plan.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
}

void move_group::MoveGroupAugmentedPickPlaceAction::executePickupCallback_PlanAndExecute(
    const moveit_msgs::PickupGoalConstPtr& goal,   moveit_msgs::PickupResult& action_res)
{
  plan_execution::PlanExecution::Options opt;

  opt.replan_ = goal->planning_options.replan;
  opt.replan_attempts_ = goal->planning_options.replan_attempts;
  opt.replan_delay_ = goal->planning_options.replan_delay;
  opt.before_execution_callback_ = boost::bind(&MoveGroupAugmentedPickPlaceAction::startPickupExecutionCallback, this);

  opt.plan_callback_ =
      boost::bind(&MoveGroupAugmentedPickPlaceAction::planUsingPickPlace_Pickup, this, boost::cref(*goal), &action_res, _1);
  if (goal->planning_options.look_around && context_->plan_with_sensing_)
  {
    opt.plan_callback_ = boost::bind(&plan_execution::PlanWithSensing::computePlan, context_->plan_with_sensing_.get(),
                                     _1, opt.plan_callback_, goal->planning_options.look_around_attempts,
                                     goal->planning_options.max_safe_execution_cost);
    context_->plan_with_sensing_->setBeforeLookCallback(
        boost::bind(&MoveGroupAugmentedPickPlaceAction::startPickupLookCallback, this));
  }

  plan_execution::ExecutableMotionPlan plan;
  context_->plan_execution_->planAndExecute(plan, goal->planning_options.planning_scene_diff, opt);

  convertToMsg(plan.plan_components_, action_res.trajectory_start, action_res.trajectory_stages);
  action_res.trajectory_descriptions.resize(plan.plan_components_.size());
  for (std::size_t i = 0; i < plan.plan_components_.size(); ++i)
    action_res.trajectory_descriptions[i] = plan.plan_components_[i].description_;
  action_res.error_code = plan.error_code_;
}

void move_group::MoveGroupAugmentedPickPlaceAction::executePlaceCallback_PlanAndExecute(
    const moveit_msgs::PlaceGoalConstPtr& goal, moveit_msgs::PlaceResult& action_res)
{
  plan_execution::PlanExecution::Options opt;

  opt.replan_ = goal->planning_options.replan;
  opt.replan_attempts_ = goal->planning_options.replan_attempts;
  opt.replan_delay_ = goal->planning_options.replan_delay;
  opt.before_execution_callback_ = boost::bind(&MoveGroupAugmentedPickPlaceAction::startPlaceExecutionCallback, this);
  opt.plan_callback_ =
      boost::bind(&MoveGroupAugmentedPickPlaceAction::planUsingPickPlace_Place, this, boost::cref(*goal), &action_res, _1);
  if (goal->planning_options.look_around && context_->plan_with_sensing_)
  {
    opt.plan_callback_ = boost::bind(&plan_execution::PlanWithSensing::computePlan, context_->plan_with_sensing_.get(),
                                     _1, opt.plan_callback_, goal->planning_options.look_around_attempts,
                                     goal->planning_options.max_safe_execution_cost);
    context_->plan_with_sensing_->setBeforeLookCallback(
        boost::bind(&MoveGroupAugmentedPickPlaceAction::startPlaceLookCallback, this));
  }

  plan_execution::ExecutableMotionPlan plan;
  context_->plan_execution_->planAndExecute(plan, goal->planning_options.planning_scene_diff, opt);

  convertToMsg(plan.plan_components_, action_res.trajectory_start, action_res.trajectory_stages);
  action_res.trajectory_descriptions.resize(plan.plan_components_.size());
  for (std::size_t i = 0; i < plan.plan_components_.size(); ++i)
    action_res.trajectory_descriptions[i] = plan.plan_components_[i].description_;
  action_res.error_code = plan.error_code_;
}

void move_group::MoveGroupAugmentedPickPlaceAction::executePickupCallback(const augmented_manipulation_msgs::AugmentedPickupGoalConstPtr&  augmented_goal)
{
  setPickupState(PLANNING);

  // before we start planning, ensure that we have the latest robot state received...
  context_->planning_scene_monitor_->waitForCurrentRobotState(ros::Time::now());
  context_->planning_scene_monitor_->updateFrameTransforms();


    // throw error and abort goal if object is not in planning scene
  if ( augmented_goal->grasp_config_sets.empty() )
  {
      ROS_ERROR_NAMED("manipulation", "A GraspConfigSet has to be given!");

      // fill AugmentedPickupResult
      augmented_manipulation_msgs::AugmentedPickupResult augmented_action_res;
      augmented_action_res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;

      // abort goal
      pickup_action_server_->setAborted(augmented_action_res);
      setPickupState(IDLE);

      return;
  }
  // get objects from planning scene and search for the given name
  grasping_msgs::Object object;

  std::map< std::string, moveit_msgs::CollisionObject > known_objects = planning_scene_interface_.getObjects();
  bool found = false;

  for( const auto& object_pair : known_objects)
  {
    ROS_DEBUG_NAMED("manipulation", "Checking object %s", object_pair.first.c_str());

    if(object_pair.first == augmented_goal->object_name)
    {
      ROS_INFO_NAMED("manipulation", "Found %s!", augmented_goal->object_name.c_str());

      // fill object with name, header, primitives and poses
      object.name = object_pair.second.id;
      object.header = object_pair.second.header;

      object.primitives.resize(object_pair.second.primitives.size());
      for (std::size_t i = 0; i < object_pair.second.primitives.size(); ++i)
        object.primitives[i] = object_pair.second.primitives[i];

      object.primitive_poses.resize(object_pair.second.primitive_poses.size());
      for (std::size_t i = 0; i < object_pair.second.primitive_poses.size(); ++i)
        object.primitive_poses[i] = object_pair.second.primitive_poses[i];

      found = true;
      break;
    }
  }

  // throw error and abort goal if object is not in planning scene
  if ( not found )
  {
      ROS_ERROR_NAMED("manipulation", "Object %s not found in current PlanningScene!", augmented_goal->object_name.c_str());

      // fill AugmentedPickupResult
      augmented_manipulation_msgs::AugmentedPickupResult augmented_action_res;
      augmented_action_res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_OBJECT_NAME;

      // abort goal
      pickup_action_server_->setAborted(augmented_action_res);
      setPickupState(IDLE);

      return;
  }

  // create PickupGoal based on AugmentedPickupGoal and generated grasps
  moveit_msgs::PickupGoalConstPtr goal;
  moveit_msgs::PickupGoal copy;

  // transfer options relevant for all groups to PickupGoal
  copy.allowed_planning_time = augmented_goal->allowed_planning_time;
  copy.planning_options = augmented_goal->planning_options;

  moveit_msgs::PickupResult action_res;
  std::string response;
  std::vector<moveit_msgs::Grasp> generated_grasps;


  // try planning for every group / config given
  for( const auto& grasp_config_set : augmented_goal->grasp_config_sets)
  {
      ROS_DEBUG_NAMED("manipulation", "Generating grasps for group %s with config %s",
                      grasp_config_set.group_name.c_str(), grasp_config_set.config_name.c_str());

      // build & send goal for generating grasps
      grasping_msgs::GenerateGraspsGoal grasp_goal;

      grasp_goal.object = object;
      grasp_goal.config_name = grasp_config_set.config_name;

      grasp_manager_client_.get()->sendGoal(grasp_goal);
      grasp_manager_client_.get()->waitForResult();

      // get grasps and store them in the goal
      grasping_msgs::GenerateGraspsResultConstPtr grasp_result = grasp_manager_client_.get()->getResult();

      // store generated grasps in vector
      generated_grasps.insert(generated_grasps.end(), grasp_result->grasps.begin(), grasp_result->grasps.end());

      // pass generated grasps down to goal
      for (std::size_t i = 0; i < grasp_result->grasps.size(); ++i)
          copy.possible_grasps[i] = grasp_result->grasps[i];

      // get end effector name from param server
      nh_.getParam("/grasp_gen_config/" + grasp_config_set.config_name + "/eef_name", copy.end_effector);

    // set groupname according to config
    copy.group_name = grasp_config_set.group_name;
    // set object name according to object name
    copy.target_name = object.name;

      goal.reset(new moveit_msgs::PickupGoal(copy));

      // trigger goal planning & execution
      if (goal->planning_options.plan_only || !context_->allow_trajectory_execution_)
      {
          if (!goal->planning_options.plan_only)
              ROS_WARN_NAMED("manipulation", "This instance of MoveGroup is not allowed to execute trajectories but the pick "
                      "goal request has plan_only set to false. Only a motion plan will be computed "
                      "anyway.");

          executePickupCallback_PlanOnly(goal, action_res);
      }
      else
          executePickupCallback_PlanAndExecute(goal, action_res);


      bool planned_trajectory_empty = action_res.trajectory_stages.empty();
      response = getActionResultString(action_res.error_code, planned_trajectory_empty, goal->planning_options.plan_only);

      // if get a success, we're done, other groups are ignored
      if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
          break;
  }

  // fill AugmentedPickupResult
  augmented_manipulation_msgs::AugmentedPickupResult augmented_action_res;
  augmented_action_res.error_code = action_res.error_code;
  augmented_action_res.grasp = action_res.grasp;

    if (augmented_action_res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    pickup_action_server_->setSucceeded(augmented_action_res, response);
  else
  {
    if (augmented_action_res.error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
      pickup_action_server_->setPreempted(augmented_action_res, response);
    else
      pickup_action_server_->setAborted(augmented_action_res, response);
  }

  setPickupState(IDLE);
}

void move_group::MoveGroupAugmentedPickPlaceAction::executePlaceCallback(const moveit_msgs::PlaceGoalConstPtr& goal)
{
  setPlaceState(PLANNING);

  // before we start planning, ensure that we have the latest robot state received...
  context_->planning_scene_monitor_->waitForCurrentRobotState(ros::Time::now());
  context_->planning_scene_monitor_->updateFrameTransforms();

  moveit_msgs::PlaceResult action_res;

  if (goal->planning_options.plan_only || !context_->allow_trajectory_execution_)
  {
    if (!goal->planning_options.plan_only)
      ROS_WARN_NAMED("manipulation", "This instance of MoveGroup is not allowed to execute trajectories but the place "
                                     "goal request has plan_only set to false. Only a motion plan will be computed "
                                     "anyway.");
    executePlaceCallback_PlanOnly(goal, action_res);
  }
  else
    executePlaceCallback_PlanAndExecute(goal, action_res);

  bool planned_trajectory_empty = action_res.trajectory_stages.empty();
  std::string response =
      getActionResultString(action_res.error_code, planned_trajectory_empty, goal->planning_options.plan_only);
  if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    place_action_server_->setSucceeded(action_res, response);
  else
  {
    if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
      place_action_server_->setPreempted(action_res, response);
    else
      place_action_server_->setAborted(action_res, response);
  }

  setPlaceState(IDLE);
}

void move_group::MoveGroupAugmentedPickPlaceAction::preemptPickupCallback()
{
}

void move_group::MoveGroupAugmentedPickPlaceAction::preemptPlaceCallback()
{
}

void move_group::MoveGroupAugmentedPickPlaceAction::setPickupState(MoveGroupState state)
{
  pickup_state_ = state;
  pickup_feedback_.state = stateToStr(state);
  pickup_action_server_->publishFeedback(pickup_feedback_);
}

void move_group::MoveGroupAugmentedPickPlaceAction::setPlaceState(MoveGroupState state)
{
  place_state_ = state;
  place_feedback_.state = stateToStr(state);
  place_action_server_->publishFeedback(place_feedback_);
}

void move_group::MoveGroupAugmentedPickPlaceAction::fillGrasps(moveit_msgs::PickupGoal& goal)
{
  planning_scene_monitor::LockedPlanningSceneRO lscene(context_->planning_scene_monitor_);

  ROS_DEBUG_NAMED("manipulation", "Using default grasp poses");
  goal.minimize_object_distance = true;

  // add a number of default grasp points
  // \todo add more!
  moveit_msgs::Grasp g;
  g.grasp_pose.header.frame_id = goal.target_name;
  g.grasp_pose.pose.position.x = -0.2;
  g.grasp_pose.pose.position.y = 0.0;
  g.grasp_pose.pose.position.z = 0.0;
  g.grasp_pose.pose.orientation.x = 0.0;
  g.grasp_pose.pose.orientation.y = 0.0;
  g.grasp_pose.pose.orientation.z = 0.0;
  g.grasp_pose.pose.orientation.w = 1.0;

  g.pre_grasp_approach.direction.header.frame_id = lscene->getPlanningFrame();
  g.pre_grasp_approach.direction.vector.x = 1.0;
  g.pre_grasp_approach.min_distance = 0.1;
  g.pre_grasp_approach.desired_distance = 0.2;

  g.post_grasp_retreat.direction.header.frame_id = lscene->getPlanningFrame();
  g.post_grasp_retreat.direction.vector.z = 1.0;
  g.post_grasp_retreat.min_distance = 0.1;
  g.post_grasp_retreat.desired_distance = 0.2;

  if (lscene->getRobotModel()->hasEndEffector(goal.end_effector))
  {
    g.pre_grasp_posture.joint_names = lscene->getRobotModel()->getEndEffector(goal.end_effector)->getJointModelNames();
    g.pre_grasp_posture.points.resize(1);
    g.pre_grasp_posture.points[0].positions.resize(g.pre_grasp_posture.joint_names.size(),
                                                   std::numeric_limits<double>::max());

    g.grasp_posture.joint_names = g.pre_grasp_posture.joint_names;
    g.grasp_posture.points.resize(1);
    g.grasp_posture.points[0].positions.resize(g.grasp_posture.joint_names.size(), -std::numeric_limits<double>::max());
  }
  goal.possible_grasps.push_back(g);
}

#include <class_loader/class_loader.h>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupAugmentedPickPlaceAction, move_group::MoveGroupCapability)
