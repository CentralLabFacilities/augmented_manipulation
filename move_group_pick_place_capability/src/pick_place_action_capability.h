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

#ifndef MOVEIT_MOVE_GROUP_PICK_PLACE_ACTION_CAPABILITY_
#define MOVEIT_MOVE_GROUP_PICK_PLACE_ACTION_CAPABILITY_

#include <grasping_msgs/Object.h>
#include <grasping_msgs/GenerateGraspsAction.h>
#include <moveit/move_group/move_group_capability.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/pick_place/pick_place.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>
#include <augmented_manipulation_msgs/AugmentedPickupAction.h>
#include <augmented_manipulation_msgs/AugmentedPickupQuery.h>
#include <std_msgs/Int32.h>

#include <memory>

namespace move_group
{
class MoveGroupAugmentedPickPlaceAction : public MoveGroupCapability
{
public:
    MoveGroupAugmentedPickPlaceAction();
  virtual void initialize();

private:
  void queryPickupCallback(const augmented_manipulation_msgs::AugmentedPickupQueryConstPtr& aq);
  void executePickupCallback(const augmented_manipulation_msgs::AugmentedPickupGoalConstPtr& augmented_goal);
  void executePlaceCallback(const moveit_msgs::PlaceGoalConstPtr& goal);

  void executePickupCallback_PlanOnly(const moveit_msgs::PickupGoalConstPtr& goal,
                                      moveit_msgs::PickupResult& action_res);
  void executePickupCallback_PlanAndExecute(const moveit_msgs::PickupGoalConstPtr& goal,
                                            moveit_msgs::PickupResult& action_res);

  void executePlaceCallback_PlanOnly(const moveit_msgs::PlaceGoalConstPtr& goal, moveit_msgs::PlaceResult& action_res);
  void executePlaceCallback_PlanAndExecute(const moveit_msgs::PlaceGoalConstPtr& goal,
                                           moveit_msgs::PlaceResult& action_res);

  bool planUsingPickPlace_Pickup(const moveit_msgs::PickupGoal& goal, moveit_msgs::PickupResult* action_res,
                                 plan_execution::ExecutableMotionPlan& plan);
  bool planUsingPickPlace_Place(const moveit_msgs::PlaceGoal& goal, moveit_msgs::PlaceResult* action_res,
                                plan_execution::ExecutableMotionPlan& plan);

  void preemptPickupCallback();
  void preemptPlaceCallback();

  void startPickupLookCallback();
  void startPickupExecutionCallback();

  void startPlaceLookCallback();
  void startPlaceExecutionCallback();

  void setPickupState(MoveGroupState state);
  void setPlaceState(MoveGroupState state);

  void fillGrasps(moveit_msgs::PickupGoal& goal);

  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

  pick_place::PickPlacePtr pick_place_;

  std::unique_ptr<actionlib::SimpleActionServer<augmented_manipulation_msgs::AugmentedPickupAction> > pickup_action_server_;
    augmented_manipulation_msgs::AugmentedPickupFeedback pickup_feedback_;

  std::unique_ptr<actionlib::SimpleActionServer<moveit_msgs::PlaceAction> > place_action_server_;
  moveit_msgs::PlaceFeedback place_feedback_;

  std::unique_ptr<actionlib::SimpleActionClient<grasping_msgs::GenerateGraspsAction> > grasp_provider_client_;

  std::unique_ptr<moveit_msgs::AttachedCollisionObject> diff_attached_object_;

  MoveGroupState pickup_state_;
  MoveGroupState place_state_;

  ros::ServiceClient grasp_planning_service_;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  const std::string GRASP_PROVIDER_ACTION = "grasp_provider";    // name of 'grasp_provider' action
  
  ros::AsyncSpinner spinner;  // needed for the subscriber to get updates of the planning scene
};
}

#endif
