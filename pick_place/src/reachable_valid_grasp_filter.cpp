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
*   * Neither the name of the Willow Garage nor the names of its
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

#include <moveit/pick_place/reachable_valid_grasp_filter.h>
#include <moveit/kinematic_constraints/utils.h>
#include <ros/console.h>

namespace pick_place
{

ReachableAndValidGraspFilter::ReachableAndValidGraspFilter(const Options &opt, 
                                                           const planning_scene::PlanningSceneConstPtr &scene,
                                                           const constraint_samplers::ConstraintSamplerManagerPtr &constraints_sampler_manager,
                                                           unsigned int nthreads) :
  ManipulationStage(nthreads),
  opt_(opt),
  planning_scene_(scene),
  constraints_sampler_manager_(constraints_sampler_manager)
{
  name_ = "reachable & valid grasp filter";
}

bool ReachableAndValidGraspFilter::isStateCollisionFree(const sensor_msgs::JointState *pre_grasp_posture, 
                                                        kinematic_state::JointStateGroup *joint_state_group,
                                                        const std::vector<double> &joint_group_variable_values) const
{
  joint_state_group->setVariableValues(joint_group_variable_values);  
  // apply pre-grasp pose for the end effector (we always apply it here since it could be the case the sampler changes this posture)
  joint_state_group->getKinematicState()->setStateValues(*pre_grasp_posture);
  return !planning_scene_->isStateColliding(*joint_state_group->getKinematicState(), joint_state_group->getName());
}

bool ReachableAndValidGraspFilter::evaluate(unsigned int thread_id, const ManipulationPlanPtr &plan) const
{
  geometry_msgs::PoseStamped pose;
  pose.header = plan->grasp_.header;
  pose.pose = plan->grasp_.grasp_pose;
  
  // convert the pose we want to reach to a set of constraints
  plan->goal_constraints_ = kinematic_constraints::constructGoalConstraints(opt_.ik_link_, pose,
                                                                            opt_.tolerance_position_xyz_,
                                                                            opt_.tolerance_rotation_xyz_);
  
  // construct a sampler for the specified constraints; this can end up calling just IK, but it is more general
  // and allows for robot-specific samplers, producing samples that also change the base position if needed, etc
  plan->goal_sampler_ = constraints_sampler_manager_->selectSampler(planning_scene_, plan->planning_group_, plan->goal_constraints_);
  if (plan->goal_sampler_)
  {
    plan->goal_sampler_->setStateValidityCallback(boost::bind(&ReachableAndValidGraspFilter::isStateCollisionFree, this, &plan->grasp_.pre_grasp_posture, _1, _2));

    // initialize with scene state 
    plan->token_goal_state_.reset(new kinematic_state::KinematicState(planning_scene_->getCurrentState()));
    
    if (plan->goal_sampler_->sample(plan->token_goal_state_->getJointStateGroup(plan->planning_group_),
                                    *plan->token_goal_state_,
                                    planning_scene_->getKinematicModel()->getJointModelGroup(plan->planning_group_)->getDefaultIKAttempts()))
    {
      return true;
    }
  }
  else
    ROS_ERROR_THROTTLE(1, "No sampler was constructed");
  return false;
}

}
