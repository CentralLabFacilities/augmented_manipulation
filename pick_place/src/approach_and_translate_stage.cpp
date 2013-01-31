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

#include <moveit/pick_place/pick_place.h>
#include <moveit/pick_place/approach_and_translate_stage.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <eigen_conversions/eigen_msg.h>
#include <ros/console.h>

namespace pick_place
{

ApproachAndTranslateStage::ApproachAndTranslateStage(const planning_scene::PlanningSceneConstPtr &pre_grasp_scene,
                                                     const planning_scene::PlanningSceneConstPtr &post_grasp_scene,
                                                     const collision_detection::AllowedCollisionMatrixConstPtr &collision_matrix) :
  ManipulationStage("approach & translate"),
  pre_grasp_planning_scene_(pre_grasp_scene),
  post_grasp_planning_scene_(post_grasp_scene),
  collision_matrix_(collision_matrix),
  max_goal_count_(5),
  max_fail_(3),
  max_step_(0.02),
  jump_factor_(2.0)
{
}

namespace
{

bool isStateCollisionFree(const planning_scene::PlanningScene *planning_scene, 
                          const collision_detection::AllowedCollisionMatrix *collision_matrix,
                          const sensor_msgs::JointState *grasp_posture, 
                          kinematic_state::JointStateGroup *joint_state_group,
                          const std::vector<double> &joint_group_variable_values)
{
  joint_state_group->setVariableValues(joint_group_variable_values);
  // apply the grasp posture for the end effector (we always apply it here since it could be the case the sampler changes this posture)
  joint_state_group->getKinematicState()->setStateValues(*grasp_posture);
  return !planning_scene->isStateColliding(*joint_state_group->getKinematicState(), joint_state_group->getName()) && 
    planning_scene->isStateFeasible(*joint_state_group->getKinematicState());
}

bool samplePossibleGoalStates(const ManipulationPlanPtr &plan, const kinematic_state::KinematicState &reference_state, double min_distance, unsigned int attempts) 
{
  // initialize with scene state 
  kinematic_state::KinematicStatePtr token_state(new kinematic_state::KinematicState(reference_state));
  kinematic_state::JointStateGroup *jsg = token_state->getJointStateGroup(plan->planning_group_);
  for (unsigned int j = 0 ; j < attempts ; ++j)
  {
    double min_d = std::numeric_limits<double>::infinity();
    if (plan->goal_sampler_->sample(jsg, *token_state, plan->sampling_attempts_))
    {
      for (std::size_t i = 0 ; i < plan->possible_goal_states_.size() ; ++i)
      {
        double d = plan->possible_goal_states_[i]->getJointStateGroup(plan->planning_group_)->distance(jsg);
        if (d < min_d)
          min_d = d;
      }
      if (min_d >= min_distance)
      {
        plan->possible_goal_states_.push_back(token_state);
        return true;
      }
    }
  }
  return false;
}

void addGraspTrajectory(const ManipulationPlanPtr &plan, const sensor_msgs::JointState &grasp_posture, const std::string &name) 
{
  if (!grasp_posture.name.empty())
  {
    kinematic_state::KinematicStatePtr state(new kinematic_state::KinematicState(plan->trajectories_.back()->getLastWayPoint()));
    state->setStateValues(grasp_posture);
    kinematic_trajectory::KinematicTrajectoryPtr traj(new kinematic_trajectory::KinematicTrajectory(state->getKinematicModel(), plan->end_effector_group_));
    traj->addWayPoint(state, PickPlace::DEFAULT_GRASP_POSTURE_COMPLETION_DURATION);
    plan->trajectories_.push_back(traj);
    plan->trajectory_descriptions_.push_back(name);
  }
}

}

bool ApproachAndTranslateStage::evaluate(const ManipulationPlanPtr &plan) const
{
  // compute what the maximum distance reported between any two states in the planning group could be
  double min_distance = 0.0;
  const kinematic_model::JointModelGroup *jmg = pre_grasp_planning_scene_->getKinematicModel()->getJointModelGroup(plan->planning_group_);
  const std::vector<const kinematic_model::JointModel*> &jmodels = jmg->getJointModels();
  for (std::size_t j = 0 ; j < jmodels.size() ; ++j)
    min_distance += jmodels[j]->getMaximumExtent() * jmodels[j]->getDistanceFactor();
  // now remember the value that is 5% of that maximum distance; this is the minimum we would like goal states to vary,
  // to consider them during the evaluation process
  min_distance *= 0.05;
  
  // convert approach direction and translation direction to Eigen structures
  Eigen::Vector3d approach_direction, translation_direction;
  tf::vectorMsgToEigen(plan->grasp_.approach_direction, approach_direction);
  tf::vectorMsgToEigen(plan->grasp_.translation_direction, translation_direction);
  
  // state validity checking during the approach must ensure that the gripper posture is that for pre-grasping
  kinematic_state::StateValidityCallbackFn approach_validCallback = boost::bind(&isStateCollisionFree, pre_grasp_planning_scene_.get(), 
                                                                                collision_matrix_.get(), &plan->grasp_.pre_grasp_posture, _1, _2);
  
  // state validity checking during the translation after the grasp must ensure the gripper posture is that of the actual grasp
  kinematic_state::StateValidityCallbackFn translation_validCallback = boost::bind(&isStateCollisionFree, post_grasp_planning_scene_.get(),
                                                                                   collision_matrix_.get(), &plan->grasp_.grasp_posture, _1, _2);
  do 
  {
    for (std::size_t i = 0 ; i < plan->possible_goal_states_.size() && !signal_stop_ ; ++i)
    {
      // try to compute a straight line path that arrives at the goal using the specified approach direction
      kinematic_state::KinematicStatePtr first_approach_state(new kinematic_state::KinematicState(*plan->possible_goal_states_[i]));
      std::vector<kinematic_state::KinematicStatePtr> approach_states;
      double d_approach = first_approach_state->getJointStateGroup(plan->planning_group_)->computeCartesianPath(approach_states, plan->ik_link_name_, -approach_direction,
                                                                                                                false, plan->grasp_.desired_approach_distance, 
                                                                                                                max_step_, jump_factor_, approach_validCallback);
      
      // if we were able to follow the approach direction for sufficient length, try to compute a translation direction
      if (d_approach > plan->grasp_.min_approach_distance && !signal_stop_)
      {
        if (plan->grasp_.desired_translation_distance > 0.0)
        {
          
          // try to compute a straight line path that moves from the goal in a desired direction
          kinematic_state::KinematicStatePtr last_translation_state(new kinematic_state::KinematicState(*plan->possible_goal_states_[i]));  
          std::vector<kinematic_state::KinematicStatePtr> translation_states;
          double d_translation = last_translation_state->getJointStateGroup(plan->planning_group_)->computeCartesianPath(translation_states, plan->ik_link_name_, 
                                                                                                                         translation_direction, true, 
                                                                                                                         plan->grasp_.desired_translation_distance, 
                                                                                                                         max_step_, jump_factor_, translation_validCallback);
          // if sufficient progress was made in the desired direction, we have a goal state that we can consider for future stages
          if (d_translation > plan->grasp_.min_translation_distance && !signal_stop_)
          {
            plan->approach_state_.swap(first_approach_state);
            plan->translation_state_.swap(last_translation_state);

            std::reverse(approach_states.begin(), approach_states.end());
            kinematic_trajectory::KinematicTrajectoryPtr approach_traj(new kinematic_trajectory::KinematicTrajectory(pre_grasp_planning_scene_->getKinematicModel(), plan->planning_group_));
            approach_traj->swap(approach_states);

            kinematic_trajectory::KinematicTrajectoryPtr translation_traj(new kinematic_trajectory::KinematicTrajectory(post_grasp_planning_scene_->getKinematicModel(), plan->planning_group_));
            translation_traj->swap(translation_states);
            
            time_param_.computeTimeStamps(*approach_traj); 
            time_param_.computeTimeStamps(*translation_traj);            
            
            plan->trajectories_.push_back(approach_traj);
            plan->trajectory_descriptions_.push_back("approach");
            
            addGraspTrajectory(plan, plan->grasp_.grasp_posture, "grasp");
            
            plan->trajectories_.push_back(translation_traj);
            plan->trajectory_descriptions_.push_back("translation");
            
            return true;          
          }
        }
        else
        {          
          plan->approach_state_.swap(first_approach_state);
          std::reverse(approach_states.begin(), approach_states.end());
          kinematic_trajectory::KinematicTrajectoryPtr approach_traj(new kinematic_trajectory::KinematicTrajectory(pre_grasp_planning_scene_->getKinematicModel(), plan->planning_group_));
          approach_traj->swap(approach_states);
          
          time_param_.computeTimeStamps(*approach_traj);
          
          plan->trajectories_.push_back(approach_traj);
          plan->trajectory_descriptions_.push_back("approach");
          
          addGraspTrajectory(plan, plan->grasp_.grasp_posture, "grasp");
          
          return true;          
        }
      }
      
    }
  }
  while (plan->possible_goal_states_.size() < max_goal_count_ && !signal_stop_ && samplePossibleGoalStates(plan, pre_grasp_planning_scene_->getCurrentState(), min_distance, max_fail_));
  plan->error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
  
  return false;
}

}
