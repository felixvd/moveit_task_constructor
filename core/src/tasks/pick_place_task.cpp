/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019 PickNik LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Henning Kayser, Simon Goldstein
   Desc:   A demo to show MoveIt Task Constructor in action
*/

#include <moveit/task_constructor/tasks/pick_place_task.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace moveit {
namespace task_constructor {
namespace tasks {

constexpr char LOGNAME[] = "pick_place_task";
PickPlaceTask::PickPlaceTask(const std::string& task_name)
  : task_name_(task_name) {
    grasp_provider_class_loader_ = std::make_unique<GraspProviderPluginLoader>("moveit_task_constructor_core", "moveit::task_constructor::stages::GraspProviderBase");
    place_provider_class_loader_ = std::make_unique<PlaceProviderPluginLoader>("moveit_task_constructor_core", "moveit::task_constructor::stages::PlaceProviderBase");
    task_.reset();
    task_.reset(new moveit::task_constructor::Task(task_name_));
    task_->reset();
    task_->loadRobotModel();
    current_state_stage_ = nullptr;
    attach_object_stage_ = nullptr;
  }

std::vector<geometry_msgs::PoseStamped> merge_grasp_poses(const moveit_msgs::Grasp& grasp_msg, const std::vector<geometry_msgs::PoseStamped>& in_poses) 
{
  std::vector<geometry_msgs::PoseStamped> poses(in_poses);
  // Add grasp pose from grasp message  if it has been set
  if (grasp_msg.grasp_pose.pose.orientation.x != 0 || grasp_msg.grasp_pose.pose.orientation.y != 0 || 
      grasp_msg.grasp_pose.pose.orientation.z != 0 || grasp_msg.grasp_pose.pose.orientation.w != 0 ||
      grasp_msg.grasp_pose.pose.position.x != 0 || grasp_msg.grasp_pose.pose.position.y != 0 || 
      grasp_msg.grasp_pose.pose.position.z != 0)
  {
    poses.emplace(poses.begin(), grasp_msg.grasp_pose);
  }
  return poses;
}

void PickPlaceTask::init(const Parameters& parameters)
{
  ROS_INFO_NAMED(LOGNAME, "Initializing task pipeline");
  // Reset ROS introspection before constructing the new object
  // TODO(henningkayser): verify this is a bug, fix if possible
  if(task_){
    task_->clear();
    task_->loadRobotModel();
  }
  Task& t = *task_;

  // Sampling planner
  // TODO(henningkayser): Setup and parameterize alternative planners
  auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
  sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

  // Cartesian planner
  auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScaling(1.0);
  cartesian_planner->setMaxAccelerationScaling(1.0);
  cartesian_planner->setStepSize(.01);

  // Set task properties
  t.setProperty("group", parameters.arm_group_name_);
  t.setProperty("eef", parameters.eef_name_);
  t.setProperty("hand", parameters.hand_group_name_);
  t.setProperty("ik_frame", parameters.hand_frame_);

  /****************************************************
   *                                                  *
   *               Current State                      *
   *                                                  *
   ***************************************************/
  {
    auto _current_state = std::make_unique<stages::CurrentState>("current state");
    _current_state->setTimeout(10);

    // Verify that object is not attached
    auto applicability_filter =
        std::make_unique<stages::PredicateFilter>("applicability test", std::move(_current_state));
    applicability_filter->setPredicate([&](const SolutionBase& s, std::string& comment) {
      s.start()->scene()->printKnownObjects(std::cout);
    
      if (parameters.task_type_ == moveit_task_constructor_msgs::PlanPickPlaceGoal::PICK_ONLY || 
          parameters.task_type_ == moveit_task_constructor_msgs::PlanPickPlaceGoal::PICK_AND_PLACE)
      {
        if (s.start()->scene()->getCurrentState().hasAttachedBody(parameters.object_name_))
        {
          comment = "object with id '" + parameters.object_name_ + "' is already attached and cannot be picked";
          return false;
        }
      }
      else if (parameters.task_type_ == moveit_task_constructor_msgs::PlanPickPlaceGoal::PLACE_ONLY)
      {
        if (!s.start()->scene()->getCurrentState().hasAttachedBody(parameters.object_name_))
        {
          comment = "object with id '" + parameters.object_name_ + "' is not attached, so it cannot be placed";
          return false;
        }
      }
      return true;
    });

    current_state_stage_ = applicability_filter.get();
    t.add(std::move(applicability_filter));
  }

  if (parameters.task_type_ == moveit_task_constructor_msgs::PlanPickPlaceGoal::PICK_ONLY || 
      parameters.task_type_ == moveit_task_constructor_msgs::PlanPickPlaceGoal::PICK_AND_PLACE)
  {
    /****************************************************
     *                                                  *
     *               Open Hand                          *
     *                                                  *
     ***************************************************/
    {  // Open Hand
      auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
      stage->setGroup(parameters.hand_group_name_);
      stage->setGoal(parameters.hand_open_pose_);
      t.add(std::move(stage));
    }

    /****************************************************
     *                                                  *
     *               Move to Pick                       *
     *                                                  *
     ***************************************************/
    {  // Move-to pre-grasp
      auto stage = std::make_unique<stages::Connect>(
          "move to pick", stages::Connect::GroupPlannerVector{ { parameters.arm_group_name_, sampling_planner } });
      stage->setTimeout(5.0);
      stage->properties().configureInitFrom(Stage::PARENT);
      t.add(std::move(stage));
    }

    /****************************************************
     *                                                  *
     *               Pick Object                        *
     *                                                  *
     ***************************************************/
    Stage* attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator
    {
      const std::vector<geometry_msgs::PoseStamped> grasp_poses = merge_grasp_poses(parameters.grasp, parameters.extra_grasp_poses);
      std::string grasp_provider_plugin;
      if (!grasp_poses.empty())
      {
        grasp_provider_plugin = "moveit_task_constructor/GraspProviderFixedPoses";
      }
      else
      {
        grasp_provider_plugin = parameters.grasp_provider_plugin_name_;
      }
      auto stage = std::make_unique<stages::Pick>("Pick object", grasp_provider_plugin);
      stage->properties().property("eef_group").configureInitFrom(Stage::PARENT, "hand");
      stage->properties().property("eef_parent_group").configureInitFrom(Stage::PARENT, "group");
      stage->setObject(parameters.object_name_);
      stage->setEndEffector(parameters.eef_name_);
      stage->setEndEffectorOpenClose(parameters.hand_open_pose_, parameters.hand_close_pose_);
      stage->setSupportSurfaces(parameters.support_surfaces_);
      stage->setIKFrame(parameters.grasp_frame_transform_, parameters.hand_frame_);
      if (!grasp_poses.empty())
      {
        stage->GraspProviderPlugin()->properties().set("poses", grasp_poses);
      }
      else // TODO: Go through properties systematically
      {
        stage->GraspProviderPlugin()->properties().set("angle_delta", M_PI / 12);  // Set plugin-specific properties
      }
      stage->setMonitoredStage(current_state);
      stage->setApproachMotion(parameters.approach_object_direction_,parameters.approach_object_min_dist_, parameters.approach_object_max_dist_);
      stage->setLiftMotion(parameters.lift_object_direction_, parameters.lift_object_min_dist_, parameters.lift_object_max_dist_);
      attach_object_stage = stage->attachStage();
      t.add(std::move(stage));
    }
  }

  if (parameters.task_type_ == moveit_task_constructor_msgs::PlanPickPlaceGoal::PLACE_ONLY || 
      parameters.task_type_ == moveit_task_constructor_msgs::PlanPickPlaceGoal::PICK_AND_PLACE)
  {

    /******************************************************
     *                                                    *
     *          Move to Place                             *
     *                                                    *
     *****************************************************/
    {
      auto stage = std::make_unique<stages::Connect>(
          "move to place", stages::Connect::GroupPlannerVector{ { parameters.arm_group_name_, sampling_planner } });
      stage->setTimeout(5.0);
      stage->properties().configureInitFrom(Stage::PARENT);
      t.add(std::move(stage));
    }

    /******************************************************
     *                                                    *
     *          Place Object                              *
     *                                                    *
     *****************************************************/
    //   Stage* detach_object_stage = nullptr;  // Forward detach_object_stage to place pose generator
    // {
    //   auto stage = std::make_unique<stages::Place>("Place object", parameters.place_provider_plugin_name_);
    //   stage->properties().property("eef_group").configureInitFrom(Stage::PARENT, "hand");
    //   stage->properties().property("eef_parent_group").configureInitFrom(Stage::PARENT, "group");
    //   stage->setObject(parameters.object_name_);
    //   stage->setEndEffector(parameters.eef_name_);
    //   stage->setEndEffectorOpenClose(parameters.hand_open_pose_, parameters.hand_close_pose_);
    //   stage->setSupportSurfaces(parameters.support_surfaces_);
    //   stage->setIKFrame(parameters.grasp_frame_transform_, parameters.hand_frame_);
    //   // stage->PlaceProviderPlugin()->properties().set("angle_delta", M_PI / 12);  // Set plugin-specific properties
    //   stage->setMonitoredStage(current_state);
    //   stage->setApproachMotion(parameters.approach_object_direction_,parameters.approach_object_min_dist_, parameters.approach_object_max_dist_);
    //   stage->setLiftMotion(parameters.lift_object_direction_, parameters.lift_object_min_dist_, parameters.lift_object_max_dist_);
    //   detach_object_stage = stage->attachStage();
    //   t.add(std::move(stage));
    // }

    /******************************************************
     *                                                    *
     *          Move to Home                              *
     *                                                    *
     *****************************************************/
    // {
    //   auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner);
    //   stage->properties().configureInitFrom(Stage::PARENT, { "group" });
    //   stage->setGoal(parameters.arm_home_pose_);
    //   stage->restrictDirection(stages::MoveTo::FORWARD);
    //   t.add(std::move(stage));
    // }
  }
}

bool PickPlaceTask::plan() {
	ROS_INFO_NAMED(LOGNAME, "Start searching for task solutions");
	try {
		task_->plan(10);  // TODO: parameterize
	} catch (InitStageException& e) {
		ROS_ERROR_STREAM_NAMED(LOGNAME, "Initialization failed: " << e);
		return false;
	}
	if (task_->numSolutions() == 0) {
		ROS_ERROR_NAMED(LOGNAME, "Planning failed");
		return false;
	}
	return true;
}

void PickPlaceTask::getSolutionMsg(moveit_task_constructor_msgs::Solution& solution) {
  task_->solutions().front()->fillMessage(solution);
}

void PickPlaceTask::publishAllSolutions(const bool& wait) {
  task_->publishAllSolutions(wait);
}

}  // namespace tasks
}  // namespace task_constructor
}  // namespace moveit
