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

// ROS
#include <ros/ros.h>

// MTC pick/place demo implementation
#include <moveit_task_constructor_demo/pick_place_task.h>

#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <tf2_ros/transform_broadcaster.h>

constexpr char LOGNAME[] = "moveit_task_constructor_demo";

void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi, const moveit_msgs::CollisionObject& object) {
	if (!psi.applyCollisionObject(object))
		throw std::runtime_error("Failed to spawn object: " + object.id);
}

moveit_msgs::CollisionObject createTable() {
	ros::NodeHandle pnh("~");
	std::string table_name, table_reference_frame;
	std::vector<double> table_dimensions;
	geometry_msgs::Pose pose;
	std::size_t errors = 0;
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_name", table_name);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_reference_frame", table_reference_frame);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_dimensions", table_dimensions);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_pose", pose);
	rosparam_shortcuts::shutdownIfError(LOGNAME, errors);

	moveit_msgs::CollisionObject object;
	object.id = table_name;
	object.header.frame_id = table_reference_frame;
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	object.primitives[0].dimensions = table_dimensions;
	pose.position.z -= 0.5 * table_dimensions[2];  // align surface with world
	object.primitive_poses.push_back(pose);
	return object;
}

moveit_msgs::CollisionObject createObject() {
	ros::NodeHandle pnh("~");
	std::string object_name, object_reference_frame;
	std::vector<double> object_dimensions;
	geometry_msgs::Pose pose;
	std::size_t error = 0;
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_name", object_name);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame", object_reference_frame);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_dimensions", object_dimensions);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_pose", pose);
	rosparam_shortcuts::shutdownIfError(LOGNAME, error);

	moveit_msgs::CollisionObject object;
	object.id = object_name;
	object.header.frame_id = object_reference_frame;
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
	object.primitives[0].dimensions = object_dimensions;
	pose.position.z += 0.5 * object_dimensions[0];
	object.primitive_poses.push_back(pose);
	return object;
}

void fillParameters(moveit_task_constructor_msgs::PlanPickPlaceGoal& goal) {
	ROS_INFO_NAMED(LOGNAME, "Filling task parameters");
	ros::NodeHandle pnh("~");
	// Planning group properties
	size_t errors = 0;
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "arm_group_name", goal.arm_group_name);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_group_name", goal.hand_group_name);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "eef_name", goal.eef_name);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_frame", goal.hand_frame);
	// errors += !rosparam_shortcuts::get(LOGNAME, pnh, "world_frame", goal.world_frame);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "grasp_frame_transform", goal.grasp_frame_transform);

	// Predefined pose targets
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_open_pose", goal.hand_open_pose);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_close_pose", goal.hand_close_pose);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "arm_home_pose", goal.arm_home_pose);

	// Target object
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "object_name", goal.object_id);
	// errors += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame", goal.object_reference_frame);
	std::string surface_link;
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "surface_link", surface_link);
	goal.support_surfaces = { surface_link };

	// Pick/Place metrics
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "approach_object_min_dist", goal.grasp.pre_grasp_approach.min_distance);
	// errors += !rosparam_shortcuts::get(LOGNAME, pnh, "approach_object_max_dist", goal.grasp.pre_grasp_approach.max_dist);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "lift_object_min_dist", goal.place_location.post_place_retreat.min_distance);
	// errors += !rosparam_shortcuts::get(LOGNAME, pnh, "lift_object_max_dist", goal.place_location.post_place_retreat.lift_object_max_dist);
	// errors += !rosparam_shortcuts::get(LOGNAME, pnh, "place_surface_offset", goal.place_surface_offset);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "world_frame", goal.place_location.header.frame_id);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "place_pose", goal.place_location.pose.place_pose);
	rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

int main(int argc, char** argv) {
	ROS_INFO_NAMED(LOGNAME, "Init node");
	ros::init(argc, argv, "mtc_tutorial");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// Add table and object to planning scene
	ros::Duration(1.0).sleep();  // Wait for ApplyPlanningScene service
	moveit::planning_interface::PlanningSceneInterface psi;
	ros::NodeHandle pnh("~");
	if (pnh.param("spawn_table", true))
		spawnObject(psi, createTable());
	spawnObject(psi, createObject());

	// Construct and run pick/place task
	actionlib::SimpleActionClient<moveit_task_constructor_msgs::PlanPickPlaceAction> plan_client("plan_pick_place", true);
	actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction> execute_client("execute_task_solution", true);


	moveit_task_constructor_demo::PickPlaceTask pick_place_task("pick_place_task", nh);
	pick_place_task.loadParameters();
	pick_place_task.init();
	if (pick_place_task.plan()) {
		ROS_INFO_NAMED(LOGNAME, "Planning succeeded");
		if (pnh.param("execute", false)) {
			pick_place_task.execute();
			ROS_INFO_NAMED(LOGNAME, "Execution complete");
		} else {
			ROS_INFO_NAMED(LOGNAME, "Execution disabled");
		}
	} else {
		ROS_INFO_NAMED(LOGNAME, "Planning failed");
	}

	// Keep introspection alive
	ros::waitForShutdown();
	return 0;
}
