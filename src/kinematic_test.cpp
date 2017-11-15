#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <boost/scoped_ptr.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "spline.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinematics_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //set up
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(kinematic_model));
  robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
  const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("manipulator");
  
  const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
  std::vector<double> joint_values(6, 0.0);
  joint_values[0] = -0.25;
  joint_values[1] = -0.94;
  joint_values[2] = 1.3;
  joint_values[3] = 0;
  joint_values[4] = 0.7;
  joint_values[5] = 0; 

  std::vector<double> new_joint_values(6, 0.0);
  robot_state.setJointGroupPositions(joint_model_group, joint_values);
  robot_state.copyJointGroupPositions(joint_model_group, new_joint_values);
  ROS_INFO_STREAM("new value: " << new_joint_values[0]);
  const Eigen::Affine3d &end_effector_state = robot_state.getGlobalLinkTransform("arm_6_link");
  ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
  ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());

  bool found_ik = robot_state.setFromIK(joint_model_group, end_effector_state, 10, 0.1);

  // Now, we can print out the IK solution (if found):
  if (found_ik)
  {
    robot_state.copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }
  ros::shutdown();
  return 0;

}
