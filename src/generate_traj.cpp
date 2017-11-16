#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <Eigen/Geometry>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
std::vector<Eigen::vector3d> sphere_centers;
std::vector<double> sphere_radius;
std::vector<Eigen::Vector3d> > Traj_pos;
std::vector<std::vector<double> > interpTraj;
std::vector<std::vector<double> > initpTraj;
const to_obs = 0.2;
const int obs_num=2;
Eigen::vector3d currentpos;
Eigen::vector3d goal;
Eigen::vector3d start;
double cur2goal_dis;

class mid_info
{
  
public:
  Eigen::vector3d pos;
  double dis;
  mid_info():p(Eigen::Vector3d(0,0,0)),dis(0.0){};

  mid_info(Eigen::vector3d p, double d):pos(p), dis(d){};
  
};
bool Mid_Greater( const mid_info& a, const mid_info& b)  { return a.dis > b.dis; }

bool cubic_interp(std::vector<std::vector<double> >& result, 
          const std::vector<std::vector<double> >& pidpoints)
{
  size_t pos_len = 0;
  if(pidpoints.size()>1)
    pos_len = pidpoints[0].size();
  else
    return false;
  if(pos_len != 6) return false;
  
  result.clear();
  for(size_t k = 0; k < pidpoints.size()*20; k++)
  {
    std::vector<double> pos(6,0.0);
    result.push_back(pos);
  }

  for(size_t i = 0; i < pos_len; i++)
  {
    std::vector<double> X,Y;
    for( size_t j = 0; j < pidpoints.size(); j++)
    {
      X.push_back(j);
      Y.push_back(pidpoints[j][i]);
    }
    tk::spline s;
    s.set_points(X,Y);
    for(int k =0; k<pidpoints.size()*20; k++)
    {
      double val  = 0.05*k;
      result[k][i] = s(val);
    }
  }
  return true;

}

moveit_msgs::RobotTrajectory toROSJointTrajectory(const std::vector<std::vector<double> >& points,
                      const std::vector<std::string>& joint_names,
                      double time_delay)
{
  moveit_msgs::RobotTrajectory result;
  result.joint_trajectory.header.stamp = ros::Time::now();
  result.joint_trajectory.header.frame_id = "/world";
  result.joint_trajectory.joint_names = joint_names;

  double time_offset = 0.0;
  for(auto it = points.begin(); it!= points.end(); ++it)
  {
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions = *it;
    pt.velocities.resize((*it).size(), 0.0);
    pt.accelerations.resize((*it).size(), 0.0);
    pt.effort.resize((*it).size(), 0.0);
    // set the time into the trajectory
    pt.time_from_start = ros::Duration(time_offset);
    // increment time
    time_offset += time_delay;

    result.joint_trajectory.points.push_back(pt);
    
  }
  return result;

}

std::vector<mid_info> pmids_vector;
//// global 
void GenerateGaps()
{
  for(size_t i = 0; i < obs_num; i++)
  {
    if(i==obs_num) 
      break;
    double dis;
    Eigen::Vector3d pos;

    for(size_t j = i+1; j < obs_num; j++)
    {
      dis = (sphere_centers[i] - sphere_centers[j]).norm();
      pos = (sphere_centers[i] + sphere_centers[j])*0.5;
      pmids_vector.push_back(mid_info(pos,dis));
    }

  }
  std::sort(pmids_vector.begin(), pmids_vector.end(),disgreater);
  for(auto i:pmids_vector)
  {
    std::cout << i.dis <<" " << i.pos.x << " " << i.pos.y<< std::endl; 
  }
}


void addobstacles()
{
  ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
      ROS_INFO("sleep");
      ros::WallDuration sleep_t(0.5);
      sleep_t.sleep();
    }
  //setup
  //robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    //robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
    moveit_msgs::CollisionObject co;
    co.header.stamp = ros::Time::now();
    co.header.frame_id = "world";
    co.id = "sphere";
    co.operation = moveit_msgs::CollisionObject::ADD;

    shape_msgs::SolidPrimitive sphere;
    sphere.type = shape_msgs::SolidPrimitive::SPHERE;
    sphere.dimensions.push_back(0.2);
    sphere_radius.push_back(0.2);

    geometry_msgs::Pose pose;
    pose.position.x = 0.6;
    pose.position.y = 0.5;
    pose.position.z = 1;
    pose.orientation.w = 1.0;
    sphere_centers.push_back(Eigen::vector3d(0.6,0.5,1));
  
    co.primitives.push_back(sphere);
    co.primitive_poses.push_back(pose);

    pub_co.publish(co);
    ROS_INFO("Adding the object into the world");
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(co);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);
    sleep_time.sleep();

    ros::ServiceClient planning_scene_diff_client =
    nh.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    planning_scene_diff_client.waitForExistence();
  // and send the diffs to the planning scene via a service call:
  moveit_msgs::ApplyPlanningScene srv;
  srv.request.scene = planning_scene;
  planning_scene_diff_client.call(srv);
}

bool isMidFree(const Eigen::Vector3d& point)
{

  double dis_cu2p = (point - currentpos).norm();
  if(dis_cu2p < 0.002)
    return true;
  Eigen::Vector3d dir_cu2p = (point - currentpos).normalized();

  for(size_t i =0; i < obs_num; i++)
  {
      if((point - sphere_centers[i]) < sphere_radius[i])
        return false;

      Eigen::Vector3d cur2obs = sphere_centers[i] - currentpos;
      double proj = cur2obs.dot(dir_cu2p);
      if(proj<0) continue;
      if(proj>dis_cu2p) continue;
      Eigen::Vector3d closestpoint = proj*dir_cu2p;
      if((closestpoint-sphere_centers[i]).norm() <= sphere_radius[i])
        return false;
  }

  cout << point(0)<<" " << point(1) << " " << point(2) << "is free!!";
  return true;


}

void AddMidPoint(const Eigen::Vector3d& point)
{
  Traj_pos.push_back(point);
  currentpos = point;
  cur2goal_dis = (currentpos - goal).norm();

}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "myplanner init");
 	ros::AsyncSpinner spinner(1);
 	spinner.start();
 	ros::NodeHandle nh;

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  ros::Publisher display_publisher =
      node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(kinematic_model));
  robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
  const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("manipulator");
  const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

  ros::ServiceClient ik_service_client = nh.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
  addobstacles();

  
  
  


  while(currentpos != goal)
  {
    if(isMidFree(goal))
    {
      cout << "goal is reaachable!!!" << endl;
      AddMidPoit(goal);
    }
    bool foundonemid = false;
    
    for(auto it = pmids_vector.begin(); it != pmids_vector.end(); ++it)
    {
      const Eigen::Vector3d p_mid= it->pos;
      if(isMidFree(p_mid))
      {
        foundonemid = true;
        AddMidPoit(p_mid);
        break;
      }

    }
    if(!foundonemid)
    {
      cout << "Mid Point Not Found " << endl;
      return false;
    }
  }

  Eigen::Affine3d pose= Eigen::Affine3d::Identity();
  pose.translation() = Eigen::Vector3d::Zero();
  std::vector<double> joint_values;
  moveit_msgs::GetPositionIK::Request ik_req;
  moveit_msgs::RobotState moveit_rs;
  ik_req.ik_request.robot_state = moveit_rs;
  ik_req.ik_request.avoid_collisions = true;
  initpTraj.clear();

  for(auto it = Traj_pos.begin(); it != Traj_pos.end(); it++)
  {


      pose.translation() = *it;
      geometry_msgs::PoseStamped pose_s;
      pose_s.header.stamp = ros::Time::now();
      pose_s.header.frame_id = "world";
      pose_s.pose = pose;


      ik_req.ik_request.pose_stamped = pose_s;
      ik_req.ik_request.timeout = ros::Duration(0.01);
      ik_req.ik_request.attempts = 3;
      
      moveit_msgs::GetPositionIK::Response ik_res;

      if(ik_service_client.call(ik_req, ik_res))
      {
        if (ik_res.error_code.val == moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION){
        ROS_INFO("*******************no ik solution!!!!**************");
        } 
      else 
        {
           moveit::core::robotStateMsgToRobotState(ik_res.solution, robot_state);
           robot_state.copyJointGroupPositions(joint_model_group, joint_values);
           initpTraj.push_back(joint_values);
          for (std::size_t i = 0; i < joint_names.size(); ++i)
          {
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
          }
        }
      }
  
  }

  bool cubic_suc =  cubic_interp(interpTraj,initpTraj);
  if (!cubic_suc)
  {
    std::cout<<"cubic fails!!!"<<std::endl;
    return 0;
  }

  moveit_msgs::RobotTrajectory joint_solution = toROSJointTrajectory(interpTraj, joint_names, 1.0);
  ROS_INFO("Visualizing the trajectory");
  display_trajectory.trajectory.push_back(joint_solution);
  display_publisher.publish(display_trajectory);
 // std::cout << display_trajectory<<std::endl;
  sleep_time.sleep();
  ROS_INFO("Done");
  
  return 0;


  	

	 
  	

}