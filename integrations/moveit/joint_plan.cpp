// Offline planning only. Geometric output is not a timed hardware command.
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>

std::string read(const std::string& p) {std::ifstream f(p);return {std::istreambuf_iterator<char>(f),{}};}
int main(int argc,char** argv) {
  if(argc!=5)return 2;
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("piper_offline_plan",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto urdf=urdf::parseURDF(read(argv[1]));auto srdf=std::make_shared<srdf::Model>();
  if(!urdf || !srdf->initString(*urdf,read(argv[2])))return 3;
  auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);auto group=model->getJointModelGroup("left_arm");
  auto scene=std::make_shared<planning_scene::PlanningScene>(model);
  auto input=YAML::LoadFile(argv[3]);YAML::Node out;
  moveit::core::RobotState start(model),goal(model);start.setToDefaultValues();
  start.setJointGroupPositions(group,input["start6"].as<std::vector<double>>());
  start.setVariablePosition("left_gripper",.019);start.setVariablePosition("right_gripper",.0021);start.update();
  goal=start;goal.setJointGroupPositions(group,input["goal6"].as<std::vector<double>>());goal.update();scene->setCurrentState(start);
  out["hardware_writes"]=0;out["execute_available"]=false;out["timed_trajectory"]=false;
  out["environment_included"]=false;out["payload_included"]=false;
  collision_detection::CollisionRequest cr;collision_detection::CollisionResult a,b;
  cr.contacts=true;cr.max_contacts=100;scene->checkSelfCollision(cr,a,start);scene->checkSelfCollision(cr,b,goal);
  out["start_collision"]=a.collision;out["goal_collision"]=b.collision;
  for(const auto& contact:a.contacts){YAML::Node pair;pair.push_back(contact.first.first);pair.push_back(contact.first.second);out["start_contacts"].push_back(pair);}
  for(const auto& contact:b.contacts){YAML::Node pair;pair.push_back(contact.first.first);pair.push_back(contact.first.second);out["goal_contacts"].push_back(pair);}
  if(!a.collision && !b.collision && start.satisfiesBounds(group) && goal.satisfiesBounds(group)) {
    pluginlib::ClassLoader<planning_interface::PlannerManager> loader("moveit_core","planning_interface::PlannerManager");
    auto planner=loader.createSharedInstance("ompl_interface/OMPLPlanner");
    if(!planner->initialize(model,node,""))return 4;
    planning_interface::PlannerConfigurationSettings cfg;cfg.name="left_arm[RRTConnect]";cfg.group="left_arm";
    cfg.config["type"]="geometric::RRTConnect";cfg.config["longest_valid_segment_fraction"]="0.005";
    planning_interface::PlannerConfigurationMap configs;configs[cfg.name]=cfg;planner->setPlannerConfigurations(configs);
    planning_interface::MotionPlanRequest req;planning_interface::MotionPlanResponse res;
    req.group_name="left_arm";req.planner_id="RRTConnect";req.allowed_planning_time=3;req.num_planning_attempts=1;
    moveit::core::robotStateToRobotStateMsg(start,req.start_state);
    req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(goal,group,.0002));
    auto context=planner->getPlanningContext(scene,req,res.error_code_);
    bool solved=context && context->solve(res);out["solved"]=solved;out["error_code"]=res.error_code_.val;
    if(solved && res.trajectory_) {
      for(size_t i=0;i<res.trajectory_->getWayPointCount();i++) {
        std::vector<double> q;res.trajectory_->getWayPoint(i).copyJointGroupPositions(group,q);out["path6"].push_back(q);
      }
      out["planning_s"]=res.planning_time_;
    }
  } else {out["solved"]=false;out["reason"]="CAD collision or limits; not bypassed";}
  std::ofstream output(argv[4]);output<<out<<"\n";rclcpp::shutdown();return 0;
}
