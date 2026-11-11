// Offline MoveIt 2 IK comparison. No driver, controller action, or robot topics.
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <chrono>

std::string read(const std::string& p) {
  std::ifstream f(p); if(!f) throw std::runtime_error("Cannot open "+p);
  return {std::istreambuf_iterator<char>(f),std::istreambuf_iterator<char>()};
}
int main(int argc, char** argv) {
  if(argc!=5) {std::cerr<<"URDF SRDF cases.json output.yaml\n";return 2;}
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("piper_offline_ik",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto urdf=urdf::parseURDF(read(argv[1]));
  auto srdf=std::make_shared<srdf::Model>();
  if(!urdf || !srdf->initString(*urdf,read(argv[2]))) throw std::runtime_error("Model parse failed");
  auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
  auto group=model->getJointModelGroup("left_arm");
  if(!group || group->getVariableCount()!=6) throw std::runtime_error("Wrong chain");
  planning_scene::PlanningScene scene(model);
  moveit::core::RobotState state(model);state.setToDefaultValues();
  state.setVariablePosition("left_gripper",.019);state.setVariablePosition("right_gripper",.0021);
  auto cases=YAML::LoadFile(argv[3]);YAML::Node output;
  output["hardware_writes"]=0;output["execute_available"]=false;
  output["base_frame"]="left_base_link";output["tip_frame"]="left_link6";
  output["approximate_ik_allowed"]=false;output["scene_geometry"]="robot CAD only; no current table/cup/coaster clearance claim";
  output["joint_names"]=group->getVariableNames();
  pluginlib::ClassLoader<kinematics::KinematicsBase> loader("moveit_core","kinematics::KinematicsBase");
  for(const std::string plugin:{"kdl_kinematics_plugin/KDLKinematicsPlugin","lma_kinematics_plugin/LMAKinematicsPlugin"}) {
    auto solver=loader.createSharedInstance(plugin);
    if(!solver->initialize(node,*model,"left_arm","left_base_link",{"left_link6"},.005)) throw std::runtime_error("IK init failed");
    if(solver->getJointNames()!=group->getVariableNames()) throw std::runtime_error("Joint order mismatch");
    for(const auto& c:cases["cases"]) {
      Eigen::Matrix3d R;
      for(int i=0;i<3;i++)for(int j=0;j<3;j++)R(i,j)=c["rotation"][i][j].as<double>();
      Eigen::Quaterniond quat(R);geometry_msgs::msg::Pose pose;
      pose.position.x=c["position"][0].as<double>();pose.position.y=c["position"][1].as<double>();pose.position.z=c["position"][2].as<double>();
      pose.orientation.x=quat.x();pose.orientation.y=quat.y();pose.orientation.z=quat.z();pose.orientation.w=quat.w();
      int index=0;
      for(const auto& legacy:c["solutions"]) {
        auto seed=legacy["seed"].as<std::vector<double>>();std::vector<double> solution;
        moveit_msgs::msg::MoveItErrorCodes code;kinematics::KinematicsQueryOptions options;
        options.return_approximate_solution=false;
        auto start=std::chrono::steady_clock::now();
        bool solved=solver->searchPositionIK(pose,seed,.1,solution,code,options);
        double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
        YAML::Node row;row["plugin"]=plugin;row["case"]=c["name"].as<std::string>();row["seed_index"]=index++;
        row["solved"]=solved;row["error_code"]=code.val;row["elapsed_s"]=elapsed;
        if(solved) {
          state.setJointGroupPositions(group,solution);state.update();
          Eigen::Isometry3d T=state.getGlobalLinkTransform("left_base_link").inverse()*state.getGlobalLinkTransform("left_link6");
          Eigen::Vector3d goal(pose.position.x,pose.position.y,pose.position.z);
          double pe=(T.translation()-goal).norm();double re=Eigen::AngleAxisd(R*T.rotation().transpose()).angle();
          row["position_error_m"]=pe;row["orientation_error_rad"]=re;
          row["strict_pass"]=pe<=.0002 && re<=.002 && state.satisfiesBounds(group);
          row["q"]=solution;row["joint_limits_ok"]=state.satisfiesBounds(group);
          collision_detection::CollisionRequest request;collision_detection::CollisionResult result;
          request.contacts=true;request.max_contacts=100;
          scene.checkSelfCollision(request,result,state);
          row["cad_self_collision"]=result.collision;
          for(const auto& contact:result.contacts) {YAML::Node pair;pair.push_back(contact.first.first);pair.push_back(contact.first.second);row["contacts"].push_back(pair);}
        }
        output["results"].push_back(row);
      }
    }
  }
  std::ofstream out(argv[4]);out<<output<<"\n";
  rclcpp::shutdown();return 0;
}
