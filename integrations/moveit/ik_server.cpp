// Persistent MoveIt2 KDL IK only. No robot transport, executor, or motion action.
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <yaml-cpp/yaml.h>
#include <sys/prctl.h>
#include <signal.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <chrono>

std::string read(const std::string& p){std::ifstream f(p);if(!f)throw std::runtime_error("MODEL_FILE");return {std::istreambuf_iterator<char>(f),{}};}
std::vector<double> vector(const YAML::Node& n,size_t count){auto v=n.as<std::vector<double>>();if(v.size()!=count)throw std::runtime_error("VECTOR_SIZE");for(double x:v)if(!std::isfinite(x))throw std::runtime_error("NONFINITE");return v;}
void send(const YAML::Node& n){YAML::Emitter e;e.SetMapFormat(YAML::Flow);e.SetSeqFormat(YAML::Flow);e.SetDoublePrecision(17);e<<n;std::cout<<e.c_str()<<std::endl;}
int main(int argc,char**argv){
 if(argc!=3)return 2;prctl(PR_SET_PDEATHSIG,SIGTERM);if(getppid()==1)return 3;
 rclcpp::init(argc,argv);auto node=std::make_shared<rclcpp::Node>("piper_moveit_ik",rclcpp::NodeOptions().enable_rosout(false));
 auto u=urdf::parseURDF(read(argv[1]));auto s=std::make_shared<srdf::Model>();if(!u||!s->initString(*u,read(argv[2])))return 4;
 auto model=std::make_shared<moveit::core::RobotModel>(u,s);auto group=model->getJointModelGroup("left_arm");
 if(!group||group->getVariableCount()!=6)return 5;
 pluginlib::ClassLoader<kinematics::KinematicsBase> loader("moveit_core","kinematics::KinematicsBase");
 auto solver=loader.createSharedInstance("kdl_kinematics_plugin/KDLKinematicsPlugin");
 if(!solver->initialize(node,*model,"left_arm","left_base_link",{"left_link6"},.005))return 6;
 if(solver->getJointNames()!=group->getVariableNames())return 7;
 YAML::Node ready;ready["ready"]=true;ready["solver"]="moveit2_kdl";ready["joint_names"]=group->getVariableNames();ready["hardware_writes"]=0;send(ready);
 std::string line;
 while(std::getline(std::cin,line)){
  YAML::Node out;out["ok"]=false;out["solver"]="moveit2_kdl";out["hardware_writes"]=0;
  try{
   if(line.size()>8192)throw std::runtime_error("REQUEST_TOO_LARGE");auto req=YAML::Load(line);
   auto id=req["id"].as<std::string>();if(id.size()!=32)throw std::runtime_error("REQUEST_ID");out["id"]=id;
   auto p=vector(req["position"],3),q=vector(req["quaternion_xyzw"],4),seed=vector(req["seed"],6);
   double timeout=req["timeout_s"].as<double>();if(!std::isfinite(timeout)||timeout<=0||timeout>.1)throw std::runtime_error("TIMEOUT_LIMIT");
   Eigen::Quaterniond quat(q[3],q[0],q[1],q[2]);if(std::abs(quat.norm()-1)>.000001)throw std::runtime_error("UNIT_QUATERNION");
   moveit::core::RobotState state(model);state.setToDefaultValues();state.setJointGroupPositions(group,seed);
   if(!state.satisfiesBounds(group,1e-8))throw std::runtime_error("SEED_LIMITS");
   geometry_msgs::msg::Pose pose;pose.position.x=p[0];pose.position.y=p[1];pose.position.z=p[2];pose.orientation.x=q[0];pose.orientation.y=q[1];pose.orientation.z=q[2];pose.orientation.w=q[3];
   std::vector<double> consistency,solution;
   if(req["consistency_limits"]){consistency=vector(req["consistency_limits"],6);for(double x:consistency)if(x<=0||x>1.047197551197)throw std::runtime_error("CONSISTENCY_LIMIT");}
   moveit_msgs::msg::MoveItErrorCodes code;kinematics::KinematicsQueryOptions options;options.return_approximate_solution=false;
   auto start=std::chrono::steady_clock::now();bool ok=solver->searchPositionIK(pose,seed,timeout,consistency,solution,code,options);
   out["elapsed_s"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();out["error_code"]=code.val;
   if(!ok)throw std::runtime_error("MOVEIT_NO_IK_SOLUTION");
   state.setJointGroupPositions(group,solution);state.update();
   auto T=state.getGlobalLinkTransform("left_base_link").inverse()*state.getGlobalLinkTransform("left_link6");
   double pe=(T.translation()-Eigen::Vector3d(p[0],p[1],p[2])).norm(),re=Eigen::AngleAxisd(quat.toRotationMatrix()*T.rotation().transpose()).angle();
   if(!state.satisfiesBounds(group,1e-8)||pe>.0002||re>.002)throw std::runtime_error("IK_RESIDUAL_OR_LIMIT");
   out["ok"]=true;out["q"]=solution;out["position_error_m"]=pe;out["orientation_error_rad"]=re;
   out["frame"]="left_base_link";out["controlled_link"]="left_link6";out["approximate"]=false;
  }catch(const std::exception&e){out["error"]=e.what();}
  send(out);
 }
 rclcpp::shutdown();return 0;
}
