#pragma once

#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "util/interpolation.hpp"
#include "planner/locomotion/alip_mpc/include/NewStep_mpc.hpp"
#include "controller/whole_body_controller/force_task.hpp"
#include <Eigen/Dense>

class Task;
class ForceTask;
class PinocchioRobotSystem;
class NewStep_mpc;

class AlipMpcTrajectoryManager {
public:
  AlipMpcTrajectoryManager(NewStep_mpc *alipMpc, Task *com_xy_task, Task *com_z_task, 
                           Task *torso_ori, Task *lfoot_task, Task *lfoot_ori,
                           Task *rfoot_task, Task *rfoot_ori,
                           ForceTask *lf_force_task, ForceTask *rg_force_task,
                           PinocchioRobotSystem *robot);

  virtual ~AlipMpcTrajectoryManager()  = default;


  void setOri();
  void outsideCommand(const YAML::Node &node);

  void MpcSolutions(const double &tr_, const double &st_leg);
  void InertiaToMpcCoordinates();
  void OutputMpcToInertiaCoordinates();

  void GenerateCOMtraj(const double &tr_);  //mirar si devolver el resultado directo o la hermite curve
  void GenerateSwingFtraj();

  void UpdateCurrentOri(Task* task);
  void UpdateCurrentPos(Task* task);

  void UpdateDesired(const double t);
  void SetParameters(const YAML::Node &node);

  double ComputeZpos(const double &x, const double &y, const double &zH_);

  void SetSwingFootStart(Eigen::Vector3d pos){Swingfoot_start = pos;}
  void SetLydes(double des){indata.Ly_des = des;}
  //for testing
  void saveTrajectories(const double start_time, const double dt,const double end_time);
  
  //Getters
  input_data_t GetIndata() {return indata;}
  output_data_t GetOutdata() {return outdata;}
  full_horizon_sol GetFullsol(){return fullsol;}

  int printCounter;

private:
  NewStep_mpc *alipMpc;
  Task *com_xy_task;
  Task *com_z_task;
  Task *torso_ori;
  Task *lfoot_task;
  Task *lfoot_ori;
  Task *rfoot_task;
  Task *rfoot_ori;
  ForceTask *lf_force_task;
  ForceTask *rg_force_task;
  PinocchioRobotSystem *robot_;

  HermiteCurveVec *COMpos;
  HermiteCurveVec *first_half_curve_SwingPos;
  HermiteCurveVec *second_half_curve_SwingPos;

  QuadraticBezierCurve *BezierSwingPos;
  int Bezier;

  AlipSwing *AlipSwingPos;
  int Alip;

  AlipSwing2 *AlipSwingPos2;
  int Alip2;

  full_horizon_sol fullsol;  //refrence frame wrt current stance leg
  input_data_t indata;       //reference frame wrt current stance leg
  output_data_t outdata;
  double com_yaw;

  Eigen::Vector3d COM_end;
  Eigen::Vector3d COMvel_end;

  Eigen::Vector3d Swingfoot_start;

  Eigen::Vector3d Swingfoot_end;
  Eigen::Vector3d Swingfootvel_end = Eigen::Vector3d::Zero();


  Eigen::Vector3d stleg_pos;
  Eigen::Vector3d stleg_pos_torso_ori;


  Eigen::VectorXd des_ori_lfoot;
  Eigen::VectorXd des_ori_rfoot;
  Eigen::VectorXd des_ori_torso;
  bool first_ever;


  double mass;
  double swing_height; //this will be reference for step of 0.5 m will use linear swing heigh with (0,0)
  double reference_swing_height;//will have to change this, also can do a variable swing height
  int variable_height;                        //can have problems with small steps

  std::fstream file1;
  std::fstream file2;
  std::fstream file3;
  std::fstream file4;
  std::fstream file5;
  std::fstream file6;


  int saveCounter;




};