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



  void MpcSolutions(const double &tr_, const double &st_leg);
  void InertiaToMpcCoordinates();
  void OutputMpcToInertiaCoordinates();

  void GenerateCOMtraj(const double &tr_);  //mirar si devolver el resultado directo o la hermite curve
  void GenerateSwingFtraj();

  void UpdateCurrentOri(Task* task);
  void UpdateCurrentPos(Task* task);

  void UpdateDesired(const double t);

  //void RToLstance();
  //void LToRstance();
  double ComputeZpos(const double &x, const double &y, const double &zH_);


  //for testing
  void saveTrajectories(const double start_time, const double dt,const double end_time);

  //getter try without getter if doesn't work try with getter;
  //full_horizon_sol GetfullMpcSol() { return fullsol;}
  
  
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

  QuadraticBezierCurve *SwingPos;

  full_horizon_sol fullsol;  //refrence frame wrt current stance leg
  input_data_t indata;       //reference frame wrt current stance leg
  output_data_t outdata;

  Eigen::Vector3d COM_end;
  Eigen::Vector3d COMvel_end;

  Eigen::Vector3d Swingfoot_end;
  Eigen::Vector3d Swingfootvel_end = Eigen::Vector3d::Zero();


  Eigen::Vector3d stleg_pos;



  double mass;
  double swing_height = 0.1;  //will have to change this, also can do a variable swing height
                              //can have problems with small steps

  std::fstream file1;
  std::fstream file2;
  std::fstream file3;
  std::fstream file4;
  int saveCounter;



};