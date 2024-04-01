#include "controller/whole_body_controller/managers/alipmpc_trajectory_manager.hpp"
#include "util/interpolation.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "planner/locomotion/dcm_planner/foot_step.hpp"
#include "util/util.hpp"
#include <iostream>
#include <fstream>
#include "configuration.hpp"

void printInData(input_data_t* indata);


AlipMpcTrajectoryManager::AlipMpcTrajectoryManager(NewStep_mpc *alipMpc,
    Task *com_xy_task, Task *com_z_task, Task *torso_ori, Task *lfoot_task, 
    Task *lfoot_ori, Task *rfoot_task, Task *rfoot_ori, 
    ForceTask *lf_force_, ForceTask *rg_force_, PinocchioRobotSystem *robot)
    : alipMpc(alipMpc), com_xy_task(com_xy_task), com_z_task(com_z_task), 
      torso_ori(torso_ori), lfoot_task(lfoot_task), lfoot_ori(lfoot_ori),
      rfoot_task(rfoot_task), rfoot_ori(rfoot_ori), 
      lf_force_task(lf_force_), rg_force_task(rg_force_),robot_(robot),
      first_half_curve_SwingPos(nullptr), second_half_curve_SwingPos(nullptr) {
  
  util::PrettyConstructor(2, "AlipMpcTrajectoryManager");
  printCounter = 5;
  first_ever = false;

  indata.xlip_current[2] = 0;
  indata.xlip_current[3] = 0; 
  saveCounter = 0;

  /*
  file1.open(THIS_COM "/test/alip/alip_COM_trajectory.txt", std::fstream::in | std::fstream::out | std::fstream::app);
  file2.open(THIS_COM "/test/alip/Swing1_trajectory.txt", std::fstream::in | std::fstream::out | std::fstream::app);
  file3.open(THIS_COM "/test/alip/Swing2_trajectory.txt", std::fstream::in | std::fstream::out | std::fstream::app);
  file4.open(THIS_COM "/test/alip/BezierSwing_trajectory.txt", std::fstream::in | std::fstream::out | std::fstream::app);
  */

  file1.open(THIS_COM "/test/alip/MpcCOMstate.txt", std::fstream::out);
  file2.open(THIS_COM "/test/alip/Swing1_trajectory.txt", std::fstream::out);
  file3.open(THIS_COM "/test/alip/Swing2_trajectory.txt", std::fstream::out);
  file4.open(THIS_COM "/test/alip/BezierSwing_trajectory.txt", std::fstream::out);
  file5.open(THIS_COM "/test/alip/AlipSwing_trajectory.txt", std::fstream::out);
  file6.open(THIS_COM "/test/alip/Alip2Swing_trajectory.txt", std::fstream::out);
  file7.open(THIS_COM "/test/alip/robotSwingFootTraj.txt", std::fstream::out);
  file8.open(THIS_COM "/test/alip/RobotCOM.txt", std::fstream::out);
  file9.open(THIS_COM "/test/alip/RobotCommand.txt", std::fstream::out);



}  //need to add ori 

void AlipMpcTrajectoryManager::initializeOri(){
  des_torso_iso = robot_->GetLinkIsometry(draco_link::torso_link);
  FootStep::MakeHorizontal(des_torso_iso);
  des_lfoot_iso = des_torso_iso;
  des_rfoot_iso = des_torso_iso;
  MakeParallelToGround(des_lfoot_iso);
  MakeParallelToGround(des_rfoot_iso);
  Eigen::Quaterniond des_torso_ori_quat(des_torso_iso.linear());
  Eigen::Quaterniond des_lfoot_ori_quat(des_lfoot_iso.linear());
  Eigen::Quaterniond des_rfoot_ori_quat(des_rfoot_iso.linear());

  des_ori_torso = des_torso_ori_quat.normalized().coeffs();
  //des_ori_lfoot = des_lfoot_ori_quat.normalized().coeffs();
  //des_ori_rfoot = des_rfoot_ori_quat.normalized().coeffs();
  des_ori_lfoot = des_ori_torso;
  des_ori_rfoot = des_ori_torso;
}


void AlipMpcTrajectoryManager::setNewOri(){
    if (com_yaw != 0){
      des_torso_iso = robot_->GetLinkIsometry(draco_link::torso_link);
      FootStep::MakeHorizontal(des_torso_iso);
      Eigen::Matrix3d rotation; 
      rotation << cos(com_yaw), -sin(com_yaw), 0, 
                  sin(com_yaw), cos(com_yaw) , 0,
                  0           , 0            , 1;
      des_torso_iso.linear() = rotation*des_torso_iso.linear();
      des_lfoot_iso = des_torso_iso;
      des_rfoot_iso = des_torso_iso;
      MakeParallelToGround(des_lfoot_iso);
      MakeParallelToGround(des_rfoot_iso);
      Eigen::Quaterniond des_torso_ori_quat(des_torso_iso.linear());
      Eigen::Quaterniond des_lfoot_ori_quat(des_lfoot_iso.linear());
      Eigen::Quaterniond des_rfoot_ori_quat(des_rfoot_iso.linear());

      des_ori_torso = des_torso_ori_quat.normalized().coeffs();
      //des_ori_lfoot = des_lfoot_ori_quat.normalized().coeffs();
      //des_ori_torso = des_rfoot_ori_quat.normalized().coeffs();
      des_ori_lfoot = des_ori_torso;
      des_ori_rfoot = des_ori_torso;
    }

}

void AlipMpcTrajectoryManager::outsideCommand(const YAML::Node &node){
    util::ReadParameter(node, "Lx_offset", indata.Lx_offset);
    util::ReadParameter(node, "Ly_des", indata.Ly_des);
    util::ReadParameter(node, "com_yaw", com_yaw);
    util::ReadParameter(node, "swing_height", swing_height);
    util::ReadParameter(node, "mu", indata.mu);
    util::ReadParameter(node, "kx", indata.kx);
    util::ReadParameter(node, "ky", indata.ky);


    com_yaw *= M_PI/180;

}

void AlipMpcTrajectoryManager::MpcSolutions(const double &tr_, const double &st_leg) {  //generate footsteps and COM pos
  indata.Tr = tr_;
  //indata.kx = 0;
  //indata.ky = 0;
  //indata.mu = 0.3;
  indata.stance_leg = st_leg;


  this->InertiaToMpcCoordinates();

   //after we will put Lx and Ly as inputs of the function
  
  

  alipMpc->Update_(indata, outdata, fullsol);


  this->OutputMpcToInertiaCoordinates();
}

//use torso horientazion with make horizontal
void AlipMpcTrajectoryManager::InertiaToMpcCoordinates(){
    Eigen::Vector3d pos = robot_->GetRobotComPos();

    //Link coordinates = Rotation*inertial coordinates + position; for now just position implemented
    //we are going to use torso orientation as mpc orientation
    if(indata.stance_leg == 1){  //stance leg is right
      stleg_pos << robot_->GetLinkIsometry(draco_link::r_foot_contact).translation();
    }
    else {   //stance leg is left
      stleg_pos << robot_->GetLinkIsometry(draco_link::l_foot_contact).translation();
    }
    terrain = Eigen::Vector3d(-indata.kx, -indata.ky, 1);
    terrain /= (terrain.norm());

    //Change the actual torso ori with desired torso ori
    //Eigen::Isometry3d torso_iso = robot_->GetLinkIsometry(draco_link::torso_link);
    //FootStep::MakeHorizontal(torso_iso);  
    
    Eigen::Vector3d Lc = robot_->GetHg().head<3>();
    std::cout << "Lc" << " " << Lc << std::endl;
    Eigen::Vector3d rotpos = des_torso_iso.linear().transpose() * pos;
    Eigen::Vector3d stleg_pos_torso_ori =  des_torso_iso.linear().transpose() * stleg_pos;

    indata.xlip_current[0] = rotpos(0)-stleg_pos_torso_ori(0);
    indata.xlip_current[1] = rotpos(1)-stleg_pos_torso_ori(1);
    double indataxlip_current_z = rotpos(2) - stleg_pos_torso_ori(2);

    /*
    indata.zH = terrain(0)*indata.xlip_current[0];
    indata.zH += terrain(1)*indata.xlip_current[1];
    indata.zH += terrain(2)*(rotpos(2)-stleg_pos_torso_ori(2));
    */
    indata.zH = refzH;

    pos = Eigen::Vector3d(indata.xlip_current[0],indata.xlip_current[1],indata.zH);
    pos = Eigen::Vector3d(indata.xlip_current[0],indata.xlip_current[1],indataxlip_current_z);
    Eigen::Vector3d vel = robot_->GetRobotComLinVel();   //check? the velocity frame needs to be aligned with the foot frame. 
                                                        //now it is aligned with the inertia frame. Maybe a rotation of robot pos and vel is needed.
    vel = des_torso_iso.linear().transpose() * vel;   //we are assuming that the rotation matrix doens't change with time

    Eigen::Vector3d L = pos.cross(mass*vel);
    L += Lc;
    indata.xlip_current[2]= L[0];             
    indata.xlip_current[3]= L[1];
    indataLz = L[2];
    std::cout << "L " << L << std::endl;


}


void AlipMpcTrajectoryManager::OutputMpcToInertiaCoordinates(){
  //COM
  double x_end = fullsol.xlip_sol[0];
  double y_end = fullsol.xlip_sol[1];


  double z_end = this->ComputeZpos(x_end, y_end, indata.zH);
  COM_end << x_end,y_end,z_end;



  Eigen::Matrix3d A; // L = r x mv = m*A(r)v  ---> v = 1/m * A^-1 * r
  A << 0,-z_end,y_end,  
       z_end,0,-x_end,   
       -y_end,x_end,0;
  A = mass*A;
  Eigen::Vector3d L_end;
  L_end << fullsol.xlip_sol[2],fullsol.xlip_sol[3],0;   //Assume that Lz = 0, that means the robot shouldn't turn (will have to change this)

  COMvel_end = A.colPivHouseholderQr().solve(L_end);  //with this we are assuming that feet of reference doesn't suffer rotations

  //Eigen::Isometry3d torso_iso = robot_->GetLinkIsometry(draco_link::torso_link);
  //FootStep::MakeHorizontal(torso_iso);  
  //COMvel_end = torso_iso * COMvel_end;
  
  COMvel_end = des_torso_iso * COMvel_end;


  double sw_end_x = outdata.ufp_wrt_st[0];
  double sw_end_y = outdata.ufp_wrt_st[1];


  double sw_end_z = this->ComputeZpos(sw_end_x, sw_end_y, 0); 
  Swingfoot_end << sw_end_x, sw_end_y, sw_end_z;


  COM_end += Swingfoot_end + stleg_pos_torso_ori;
  //COM_end = torso_iso.linear() * COM_end;
  //Swingfoot_end = torso_iso.linear() * Swingfoot_end;

  COM_end = des_torso_iso.linear() * COM_end;
  Swingfoot_end = des_torso_iso.linear() * Swingfoot_end;
  //double ftz_end = this->ComputeZpos(outdata.ufp_wrt_st[0] + stleg_pos(0), outdata.ufp_wrt_st[1] + stleg_pos(1), 0);
  //Swingfoot_end << outdata.ufp_wrt_st[0] + stleg_pos(0), outdata.ufp_wrt_st[1] + stleg_pos(1), ftz_end;
  Swingfoot_end = Swingfoot_end + stleg_pos;

  Swingfoot_end(2) = this->ComputeZpos(Swingfoot_end(0), Swingfoot_end(1), 0);
}


void AlipMpcTrajectoryManager::GenerateSwingFtraj(const double &tr_){
  indata.Tr = tr_;
  Eigen::Isometry3d curr_swfoot_iso;
  if (indata.stance_leg == 1){  //LF is swing foot
    curr_swfoot_iso = robot_->GetLinkIsometry(draco_link::l_foot_contact);  //chequear si contact solo funciona cuando es stance foot
    FootStep::MakeHorizontal(curr_swfoot_iso);          
  }
  else {
    curr_swfoot_iso = robot_->GetLinkIsometry(draco_link::r_foot_contact);
    FootStep::MakeHorizontal(curr_swfoot_iso);
  }

  Eigen::Vector3d curr_swfoot_pos = curr_swfoot_iso.translation();

  Eigen::Vector3d curr_swfoot_linearVel = robot_->GetLinkSpatialVel(draco_link::l_foot_contact).tail<3>();
  //for now we will assume that the feet moves at an horizontal velocity that is constant in time
  //so we can compute the middle foot position proportionally to the time remaining
  

  if(variable_height){
    Eigen::Vector2d vectdist = (Swingfoot_end - Swingfoot_start).head<2>();
    double step_distance = vectdist.norm();
    swing_height = step_distance*reference_swing_height/0.5;
    if (swing_height < 0.025) {  
      swing_height = 0.025;   
    }
  }
  if (Alip) AlipSwingPos = new AlipSwing(Swingfoot_start, Swingfoot_end, swing_height, indata.Ts);
  AlipSwingPos2 = new AlipSwing2(Swingfoot_start, Swingfoot_end, swing_height, indata.Ts);

  
  if(indata.Tr > 0.5*indata.Ts){  //pensar en como calcular la midfoot position

    Eigen::Vector3d mid_pos;
    double t1 = indata.Tr - 0.5*indata.Ts ;
    mid_pos << (curr_swfoot_pos + Swingfoot_end)*t1/(indata.Tr);  //could be problematic if indata.Tr is verry small
    mid_pos(2) = swing_height;

    Eigen::Vector3d mid_linearVel;
    mid_linearVel = (Swingfoot_end-curr_swfoot_pos)/indata.Tr;
    mid_linearVel(2) = 0;
    //Bezier is just a test for know, will have to change plenty of things
    //can have a problem when Tr < 0.5, i don't want to do hermite 

    BezierSwingPos = new QuadraticBezierCurve(curr_swfoot_pos, mid_pos, Swingfoot_end, indata.Tr);

    first_half_curve_SwingPos = 
      new HermiteCurveVec(curr_swfoot_pos, curr_swfoot_linearVel, mid_pos, mid_linearVel, t1);
    second_half_curve_SwingPos = 
      new HermiteCurveVec(mid_pos, mid_linearVel, Swingfoot_end, Swingfootvel_end, 0.5*indata.Ts);
  }
  else{
    second_half_curve_SwingPos = 
      new HermiteCurveVec(curr_swfoot_pos, curr_swfoot_linearVel, Swingfoot_end, Swingfootvel_end, indata.Tr);
  }
}





void AlipMpcTrajectoryManager::UpdateDesired(const double t){  
  //t is the time since last evaluation of Mpc and trajectory was generated so it's the time since Tr was coomputed
  //not sure works fine for every trajectory right now
  //only works for alip2swing

  Eigen::Isometry3d curr_swfoot_iso;
  if (indata.stance_leg == 1){  //LF is swing foot
    curr_swfoot_iso = robot_->GetLinkIsometry(draco_link::l_foot_contact);  //chequear si contact solo funciona cuando es stance foot
    FootStep::MakeHorizontal(curr_swfoot_iso);          
  }
  else {
    curr_swfoot_iso = robot_->GetLinkIsometry(draco_link::r_foot_contact);
    FootStep::MakeHorizontal(curr_swfoot_iso);
  }
  //file7 << curr_swfoot_iso.translation().transpose() << "  " << std::endl;

  if (Alip){ //xy sinusoid; z is two splines
    des_swfoot_pos = AlipSwingPos->Evaluate(t);
    des_swfoot_vel = AlipSwingPos->EvaluateFirstDerivative(t);
    des_swfoot_acc = AlipSwingPos->EvaluateSecondDerivative(t);  
  }
  else if (Alip2){ //xy sinusoid; z is quadratic interpolation
    des_swfoot_pos = AlipSwingPos2->Evaluate(t);
    des_swfoot_vel = AlipSwingPos2->EvaluateFirstDerivative(t);
    des_swfoot_acc = AlipSwingPos2->EvaluateSecondDerivative(t);
  }
  else if(Bezier){ 
    des_swfoot_pos = BezierSwingPos->Evaluate(t);
    des_swfoot_vel = BezierSwingPos->EvaluateFirstDerivative(t);
    des_swfoot_acc = BezierSwingPos->EvaluateSecondDerivative(t);
  }
  else { //two splines 
    if (indata.Tr > 0.5*indata.Ts){
      if (indata.Tr-t > 0.5*indata.Ts){
          des_swfoot_pos = first_half_curve_SwingPos->Evaluate(t);
          des_swfoot_vel = first_half_curve_SwingPos->EvaluateFirstDerivative(t);
          des_swfoot_acc = first_half_curve_SwingPos->EvaluateSecondDerivative(t);

      }
      else {
          des_swfoot_pos = second_half_curve_SwingPos->Evaluate(t-(indata.Tr-0.5*indata.Ts));
          des_swfoot_vel = second_half_curve_SwingPos->EvaluateFirstDerivative(t-(indata.Tr-0.5*indata.Ts));
          des_swfoot_acc = second_half_curve_SwingPos->EvaluateSecondDerivative(t-(indata.Tr-0.5*indata.Ts));
      }
    }
    else {
      des_swfoot_pos = second_half_curve_SwingPos->Evaluate(t);
      des_swfoot_vel = second_half_curve_SwingPos->EvaluateFirstDerivative(t);
      des_swfoot_acc = second_half_curve_SwingPos->EvaluateSecondDerivative(t);
    }

  }

  //in single_support_swing update both the tasks
  //for the stance leg they use UseCurrent. Is it really necessary
  //it would mean that tasks are erased at each controller iteration
  if (indata.stance_leg == 1){ //update left
 
    lfoot_task->UpdateDesired(des_swfoot_pos, des_swfoot_vel, des_swfoot_acc);
    this->UpdateCurrentPos(rfoot_task);


    //Set hierarchy weights
    com_xy_task->SetWeight(com_xy_task_weight);
    com_z_task->SetWeight(com_z_task_weight);
    rfoot_task->SetWeight(stance_foot_weight);
    lfoot_task->SetWeight(swing_foot_weight);
    lfoot_ori->SetWeight(swing_foot_ori_weight);
    rfoot_ori->SetWeight(stance_foot_ori_weight);
  }
  else { //update right
    rfoot_task->UpdateDesired(des_swfoot_pos, des_swfoot_vel, des_swfoot_acc);
    this->UpdateCurrentPos(lfoot_task);


    //set hierarchy weights
    com_xy_task->SetWeight(com_xy_task_weight);
    com_z_task->SetWeight(com_z_task_weight);
    rfoot_task->SetWeight(swing_foot_weight);
    lfoot_task->SetWeight(stance_foot_weight);
    rfoot_ori->SetWeight(swing_foot_ori_weight);
    lfoot_ori->SetWeight(stance_foot_ori_weight);
    torso_ori->SetWeight(torso_ori_weight);

  }
  
  Eigen::VectorXd Z(1);
  Z << refzH;
  com_z_task->UpdateDesired(Z, Eigen::VectorXd::Zero(1),
                            Eigen::VectorXd::Zero(1));
                            


  Eigen::VectorXd zero3 = Eigen::VectorXd::Zero(3);
  torso_ori->UpdateDesired(des_ori_torso, zero3, zero3);
  rfoot_ori->UpdateDesired(des_ori_rfoot, zero3, zero3);
  lfoot_ori->UpdateDesired(des_ori_lfoot, zero3, zero3);

}


void AlipMpcTrajectoryManager::UpdateCurrentOri(Task* task){
  Eigen::Quaterniond des_ori_quat(
    robot_->GetLinkIsometry(task->TargetIdx()).linear());
  Eigen::VectorXd des_ori(4);

  des_ori << des_ori_quat.normalized().coeffs();

  Eigen::VectorXd des_ang_vel = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd des_acc = Eigen::VectorXd::Zero(3);
  
  task->UpdateDesired(des_ori, des_ang_vel, des_acc);
}


void AlipMpcTrajectoryManager::UpdateCurrentPos(Task* task){
  Eigen::VectorXd des_pos(3);
  des_pos << robot_->GetLinkIsometry(task->TargetIdx()).translation();

  Eigen::VectorXd des_vel = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd des_acc = Eigen::VectorXd::Zero(3);

  task->UpdateDesired(des_pos, des_vel, des_acc);
}

//will assume that kx and ky are true in the actual frame of reference 
//need a high level controller or perception to adjust kx and ky when turning on incline plane
//normal vector to ground is (-kx, -ky, 1)
void AlipMpcTrajectoryManager::MakeParallelToGround(Eigen::Isometry3d &pose){
  FootStep::MakeHorizontal(pose);
  const Eigen::Matrix3d R = pose.linear();
  //const Eigen::Vector3d p = pose.translation(); don't change
  double kx = indata.kx;
  double ky = indata.ky;
  double c1 = 1/sqrt(1 + kx*kx);
  double c2 = 1/sqrt(1 + kx*kx + ky*ky);
  Eigen::Matrix3d T;
  T <<  c1  ,   -c1*c2*kx*ky   , -c2*kx,
         0  , c1*c2*(kx*kx + 1), -c2*ky,
       c1*kx,     c1*c2*ky     ,   c2;
  pose.linear() = T*R;
}

double AlipMpcTrajectoryManager::ComputeZpos(const double &x, const double &y, const double &zH_){
  return indata.kx*x + indata.ky*y + zH_;
}



void AlipMpcTrajectoryManager::saveRobotCommand(const double t){
  file9 << des_swfoot_pos.transpose() << " " << des_swfoot_vel.transpose() << " " << des_swfoot_acc.transpose();
  file9 << " " << t << endl;
}



void AlipMpcTrajectoryManager::saveMpcCOMstate(const double t){
  file1 << indata.xlip_current[0] << " " << indata.xlip_current[1] << " " << indata.zH << "  ";
  file1 << indata.xlip_current[2] << " " << indata.xlip_current[3] << " " << indataLz << " ";
  file1 << t << " " << indata.Lx_offset << " " << indata.Ly_des << " ";
  file1 << indata.mu << " "  << indata.leg_width << " " << indata.zH << " ";
  file1 << mass << " " << indata.Ts <<  " ";
  file1 << fullsol.xlip_sol[0] << " " << fullsol.xlip_sol[1] << " " << fullsol.xlip_sol[2] << " ";
  file1 << fullsol.xlip_sol[3] << endl;
}


void AlipMpcTrajectoryManager::saveCurrentCOMstate(const double t){
  Eigen::Vector3d pos = robot_->GetRobotComPos();
  Eigen::Vector3d vel = robot_->GetRobotComLinVel();
  Eigen::Vector3d posWorld = pos;

  Eigen::Isometry3d torso_iso = robot_->GetLinkIsometry(draco_link::torso_link);
  FootStep::MakeHorizontal(torso_iso);  

  pos = torso_iso.linear().transpose() * pos;
  Eigen::Vector3d stleg_pos_torso_ori =  torso_iso.linear().transpose() * stleg_pos;

  pos -= stleg_pos_torso_ori;
  vel = torso_iso.linear().transpose() * vel;
  Eigen::Vector3d L = pos.cross(mass*vel);
  Eigen::Vector3d LCOM = robot_->GetHg().head<3>();

  file8 << pos.transpose() << "  ";
  file8 << L.transpose() << " ";
  file8 << t <<  " " << posWorld.transpose() << " ";
  file8 << stleg_pos.transpose() << " ";
  file8 << vel.transpose() << " ";
  file8 << LCOM.transpose() << endl;
}

void AlipMpcTrajectoryManager::saveSwingState(const double t){
  Eigen::Isometry3d curr_swfoot_iso;
  Eigen::Matrix<double, 6, 1> curr_swfoot_vel;
  if (indata.stance_leg == 1){  //LF is swing foot
    curr_swfoot_iso = robot_->GetLinkIsometry(draco_link::l_foot_contact);  //chequear si contact solo funciona cuando es stance foot
    FootStep::MakeHorizontal(curr_swfoot_iso);  
    curr_swfoot_vel = robot_->GetLinkBodySpatialVel(draco_link::l_foot_contact);        
  }
  else {
    curr_swfoot_iso = robot_->GetLinkIsometry(draco_link::r_foot_contact);
    FootStep::MakeHorizontal(curr_swfoot_iso);
    curr_swfoot_vel = robot_->GetLinkBodySpatialVel(draco_link::r_foot_contact);

  }
  file7 << curr_swfoot_iso.translation().transpose() << "  " << t << " " << 
           curr_swfoot_vel[3, 0] << " " <<  curr_swfoot_vel[4, 0] << " " << curr_swfoot_vel[5, 0] << std::endl;
}


void AlipMpcTrajectoryManager::saveTrajectories(const double start_time, const double dt,const double end_time){
/*
  std::fstream file1;
  std::fstream file2;
  std::fstream file3;
  std::fstream file4;


  file1.open(THIS_COM "/test/alip/alip_COM_trajectory.txt", ios::out);
  file2.open(THIS_COM "/test/alip/Swing1_trajectory.txt", ios::out);
  file3.open(THIS_COM "/test/alip/Swing2_trajectory.txt", ios::out);
  file4.open(THIS_COM "/test/alip/BezierSwing_trajectory.txt", ios::out);
*/
  saveCounter++;
  if (!file2.is_open() || !file3.is_open() || !file4.is_open() || !file5.is_open() || !file6.is_open()) {
      std::cerr << "Error: Unable to open one or more files for writing." << std::endl;
      return;
  }
  file2 <<"start------"<< saveCounter << "------------------------" << std::endl;
  file3 <<"start------"<< saveCounter << "------------------------" << std::endl;
  file4 <<"start------"<< saveCounter << "------------------------" << std::endl;
  file5 <<"start------"<< saveCounter << "------------------------" << std::endl;
  file6 <<"start------"<< saveCounter << "------------------------" << std::endl;


  for (double t = start_time; t <= end_time; t += dt){
    Eigen::VectorXd Swing_part1 = first_half_curve_SwingPos->Evaluate(t/2);
    Eigen::VectorXd Swing_part2 = second_half_curve_SwingPos->Evaluate(t/2);
    Eigen::VectorXd BezierSwing = BezierSwingPos->Evaluate(t);
    //Eigen::Vector3d SwingAlip = AlipSwingPos->Evaluate(t);
    Eigen::Vector3d SwingAlip2 = AlipSwingPos2->Evaluate(t);


    file2 << Swing_part1.transpose() << "  " << t << std::endl;
    file3 << Swing_part2.transpose() << "  " << t <<  std::endl;
    file4 << BezierSwing.transpose() << "  " << t <<  std::endl;
    //file5 << SwingAlip.transpose() << "  " << t <<  std::endl;
    file6 << SwingAlip2.transpose() << "  " << t <<  std::endl;



  }
  file2 <<"end------"<< saveCounter << "------------------------" << std::endl;
  file3 <<"end------"<< saveCounter << "------------------------" << std::endl;
  file4 <<"end------"<< saveCounter << "------------------------" << std::endl;
  file5 <<"end------"<< saveCounter << "------------------------" << std::endl;
  file6 <<"end------"<< saveCounter << "------------------------" << std::endl;

/*
  file1.close();
  file2.close();
  file3.close();
  file4.close();
*/

}



void AlipMpcTrajectoryManager::SetParameters(const YAML::Node &node) {
  try {
    util::ReadParameter(node, "Ts", indata.Ts);
    util::ReadParameter(node, "zH", refzH);
    util::ReadParameter(node, "leg_width", indata.leg_width);
    util::ReadParameter(node, "total_mass", mass);
    util::ReadParameter(node, "Lx_offset", indata.Lx_offset);
    util::ReadParameter(node, "Ly_des", indata.Ly_des);
    util::ReadParameter(node, "Bezier", Bezier);
    util::ReadParameter(node, "Alip", Alip);
    util::ReadParameter(node, "Alip2", Alip2);
    util::ReadParameter(node, "variable_height", variable_height);
    util::ReadParameter(node, "reference_swing_height", reference_swing_height);
    util::ReadParameter(node, "swing_height", swing_height);

  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }
}



void AlipMpcTrajectoryManager::SetTaskWeights(const YAML::Node &node){
  try {
    util::ReadParameter(node, "com_z_pos", com_z_task_weight);
    util::ReadParameter(node, "com_xy_pos", com_xy_task_weight);
    util::ReadParameter(node, "torso_ori", torso_ori_weight);
    util::ReadParameter(node, "swing_foot_weight", swing_foot_weight);
    util::ReadParameter(node, "stance_foot_weight", stance_foot_weight);
    util::ReadParameter(node, "stance_foot_ori_weight", stance_foot_ori_weight);
    util::ReadParameter(node, "swing_foot_ori_weight", swing_foot_ori_weight);


  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

void printInData(input_data_t* data) {
    //printf("time: %f\n", data->time);
    //printf("s: %f\n", data->s);
    printf("x_c: %f\n", data->xlip_current[0]);
    printf("y_c: %f\n", data->xlip_current[1]);
    printf("L_xc: %f\n", data->xlip_current[2]);
    printf("L_yc: %f\n", data->xlip_current[3]);
    printf("stance_leg: %f\n", data->stance_leg);
    printf("zH: %f\n", data->zH);
    printf("Ts: %f\n", data->Ts);
    printf("Tr: %f\n", data->Tr);
   // printf("leg_width: %f\n", data->leg_width);
    printf("Lx_offset: %f\n", data->Lx_offset);
    printf("Ly_des: %f\n", data->Ly_des);
    //printf("kx: %f\n", data->kx);
    //printf("ky: %f\n", data->ky);
    //printf("mu: %f\n", data->mu);
    printf("---\n");
}