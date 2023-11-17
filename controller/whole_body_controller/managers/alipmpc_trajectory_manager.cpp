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
      COMpos(nullptr), first_half_curve_SwingPos(nullptr), second_half_curve_SwingPos(nullptr) {
  
  util::PrettyConstructor(2, "AlipMpcTrajectoryManager");
  printCounter = 5;
  first_ever = false;



  indata.xlip_current[2] = 0;
  indata.xlip_current[3] = 0; 
  saveCounter = 0;
  ///*
  file1.open(THIS_COM "/test/alip/alip_COM_trajectory.txt", std::fstream::in | std::fstream::out | std::fstream::app);
  file2.open(THIS_COM "/test/alip/Swing1_trajectory.txt", std::fstream::in | std::fstream::out | std::fstream::app);
  file3.open(THIS_COM "/test/alip/Swing2_trajectory.txt", std::fstream::in | std::fstream::out | std::fstream::app);
  file4.open(THIS_COM "/test/alip/BezierSwing_trajectory.txt", std::fstream::in | std::fstream::out | std::fstream::app);
  //*/
  /*
  file1.open(THIS_COM "/test/alip/alip_COM_trajectory.txt", std::fstream::out);
  file2.open(THIS_COM "/test/alip/Swing1_trajectory.txt", std::fstream::out);
  file3.open(THIS_COM "/test/alip/Swing2_trajectory.txt", std::fstream::out);
  file4.open(THIS_COM "/test/alip/BezierSwing_trajectory.txt", std::fstream::out);
*/

}  //need to add ori 

void AlipMpcTrajectoryManager::firstVisit(){
  if (not first_ever) {
    Eigen::Quaterniond des_ori_quat_lfoot(robot_->GetLinkIsometry(draco_link::l_foot_contact).linear());
    Eigen::Quaterniond des_ori_quat_rfoot(robot_->GetLinkIsometry(draco_link::r_foot_contact).linear());
    des_ori_lfoot = des_ori_quat_lfoot.normalized().coeffs();
    des_ori_rfoot = des_ori_quat_rfoot.normalized().coeffs();
    Eigen::Quaterniond des_torso_ori_quat(robot_->GetLinkIsometry(draco_link::torso_com_link).linear());
    des_ori_torso = des_torso_ori_quat.normalized().coeffs();



    first_ever = true;
  }
}



void AlipMpcTrajectoryManager::MpcSolutions(const double &tr_, const double &st_leg) {  //generate footsteps and COM pos
  indata.Tr = tr_;
  indata.kx = 0;
  indata.ky = 0;
  indata.mu = 0.3;
  indata.stance_leg = st_leg;
  //I need to calculate indata.Tr so I would need a current time
  cout << "indata stance_leg : " <<indata.stance_leg << endl;
  util::PrettyConstructor(3, "AlipMpc tm : Mpc Solutions");

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

    Eigen::Isometry3d torso_iso = robot_->GetLinkIsometry(draco_link::torso_link);
    FootStep::MakeHorizontal(torso_iso);  

    Eigen::Vector3d rotpos = torso_iso.linear().transpose() * pos;
    Eigen::Vector3d stleg_pos_torso_ori =  torso_iso.linear().transpose() * stleg_pos;

    indata.xlip_current[0] = rotpos(0)-stleg_pos_torso_ori(0);
    indata.xlip_current[1] = rotpos(1)-stleg_pos_torso_ori(1);

    pos = Eigen::Vector3d(indata.xlip_current[0],indata.xlip_current[1],indata.zH);
    Eigen::Vector3d vel = robot_->GetRobotComLinVel();   //check? the velocity frame needs to be aligned with the foot frame. 
                                                        //now it is aligned with the inertia frame. Maybe a rotation of robot pos and vel is needed.
    vel = torso_iso.linear().transpose() * vel;   //we are assuming that the rotation matrix doens't change with time
    
    Eigen::Vector3d L = pos.cross(mass*vel);
    indata.xlip_current[2]= L[0];             //maybe write a getter function in PInocchioRobotSystem to compute L = rxmv
    indata.xlip_current[3]= L[1];

    //indata.zH = pos(2)-stleg_pos(2);  //check for that the z position of stleg wrt COM should be zH//here zH is changing maybe try static zH
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

  Eigen::Isometry3d torso_iso = robot_->GetLinkIsometry(draco_link::torso_link);
  FootStep::MakeHorizontal(torso_iso);  
  COMvel_end = torso_iso * COMvel_end;

  double sw_end_x = outdata.ufp_wrt_st[0];
  double sw_end_y = outdata.ufp_wrt_st[1];
  double sw_end_z = this->ComputeZpos(sw_end_x, sw_end_y, 0); 

  Swingfoot_end << sw_end_x, sw_end_y, sw_end_z;


  COM_end += Swingfoot_end + stleg_pos_torso_ori;
  COM_end = torso_iso.linear() * COM_end;
  Swingfoot_end = torso_iso.linear() * Swingfoot_end;

  //double ftz_end = this->ComputeZpos(outdata.ufp_wrt_st[0] + stleg_pos(0), outdata.ufp_wrt_st[1] + stleg_pos(1), 0);
  //Swingfoot_end << outdata.ufp_wrt_st[0] + stleg_pos(0), outdata.ufp_wrt_st[1] + stleg_pos(1), ftz_end;
  Swingfoot_end = Swingfoot_end + stleg_pos;

  Swingfoot_end(2) = this->ComputeZpos(Swingfoot_end(0), Swingfoot_end(1), 0);

  cout << "st leg pos" << stleg_pos << endl;
  cout << "swing leg start" << robot_->GetLinkIsometry(draco_link::l_foot_contact).translation() << endl;
  cout << "SWINg foot end " << Swingfoot_end << endl;
  cout << "sol ufp " << outdata.ufp_wrt_st[0] << " " << outdata.ufp_wrt_st[1] << endl;
  cout << "sol com " << fullsol.xlip_sol[0] << " " << fullsol.xlip_sol[1] << endl;

}


void AlipMpcTrajectoryManager::GenerateCOMtraj(const double &tr_){
  //i need velocity instead of angular momentum 
  //to create an hermite curve
  util::PrettyConstructor(3,"Generate traj");
  indata.Tr = tr_;
  Eigen::Vector3d start_vel = robot_->GetRobotComLinVel();//will manually put 0 in the z component
  start_vel(2) = 0;
  Eigen::Vector3d end_vel = COMvel_end;  //have to set to 0 the z component because we don't want to change the robot COM height
  end_vel(2) = 0;

  //Eigen::Vector3d end_vel = COMvel_end -
  COMpos = 
    new HermiteCurveVec (robot_->GetRobotComPos(), robot_->GetRobotComLinVel(), COM_end, COMvel_end, indata.Tr);
}


void AlipMpcTrajectoryManager::GenerateSwingFtraj(){
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
  cout << "curr swfooot pos generation " << endl << curr_swfoot_pos << endl;

  Eigen::Vector3d curr_swfoot_linearVel = robot_->GetLinkSpatialVel(draco_link::l_foot_contact).tail<3>();
  //for now we will assume that the feet moves at an horizontal velocity that is constant in time
  //so we can compute the middle foot position proportionally to the time remaining
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
    SwingPos = new QuadraticBezierCurve(curr_swfoot_pos, mid_pos, Swingfoot_end, indata.Tr);

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


double AlipMpcTrajectoryManager::ComputeZpos(const double &x, const double &y, const double &zH_){
  return indata.kx*x + indata.ky*y + zH_;
}


void AlipMpcTrajectoryManager::UpdateDesired(const double t){  //have to be carefull when choosing t
  //std::cout << "Tr: " << indata.Tr << " t: " << t << " || ";
  
  //t will be the time from Tr. Tr is the remaining time when the curve was computing
  //so Tr-t is the remaining time when the curve is evaluated
  assert(first_half_curve_SwingPos != nullptr);
  assert(second_half_curve_SwingPos != nullptr);
  assert(COMpos != nullptr); 
  //assert(ori_curve_ != nullptr);

  Eigen::VectorXd des_COM_pos = COMpos->Evaluate(t);
  Eigen::VectorXd des_COM_vel = COMpos->EvaluateFirstDerivative(t);
  Eigen::VectorXd des_COM_acc = COMpos->EvaluateSecondDerivative(t);
  //Eigen::VectorXd des_COM_acc = Eigen::VectorXd::Zero(3);
  //look why i have a task for xy and another for z
  //do update


  Eigen::VectorXd des_swfoot_pos;
  Eigen::VectorXd des_swfoot_vel;
  Eigen::VectorXd des_swfoot_acc;

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




  //in single_support_swing update both the tasks
  //for the stance leg they use UseCurrent. Is it really necessary
  //it would mean that tasks are erased at each controller iteration
  if (indata.stance_leg == 1){ //update left
 
    lfoot_task->UpdateDesired(des_swfoot_pos, des_swfoot_vel, des_swfoot_acc);
    this->UpdateCurrentPos(rfoot_task);
    rfoot_task->SetWeight(Eigen::Vector3d(5000, 5000,5000));
    lfoot_task->SetWeight(Eigen::Vector3d(8000, 8000,8000));
  }
  else { //update right
    rfoot_task->UpdateDesired(des_swfoot_pos, des_swfoot_vel, des_swfoot_acc);
    this->UpdateCurrentPos(lfoot_task);
    rfoot_task->SetWeight(Eigen::Vector3d(8000, 8000,8000));
    lfoot_task->SetWeight(Eigen::Vector3d(5000, 5000,5000));

  }

  
  com_z_task->UpdateDesired(des_COM_pos.tail<1>(), Eigen::VectorXd::Zero(1),
                            Eigen::VectorXd::Zero(1));  //CHECK FOR this. should stay at same height. 

  

  //ori for now use the current orientation. Don't change it


  lfoot_ori->SetWeight(Eigen::Vector3d(500, 100, 10));
  rfoot_ori->SetWeight(Eigen::Vector3d(500, 100, 10));
  torso_ori->SetWeight(Eigen::Vector3d(10000, 1000, 100));

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

/*
void AlipMpcTrajectoryManager::RToLstance(){
      //left force to 500 and right to 0
      Eigen::VectorXd wrong;
      wrong << 0,0,0,0,0,mass*9.8;
      Eigen::VectorXd wrong2 = Eigen::VectorXd::Zero(6);
      rg_force_task->UpdateDesired(wrong2);
      lf_force_task->UpdateDesired(wrong);
}


void AlipMpcTrajectoryManager::LToRstance(){
        //left force to 500 and right to 0
      Eigen::VectorXd wrong;
      wrong << 0,0,0,0,0,mass*9.8;
      Eigen::VectorXd wrong2 = Eigen::VectorXd::Zero(6);
      rg_force_task->UpdateDesired(wrong);
      lf_force_task->UpdateDesired(wrong2);
}

*/


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
  if (!file1.is_open() || !file2.is_open() || !file3.is_open() || !file4.is_open()) {
      std::cerr << "Error: Unable to open one or more files for writing." << std::endl;
      return;
  }
  file1 <<"start------"<< saveCounter << "------------------------" << std::endl;
  file2 <<"start------"<< saveCounter << "------------------------" << std::endl;
  file3 <<"start------"<< saveCounter << "------------------------" << std::endl;
  file4 <<"start------"<< saveCounter << "------------------------" << std::endl;

  for (double t = start_time; t < end_time; t += dt){
    Eigen::VectorXd COM = COMpos->Evaluate(t);
    Eigen::VectorXd Swing_part1 = first_half_curve_SwingPos->Evaluate(t/2);
    Eigen::VectorXd Swing_part2 = second_half_curve_SwingPos->Evaluate(t/2);
    Eigen::VectorXd BezierSwing = SwingPos->Evaluate(t);

    file1 << COM.transpose() << "  " << t << std::endl;
    file2 << Swing_part1.transpose() << "  " << t << std::endl;
    file3 << Swing_part2.transpose() << "  " << t <<  std::endl;
    file4 << BezierSwing.transpose() << "  " << t <<  std::endl;

  }
  file1 <<"end------"<< saveCounter << "------------------------" << std::endl;
  file2 <<"end------"<< saveCounter << "------------------------" << std::endl;
  file3 <<"end------"<< saveCounter << "------------------------" << std::endl;
  file4 <<"end------"<< saveCounter << "------------------------" << std::endl;

/*
  file1.close();
  file2.close();
  file3.close();
  file4.close();
*/

}

void AlipMpcTrajectoryManager::SetParameters(const YAML::Node &node) {
  try {
    util::ReadParameter(node, "swing_height", swing_height);
    util::ReadParameter(node, "Ts", indata.Ts);
    util::ReadParameter(node, "zH", indata.zH);
    util::ReadParameter(node, "leg_width", indata.leg_width);
    util::ReadParameter(node, "total_mass", mass);
    util::ReadParameter(node, "Lx_offset", indata.Lx_offset);
    util::ReadParameter(node, "Ly_des", indata.Ly_des);


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