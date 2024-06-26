#include "controller/draco_controller/draco_definition.hpp"
#include "controller/model_predictive_controller/lmpc/lmpc_handler.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "controller/whole_body_controller/force_task.hpp"
#include "planner/locomotion/dcm_planner/dcm_planner.hpp"

int main() {
  PinocchioRobotSystem robot(THIS_COM "robot_model/draco/draco_modified.urdf",
                             THIS_COM "robot_model/draco", false, false);
  DCMPlanner dcm_planner;
  // dummy task
  Task *com_task(NULL);
  Task *torso_ori_task(NULL);
  ForceTask *lfoot_rf_task(NULL);
  ForceTask *rfoot_rf_task(NULL);

  LMPCHandler lmpc_handler(
      &dcm_planner, &robot, com_task, torso_ori_task, lfoot_rf_task,
      rfoot_rf_task, draco_link::l_foot_contact, draco_link::r_foot_contact);

  // initialize robot
  Eigen::Vector3d bjoint_pos(0, 0, 1.5 - 0.757);
  Eigen::Quaterniond bjoint_quat(0.7071, 0, 0, 0.7071);
  Eigen::Vector3d bjoint_lv(0.1, 0, 0);
  Eigen::Vector3d bjoint_av(0.1, 0, 0);

  Eigen::VectorXd joint_pos = Eigen::VectorXd::Zero(draco::n_adof);
  Eigen::VectorXd joint_vel = Eigen::VectorXd::Zero(draco::n_adof);
  joint_pos[draco_joint::l_hip_fe] = -M_PI / 4;
  joint_pos[draco_joint::l_knee_fe_jp] = M_PI / 4;
  joint_pos[draco_joint::l_knee_fe_jd] = M_PI / 4;
  joint_pos[draco_joint::l_ankle_fe] = -M_PI / 4;
  joint_pos[draco_joint::l_shoulder_aa] = M_PI / 6;
  joint_pos[draco_joint::l_elbow_fe] = -M_PI / 2;

  joint_pos[draco_joint::r_hip_fe] = -M_PI / 4;
  joint_pos[draco_joint::r_knee_fe_jp] = M_PI / 4;
  joint_pos[draco_joint::r_knee_fe_jd] = M_PI / 4;
  joint_pos[draco_joint::r_ankle_fe] = -M_PI / 4;
  joint_pos[draco_joint::r_shoulder_aa] = -M_PI / 6;
  joint_pos[draco_joint::r_elbow_fe] = -M_PI / 2;

  robot.UpdateRobotModel(bjoint_pos, bjoint_quat.normalized(), bjoint_lv,
                         bjoint_av, joint_pos, joint_vel, true);

  // dcm tm initialize
  YAML::Node cfg = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");
  lmpc_handler.InitializeParameters(cfg["lmpc_walking"]);

  Eigen::Vector3d init_dcm = robot.GetRobotComPos();
  Eigen::Vector3d init_dcm_vel = Eigen::Vector3d::Zero();
  Eigen::Quaterniond init_torso_quat(
      robot.GetLinkIsometry(draco_link::torso_com_link).linear());

  lmpc_handler.ForwardWalkMode();
  lmpc_handler.UpdateReferenceTrajectory(
      0., dcm_transfer_type::kInitial, init_torso_quat, init_dcm, init_dcm_vel);
  lmpc_handler.GetDCMPlanner()->SaveSolution("1");

  lmpc_handler.BackwardWalkMode();
  lmpc_handler.UpdateReferenceTrajectory(
      0., dcm_transfer_type::kInitial, init_torso_quat, init_dcm, init_dcm_vel);
  lmpc_handler.GetDCMPlanner()->SaveSolution("2");

  lmpc_handler.InplaceWalkMode();
  lmpc_handler.UpdateReferenceTrajectory(
      0., dcm_transfer_type::kInitial, init_torso_quat, init_dcm, init_dcm_vel);
  lmpc_handler.GetDCMPlanner()->SaveSolution("3");

  lmpc_handler.LeftTurnWalkMode();
  lmpc_handler.UpdateReferenceTrajectory(
      0., dcm_transfer_type::kInitial, init_torso_quat, init_dcm, init_dcm_vel);
  lmpc_handler.GetDCMPlanner()->SaveSolution("4");

  lmpc_handler.RightTurnWalkMode();
  lmpc_handler.UpdateReferenceTrajectory(
      0., dcm_transfer_type::kInitial, init_torso_quat, init_dcm, init_dcm_vel);
  lmpc_handler.GetDCMPlanner()->SaveSolution("5");

  lmpc_handler.LeftStrafeWalkMode();
  lmpc_handler.UpdateReferenceTrajectory(
      0., dcm_transfer_type::kInitial, init_torso_quat, init_dcm, init_dcm_vel);
  lmpc_handler.GetDCMPlanner()->SaveSolution("6");

  lmpc_handler.RightStrafeWalkMode();
  lmpc_handler.UpdateReferenceTrajectory(
      0., dcm_transfer_type::kInitial, init_torso_quat, init_dcm, init_dcm_vel);
  lmpc_handler.GetDCMPlanner()->SaveSolution("7");

  return 0;
}
