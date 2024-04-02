#pragma once
#include <Eigen/Dense>
#include <vector>
#include <util/util.hpp>

class DracoStateProvider {
public:
  static DracoStateProvider *GetStateProvider();
  ~DracoStateProvider() = default;

  // servo dt should be set outside of controller
  double servo_dt_;
  int data_save_freq_;

  int count_;
  double current_time_;

  // should be set outside of controller
  Eigen::VectorXd nominal_jpos_;

  // used in pos estimate in estimator module
  int stance_foot_;
  int prev_stance_foot_;

  Eigen::Matrix3d rot_world_local_;

  Eigen::Vector3d dcm_;
  Eigen::Vector3d prev_dcm_;
  Eigen::Vector3d dcm_vel_;

  bool b_lf_contact_;
  bool b_rf_contact_;
  bool b_request_change_swing_leg_;
  int b_swing_leg_;

  Eigen::Vector3d com_vel_est_;

  int state_;
  int prev_state_;

  bool b_use_base_height_;
  bool b_use_kf_state_estimator_;

  double des_com_height_;
  Eigen::Quaterniond des_torso_quat_;

  int planning_id_;

  std::vector<int> floating_base_jidx_;

  Eigen::Vector3d cam_est_;




  double mass_;
  int initial_stance_leg_;
  double stance_leg_;
  double Lx_offset_des_;
  double Ly_des_;
  double des_com_yaw_;
  double Ts_;
  double Tr_;




  double mu_;
  double kx_;
  double ky_;

  void outsideCommand(const YAML::Node &node){
    util::ReadParameter(node, "Lx_offset", Lx_offset_des_);
    util::ReadParameter(node, "Ly_des", Ly_des_);
    util::ReadParameter(node, "com_yaw", des_com_yaw_);
    util::ReadParameter(node, "mu", mu_);
    util::ReadParameter(node, "kx", kx_);
    util::ReadParameter(node, "ky", ky_);
    des_com_yaw_ *= M_PI/180;
  }



private:
  DracoStateProvider();
};
