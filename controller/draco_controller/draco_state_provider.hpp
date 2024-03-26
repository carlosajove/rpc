#pragma once
#include <Eigen/Dense>
#include <vector>
#include "util/util.hpp"

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

  //RL action
  Eigen::VectorXd res_rl_action_;

  //WBC obs
  double mass_;

  bool rl_trigger_ = false;
  double stance_leg_;
  double Lx_offset_;
  double Ly_des_;
  double des_com_yaw_;
  double Ts_;
  double Tr_;
  Eigen::Vector3d com_pos_stance_frame_;
  Eigen::Vector3d L_stance_frame_;
  Eigen::Vector3d stfoot_pos_;
  Eigen::Vector3d torso_roll_pitch_yaw_;

  Eigen::VectorXd get_wbc_obs(){
    Eigen::VectorXd obs(18);
    obs <<  stance_leg_,
            Lx_offset_,               //2
            Ly_des_,
            des_com_yaw_,             //4
            Ts_,
            Tr_,
            com_pos_stance_frame_(0), //7
            com_pos_stance_frame_(1),
            com_pos_stance_frame_(2),
            L_stance_frame_(0),       //10
            L_stance_frame_(1),
            L_stance_frame_(2),
            stfoot_pos_(0),           //13
            stfoot_pos_(1),
            stfoot_pos_(2),
            torso_roll_pitch_yaw_(0), //16
            torso_roll_pitch_yaw_(1),
            torso_roll_pitch_yaw_(2); //18
    return obs;
  }

  double mu_;
  double kx_;
  double ky_;

  void outsideCommand(const YAML::Node &node){
    util::ReadParameter(node, "Lx_offset", Lx_offset_);
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
