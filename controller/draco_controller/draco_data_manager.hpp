#pragma once

#include "draco.pb.h" //in build/messages/
#include <Eigen/Dense>
#include <memory>
#include <zmq.hpp>

struct DracoData {
public:
  DracoData(){};
  ~DracoData() = default;

  double time_ = 0;
  int phase_ = 1;

  // Eigen::Vector3d base_joint_pos_;
  // Eigen::Vector4d base_joint_ori_;
  // Eigen::Vector3d base_joint_lin_vel_;
  // Eigen::Vector3d base_joint_ang_vel_;

  Eigen::Vector3d est_base_joint_pos_ = Eigen::Vector3d::Zero();
  Eigen::Vector4d est_base_joint_ori_ = Eigen::Vector4d::Zero();

  Eigen::VectorXd joint_positions_ = Eigen::VectorXd::Zero(27);

  Eigen::Vector3d des_com_pos_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d act_com_pos_ = Eigen::Vector3d::Zero();

  Eigen::VectorXd lfoot_pos_ = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd rfoot_pos_ = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd lfoot_ori_ = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd rfoot_ori_ = Eigen::VectorXd::Zero(4);

  Eigen::VectorXd lfoot_rf_cmd_ = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd rfoot_rf_cmd_ = Eigen::VectorXd::Zero(6);

  Eigen::Vector2d est_icp = Eigen::Vector2d::Zero();
  Eigen::Vector2d des_icp = Eigen::Vector2d::Zero();

  Eigen::Vector2d des_cmp = Eigen::Vector2d::Zero();
};

// Singleton class
class DracoDataManager {
public:
  static DracoDataManager *GetDataManager();
  ~DracoDataManager() = default;

  void InitializeSocket(const std::string &ip_address);
  void SendData();

  bool IsInitialized(); // socket initialized boolean getter

  std::unique_ptr<DracoData> data_;

private:
  DracoDataManager();

  std::unique_ptr<zmq::context_t> context_;
  std::unique_ptr<zmq::socket_t> socket_;

  bool b_initialize_socket_;
};
