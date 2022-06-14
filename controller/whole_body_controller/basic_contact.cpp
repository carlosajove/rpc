#include "controller/whole_body_controller/basic_contact.hpp"

// Point Contact
PointContact::PointContact(PinocchioRobotSystem *robot,
                           const int &target_link_idx, const double &mu)
    : Contact(robot, 3, target_link_idx, mu) {
  cone_constraint_matrix_ = Eigen::MatrixXd::Zero(6, dim_);
  cone_constraint_vector_ = Eigen::VectorXd::Zero(6);
}

void PointContact::UpdateContactJacobian() {
  jacobian_ = robot_->GetLinkJacobian(target_link_idx_)
                  .block(dim_, 0, dim_, robot_->GetNumQdot());
}

void PointContact::UpdateContactJacobianDotQdot() {
  jacobian_dot_q_dot_ =
      robot_->GetLinkJacobianDotQdot(target_link_idx_).tail(dim_);
}

void PointContact::UpdateConeConstraint() {
  cone_constraint_matrix_(0, 2) = 1.; // Fz >= 0
  cone_constraint_matrix_(1, 0) = 1.;
  cone_constraint_matrix_(1, 2) = mu_;
  cone_constraint_matrix_(2, 0) = -1.;
  cone_constraint_matrix_(2, 2) = mu_;
  cone_constraint_matrix_(3, 1) = 1.;
  cone_constraint_matrix_(3, 2) = mu_;
  cone_constraint_matrix_(4, 1) = -1.;
  cone_constraint_matrix_(4, 2) = mu_;
  cone_constraint_matrix_(5, 2) = -1.;

  Eigen::Matrix3d rot_world_local =
      robot_->GetLinkIsometry(target_link_idx_).linear();
  cone_constraint_matrix_ =
      cone_constraint_matrix_ * rot_world_local.transpose();

  cone_constraint_vector_[5] = -rf_z_max_;
}

// Surface Contact
SurfaceContact::SurfaceContact(PinocchioRobotSystem *robot,
                               const int &target_link_idx, const double &mu,
                               const double &x, const double &y)
    : Contact(robot, 6, target_link_idx, mu), x_(x), y_(y) {

  cone_constraint_matrix_ = Eigen::MatrixXd::Zero(16 + 2, dim_);
  cone_constraint_vector_ = Eigen::VectorXd::Zero(16 + 2);
}
void SurfaceContact::UpdateContactJacobian() {
  jacobian_ = robot_->GetLinkJacobian(target_link_idx_);
}

void SurfaceContact::UpdateContactJacobianDotQdot() {
  jacobian_dot_q_dot_ = robot_->GetLinkJacobianDotQdot(target_link_idx_);
}

void SurfaceContact::UpdateConeConstraint() {

  cone_constraint_matrix_(0, 5) = 1.; // Fz >= 0

  // Coulomb friction cone constraint
  cone_constraint_matrix_(1, 3) = 1.;
  cone_constraint_matrix_(1, 5) = mu_;
  cone_constraint_matrix_(2, 3) = -1.;
  cone_constraint_matrix_(2, 5) = mu_;

  cone_constraint_matrix_(3, 4) = 1.;
  cone_constraint_matrix_(3, 5) = mu_;
  cone_constraint_matrix_(4, 4) = -1.;
  cone_constraint_matrix_(4, 5) = mu_;

  // Cop condition
  cone_constraint_matrix_(5, 0) = 1.;
  cone_constraint_matrix_(5, 5) = y_;
  cone_constraint_matrix_(6, 0) = -1.;
  cone_constraint_matrix_(6, 5) = y_;

  cone_constraint_matrix_(7, 1) = 1.;
  cone_constraint_matrix_(7, 5) = x_;
  cone_constraint_matrix_(8, 1) = -1.;
  cone_constraint_matrix_(8, 5) = x_;

  // no-yaw-slippage condition
  cone_constraint_matrix_(9, 0) = -mu_;
  cone_constraint_matrix_(9, 1) = -mu_;
  cone_constraint_matrix_(9, 2) = 1.;
  cone_constraint_matrix_(9, 3) = y_;
  cone_constraint_matrix_(9, 4) = x_;
  cone_constraint_matrix_(9, 5) = (x_ + y_) * mu_;

  cone_constraint_matrix_(10, 0) = -mu_;
  cone_constraint_matrix_(10, 1) = mu_;
  cone_constraint_matrix_(10, 2) = 1.;
  cone_constraint_matrix_(10, 3) = y_;
  cone_constraint_matrix_(10, 4) = -x_;
  cone_constraint_matrix_(10, 5) = (x_ + y_) * mu_;

  cone_constraint_matrix_(11, 0) = mu_;
  cone_constraint_matrix_(11, 1) = -mu_;
  cone_constraint_matrix_(11, 2) = 1.;
  cone_constraint_matrix_(11, 3) = -y_;
  cone_constraint_matrix_(11, 4) = x_;
  cone_constraint_matrix_(11, 5) = (x_ + y_) * mu_;

  cone_constraint_matrix_(12, 0) = mu_;
  cone_constraint_matrix_(12, 1) = mu_;
  cone_constraint_matrix_(12, 2) = 1.;
  cone_constraint_matrix_(12, 3) = -y_;
  cone_constraint_matrix_(12, 4) = -x_;
  cone_constraint_matrix_(12, 5) = (x_ + y_) * mu_;
  /////////////////////////////////////////////////
  cone_constraint_matrix_(13, 0) = -mu_;
  cone_constraint_matrix_(13, 1) = -mu_;
  cone_constraint_matrix_(13, 2) = -1.;
  cone_constraint_matrix_(13, 3) = -y_;
  cone_constraint_matrix_(13, 4) = -x_;
  cone_constraint_matrix_(13, 5) = (x_ + y_) * mu_;

  cone_constraint_matrix_(14, 0) = -mu_;
  cone_constraint_matrix_(14, 1) = mu_;
  cone_constraint_matrix_(14, 2) = -1.;
  cone_constraint_matrix_(14, 3) = -y_;
  cone_constraint_matrix_(14, 4) = x_;
  cone_constraint_matrix_(14, 5) = (x_ + y_) * mu_;

  cone_constraint_matrix_(15, 0) = mu_;
  cone_constraint_matrix_(15, 1) = -mu_;
  cone_constraint_matrix_(15, 2) = -1.;
  cone_constraint_matrix_(15, 3) = y_;
  cone_constraint_matrix_(15, 4) = -x_;
  cone_constraint_matrix_(15, 5) = (x_ + y_) * mu_;

  cone_constraint_matrix_(16, 0) = mu_;
  cone_constraint_matrix_(16, 1) = mu_;
  cone_constraint_matrix_(16, 2) = -1.;
  cone_constraint_matrix_(16, 3) = y_;
  cone_constraint_matrix_(16, 4) = x_;
  cone_constraint_matrix_(16, 5) = (x_ + y_) * mu_;
  // ////////////////////////////////////////////////////
  cone_constraint_matrix_(17, 5) = -1.;

  Eigen::MatrixXd aug_R_world_local = Eigen::MatrixXd::Zero(dim_, dim_);
  aug_R_world_local.block(0, 0, 3, 3) =
      robot_->GetLinkIsometry(target_link_idx_).linear();
  aug_R_world_local.block(3, 3, 3, 3) =
      robot_->GetLinkIsometry(target_link_idx_).linear();

  cone_constraint_matrix_ =
      cone_constraint_matrix_ * aug_R_world_local.transpose();

  cone_constraint_vector_[17] = -rf_z_max_;
}
