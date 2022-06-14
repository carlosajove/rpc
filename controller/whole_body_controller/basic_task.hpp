#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>

#include "controller/whole_body_controller/task.hpp"

class JointTask : public Task {
public:
  JointTask(PinocchioRobotSystem *robot);
  virtual ~JointTask() = default;

  void UpdateOscCommand();

  void UpdateTaskJacobian();
  void UpdateTaskJacobianDotQdot();
};

class SelectedJointTask : public Task {
public:
  SelectedJointTask(PinocchioRobotSystem *robot,
                    const std::vector<int> &joint_container);
  virtual ~SelectedJointTask() = default;

  void UpdateOscCommand();

  void UpdateTaskJacobian();
  void UpdateTaskJacobianDotQdot();

private:
  std::vector<int> joint_idx_container_;
};

class LinkPosTask : public Task {
public:
  LinkPosTask(PinocchioRobotSystem *robot, const int &target_link);
  virtual ~LinkPosTask() = default;

  void UpdateOscCommand();

  void UpdateTaskJacobian();
  void UpdateTaskJacobianDotQdot();

private:
  int target_link_idx_;
};

class LinkOriTask : public Task {
public:
  LinkOriTask(PinocchioRobotSystem *robot, const int &target_link);
  virtual ~LinkOriTask() = default;

  void UpdateOscCommand();

  void UpdateTaskJacobian();
  void UpdateTaskJacobianDotQdot();

private:
  int target_link_idx_;
};

class ComTask : public Task {
public:
  ComTask(PinocchioRobotSystem *robot);
  virtual ~ComTask() = default;

  void UpdateOscCommand();

  void UpdateTaskJacobian();
  void UpdateTaskJacobianDotQdot();
};
