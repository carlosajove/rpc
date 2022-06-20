#pragma once

#include "controller/state_machine.hpp"

class DracoControlArchitecture;
class DracoStateProvider;
class DoubleSupportStandUp : public StateMachine {
public:
  DoubleSupportStandUp(const StateId state_id, PinocchioRobotSystem *robot,
                       DracoControlArchitecture *ctrl_arch);
  virtual ~DoubleSupportStandUp() = default;

  void FirstVisit() override;
  void OneStep() override;
  void LastVisit() override;
  bool EndOfState() override;

  StateId GetNextState() override;

  void InitializeParameters(const YAML::Node &node) override;

private:
  DracoControlArchitecture *ctrl_arch_;
  DracoStateProvider *sp_;

  double duration_;
  double target_height_;
  bool b_use_base_height_;
};