#include "controller/draco_controller/draco_state_machines/alip_locomotion.hpp"
#include "util/util.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/whole_body_controller/managers/alipmpc_trajectory_manager.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include <fstream>

//the state will be the following; 
//First visit: the thing would be do a single step since so i start moving
//One step(): will be the main loop. While not interrupt do (at the moment no interrupt,will walk indifinitely)
//Get command will call one step one time but will perform multiple steps. controller_->GetCommand will be performed inside state_machine
AlipLocomotion::AlipLocomotion(StateId state_id, PinocchioRobotSystem *robot,
                            DracoControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch){

    util::PrettyConstructor(2, "AlipLocomotion");

    sp_ = DracoStateProvider::GetStateProvider();


    first_ever = true;
    new_leg = false;

    file1.open(THIS_COM "/test/alip/LandTime.txt", std::fstream::out);


}


void AlipLocomotion::FirstVisit(){
  if(first_ever) {
    state_machine_start_time_ = sp_->current_time_;
    first_ever = false;
    if (stance_leg == 1) {
      ctrl_arch_->alip_tm_->SetSwingFootStart(robot_->GetLinkIsometry(draco_link::l_foot_contact).translation());
    } else {
      ctrl_arch_->alip_tm_->SetSwingFootStart(robot_->GetLinkIsometry(draco_link::r_foot_contact).translation());
    }
    ctrl_arch_->alip_tm_->initializeOri();
    
  }
  
  else if (new_leg) {
    new_leg = false;
    state_machine_start_time_ = sp_->current_time_;
  }
  
  
  ctrl_arch_->alip_tm_->setNewOri();

  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

  Tr = Ts - state_machine_time_;

  ctrl_arch_->alip_tm_->MpcSolutions(Tr, stance_leg);

  ctrl_arch_->alip_tm_->GenerateSwingFtraj(Tr);
  ctrl_arch_->alip_tm_->saveTrajectories(0, Ts/20, Ts);
  util::PrettyConstructor(3, "Trajectories saved");

  if (stance_leg == 1) {
    //sp_->b_lf_contact_ = false;
    //sp_->b_rf_contact_ = true;

    ctrl_arch_->tci_container_->contact_map_["rf_contact"]->SetMaxFz(500);
    ctrl_arch_->tci_container_->contact_map_["lf_contact"]->SetMaxFz(0);
  }
  else {
    ctrl_arch_->tci_container_->contact_map_["lf_contact"]->SetMaxFz(500);
    ctrl_arch_->tci_container_->contact_map_["rf_contact"]->SetMaxFz(0);
    //sp_->b_rf_contact_ = false;
    //sp_->b_lf_contact_ = true;

  }

}
void AlipLocomotion::OneStep(){
    state_machine_time_ = sp_->current_time_ -state_machine_start_time_;
    double t = state_machine_time_+ Tr - Ts;
    ctrl_arch_->alip_tm_->UpdateDesired(t);
}
void AlipLocomotion::LastVisit(){

}
bool AlipLocomotion::EndOfState(){

}


StateId AlipLocomotion::GetNextState() {
  return draco_states::kDoubleSupportBalance;
}





bool AlipLocomotion::SwitchLeg(){  //ahora asume que tocamos en Tr o antes. Que pasa si se atrasa?
  bool switch_leg = false;
  counter ++;
  if (counter == 50){
    cout << sp_->current_time_ - state_machine_start_time_ << endl;
    cout << sp_->b_lf_contact_ << "left contact " << endl;
    cout << sp_->b_rf_contact_ << "right contact " << endl;
    counter = 0;
  }


  //if (Tr < 0.5*Ts) {   //have to add a restriction to not have consecutives
  if (sp_->current_time_ - state_machine_start_time_ > 0.5*Ts){
    if ((stance_leg == 1) && (sp_->b_lf_contact_)){  //right stance, left swing
    //if((stance_leg == 1) && (robot_->GetLinkIsometry(draco_link::l_foot_contact).translation()(2) < 0.00005)){
      util::PrettyConstructor(2, "Switch Leg AlipLocomotion true ");

      std::cout << "Right stance to left" << " | Tr:" << Tr << "  | state machine time:" << state_machine_time_  <<std::endl;
      stance_leg *= -1;
      //ctrl_arch_->alip_tm_->RToLstance();
      //update the force managers
      switch_leg = true;

      state_machine_start_time_ = sp_->current_time_; //this should go to new leg

      ctrl_arch_->tci_container_->contact_map_["lf_contact"]->SetMaxFz(500);
      new_leg = true;


      //ctrl_arch_->tci_container_->task_map_["rf_pos"]->SetMaxFz(0.01);
      ctrl_arch_->alip_tm_->SetSwingFootStart(robot_->GetLinkIsometry(draco_link::r_foot_contact).translation());


    }   
    else if((stance_leg == -1) && (sp_->b_rf_contact_)){
    //else if((stance_leg == -1) && (robot_->GetLinkIsometry(draco_link::r_foot_contact).translation()(2) < 0.00005)){
      util::PrettyConstructor(2, "Switch Leg AlipLocomotion true ");

      std::cout << "Left stance to right" << " | Tr:" << Tr << "  | state machine time:" << state_machine_time_ <<std::endl;

      stance_leg *=-1;
      //ctrl_arch_->alip_tm_->LToRstance();
      //update the force managers
      switch_leg = true;

      state_machine_start_time_ = sp_->current_time_; //same thing than before

      ctrl_arch_->tci_container_->contact_map_["rf_contact"]->SetMaxFz(500);
      new_leg = true;

      ctrl_arch_->alip_tm_->SetSwingFootStart(robot_->GetLinkIsometry(draco_link::l_foot_contact).translation());

    }
  }
  
  if (switch_leg){ 
    file1 << sp_->current_time_ << endl; 

    //ctrl_arch_->alip_tm_->saveCurrentCOMstate();
    /*
    std::cout << robot_->GetLinkIsometry(draco_link::l_foot_contact).translation()(2) ;
    std::cout << "  l foot" << std::endl;
    std::cout << robot_->GetLinkIsometry(draco_link::r_foot_contact).translation()(2) << "  r foot" << std::endl;
    */
  }
  
  return switch_leg;

}




void AlipLocomotion::SetParameters(const YAML::Node &node) {
  try {
    util::ReadParameter(node, "swing_height", swing_height_);
    util::ReadParameter(node, "Ts", Ts);
    util::ReadParameter(node, "stance_leg", stance_leg);

  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }
}


int AlipLocomotion::GetStance_leg(){return stance_leg;}

