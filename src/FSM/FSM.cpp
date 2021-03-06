#include <stdio.h>
#include <stdlib.h>

#include "FSM.h"

StateManager::StateManager(){
  currentState = new StateSearchPuck();
  
  trans_speed = 5.;
  rot_speed = 0.;
  openServo = true;
  
  kp = 4.0;

  rotControlErrorPuck = 0.;
  rotControlErrorGoal = 0.;    
};

void StateManager::UpdateBehaviour(bool* stimuli){
  State * newState;
  
  newState = currentState->Transition(stimuli);
  
  // Transition returns NULL if no transition occurs
  // (i.e when state remains the same);
  if(newState != NULL){
    delete currentState;
    currentState = newState;
  }
  
};

void StateManager::ExecuteBehaviour(float& trans, float& rot, bool& servoOpen){
  
  currentState->Execute(this);
  
  trans = trans_speed;
  rot = rot_speed;
  servoOpen = openServo;
};

void StateManager::PrintCurrentState(){
  currentState->Print();
};

string StateManager::GetCurrentStateName(){
  return currentState->GetNameString();
}

void StateManager::SetPuckControlError(float e){
  rotControlErrorPuck = e;
};

void StateManager::SetGoalControlError(float e){
  rotControlErrorGoal = e;
};

float StateManager::GetRotControlErrorPuck(){
  return rotControlErrorPuck;
};

float StateManager::GetRotControlErrorGoal(){
  return rotControlErrorGoal;
};

float StateManager::GetProportionalGain(){
  return kp;
};

void StateManager::SetRotSpeed(float speed){
  rot_speed = speed;
};

void StateManager::SetTransSpeed(int speed){
  if(speed == 1){
    trans_speed = 5.;
  }
  else if(speed == -1){
    trans_speed = -5.;
  }
  else{
    trans_speed = 0.;
  }
};
bool StateManager::MovingForward(){
  if(trans_speed > 0.)
    return true;
  else
    return false;
};

void StateManager::CloseServo(){
  openServo = false;
};

void StateManager::OpenServo(){
  openServo = true;
};
