#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "StateReleasePuck.h"
#include "FSM.h"

StateReleasePuck::StateReleasePuck(){
  name = "ReleasePuck"; 
}

void StateReleasePuck::Enter(){};
void StateReleasePuck::Execute(StateManager* fsm){
  printf("Executing behaviour ReleasePuck...\n");

  fsm->OpenServo();

};
void StateReleasePuck::Exit(){};

// seePuck, havePuck, seeGoal, atGoal, movingForward, puck2Close2Goal, prox
State * StateReleasePuck::Transition(bool* stimuli){
  
  if(stimuli[6] == true){
    return new StateEvade();
  }
  else{
    return new StateDepositPuck();
  }
  
};

std::string StateReleasePuck::GetNameString(){
  return name;
};

void StateReleasePuck::Print(){
  printf("%s\n",name.c_str());
}
