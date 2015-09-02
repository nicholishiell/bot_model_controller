#include <stdio.h>
#include <stdlib.h>
#include <time.h> 

#include <string>

#include "StateDepositPuck.h"
#include "FSM.h"

StateDepositPuck::StateDepositPuck(){
  name = "DepositPuck"; 
  deltaT = 5.;
  
  // Get the time when object was created;
  time(&timeStamp);
  timerExpired = false;
};

void StateDepositPuck::Enter(){};

void StateDepositPuck::Execute(StateManager* fsm){
  
  time_t currentTime;
  time(&currentTime);
  
  // Basically do a three point turn
  if(difftime(currentTime, timeStamp) < 2.5){
    fsm->SetRotSpeed(-2.);
    fsm->SetTransSpeed(-1);
  }
  else{
    fsm->SetRotSpeed(-2.);
    fsm->SetTransSpeed(1);
  }

  fsm->OpenServo();
};

void StateDepositPuck::Exit(){};

// seePuck, havePuck, seeGoal, atGoal, movingForward, puck2Close2Goal, prox
State * StateDepositPuck::Transition(bool* stimuli){
  
  time_t currentTime;
  time(&currentTime);
  if(difftime(currentTime, timeStamp) > deltaT){
    timerExpired = true;
  }
  
  if(stimuli[6] == true){
    return new StateEvade();
  }
  else if(!timerExpired){
    return NULL;
  }
  else{
    return new StateSearchPuck();
  }
  
};

std::string StateDepositPuck::GetNameString(){
  return name;
};

void StateDepositPuck::Print(){
  printf("%s\n",name.c_str());
};
