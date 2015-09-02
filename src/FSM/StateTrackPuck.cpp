#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "StateTrackPuck.h"
#include "FSM.h"

StateTrackPuck::StateTrackPuck(){
    name = "TrackPuck"; 
};

void StateTrackPuck::Enter(){};

void StateTrackPuck::Execute(StateManager * fsm){
  printf("Executing behaviour %s...\n", name.c_str());

  float trackingError = fsm->GetRotControlErrorPuck();
  float kp = fsm->GetProportionalGain();
  
  fsm->SetRotSpeed(kp*trackingError);
  fsm->SetTransSpeed(1);
  fsm->OpenServo();
};

void StateTrackPuck::Exit(){};
  
// seePuck, havePuck, seeGoal, atGoal, movingForward, puck2Close2Goal, prox
State * StateTrackPuck::Transition(bool* stimuli){
 
  for(int i = 0; i < 7; i++)
    printf("%i ", stimuli[i]);
  printf("\n");
    
  // Evade if prox true
  if(stimuli[6] == true){
    return new StateEvade();
  }
  // Don't see and don't have the puck SearchPuck
  else if(stimuli[0] == false and stimuli[1] == false){
    return new StateSearchPuck();
  }
  // See the puck but don't have one TrackPuck
  else if(stimuli[0] == true and stimuli[1] == false){
    return NULL;
  }
  // See the puck but don't have one TrackPuck
  else if(stimuli[0] == true and stimuli[1] == true){
    return new StateCapturePuck;
  }  
  else{
    return new StateSearchPuck();
  }
  
};

std::string StateTrackPuck::GetNameString(){
  return name;
};

void StateTrackPuck::Print(){
  printf("%s\n",name.c_str());
};
