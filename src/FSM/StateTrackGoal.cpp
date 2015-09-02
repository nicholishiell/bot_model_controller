#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "StateTrackGoal.h"
#include "FSM.h"

StateTrackGoal::StateTrackGoal(){
  name = "TrackGoal"; 
};

void StateTrackGoal::Enter(){};

void StateTrackGoal::Execute(StateManager* fsm){

  printf("Executing behaviour trackGoal...\n");

  float trackingError = fsm->GetRotControlErrorGoal();
  float kp = fsm->GetProportionalGain();
  
  fsm->SetRotSpeed(kp*trackingError);
  fsm->SetTransSpeed(1);
    
};

void StateTrackGoal::Exit(){};

// seePuck, havePuck, seeGoal, atGoal, movingForward, puck2Close2Goal, prox
State * StateTrackGoal::Transition(bool* stimuli){
  
  for(int i = 0; i < 7; i++)
    printf("%i ", stimuli[i]);
  printf("\n");
   

  if(stimuli[6] == true){
    return new StateEvade();
  }
  else if(stimuli[2] == false){
    return new StateSearchGoal();
  }

  else if(stimuli[2] == true and ! stimuli[3] == true ){
    return new StateTrackGoal();
  }
  else if(stimuli[3] == true){
    return new StateReleasePuck();
  }

  else{

  }

};

std::string StateTrackGoal::GetNameString(){
  return name;
};

void StateTrackGoal::Print(){
  printf("%s\n",name.c_str());
};
