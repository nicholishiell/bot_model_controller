#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "StateEvade.h"
#include "FSM.h"

StateEvade::StateEvade(){
  name = "Evade"; 
  deltaT = 5.;

  time(&timeStamp);
  timerExpired = false;

  first = true;
}

void StateEvade::Enter(){};
void StateEvade::Execute(StateManager* fsm){

  if(first){
    first = false;
    if(fsm->MovingForward()){
      printf("Backward\n");
      fsm->SetTransSpeed(-1);
      fsm->SetRotSpeed(-1.);
    }
    else{
      printf("Forward\n");
      fsm->SetTransSpeed(1);
      fsm->SetRotSpeed(1.);
    }
  }

};
void StateEvade::Exit(){};

// seePuck, havePuck, seeGoal, atGoal, movingForward, puck2Close2Goal, prox
State * StateEvade::Transition(bool* stimuli){
  time_t currentTime;
  time(&currentTime);
  
  printf("dt = %f\n", difftime(currentTime, timeStamp));
  
  // Check if manvouver has finished yet
  if(difftime(currentTime, timeStamp) > deltaT){
    printf("\n\n!!!WHAT!!!\n\n");
    timerExpired = true;
  }
  
  if(stimuli[6]){
    return new StateEvade();
  }
  else if(!timerExpired){ 
    return NULL;
  }
  else if(!stimuli[0] && !stimuli[1]){
    return new StateSearchPuck();
  }
  else if(stimuli[0] && !stimuli[1]){
    return new StateTrackPuck();
  }
  else if(!stimuli[2] && stimuli[1]){
    return new StateSearchGoal();
  }
  else if(stimuli[0] && stimuli[1]){
    return new StateTrackGoal();
  }
  else{}
  
};

std::string StateEvade::GetNameString(){
  return name;
};

void StateEvade::Print(){
  printf("%s\n",name.c_str());
};
