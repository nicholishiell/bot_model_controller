#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "StateSearchGoal.h"
#include "FSM.h"

StateSearchGoal::StateSearchGoal(){
  name = "SearchGoal";

  /* initialize random seed: */
  srand (time(NULL));
}

void StateSearchGoal::Enter(){};
void StateSearchGoal::Execute(StateManager * fsm){
 
  if(rand() / (float)RAND_MAX < 0.1){
    fsm->SetRotSpeed( 10.*(rand() / (float)RAND_MAX - 0.5));
  }

};
void StateSearchGoal::Exit(){};

// seePuck, havePuck, seeGoal, atGoal, movingForward, puck2Close2Goal, prox
State * StateSearchGoal::Transition(bool* stimuli){
 
  for(int i = 0; i < 7; i++)
    printf("%i ", stimuli[i]);
  printf("\n");

  if(stimuli[6] == true){
    return new StateEvade();
  }
  else if(stimuli[2] == true and stimuli[1] == true){
    return new StateTrackGoal();
  }
  else{
    return NULL;
  } 
 
  
};

std::string StateSearchGoal::GetNameString(){
  return name;
};

void StateSearchGoal::Print(){
  printf("%s\n",name.c_str());
}


//g++ main.cpp StateSearchPuck.cpp StateTrackPuck.cpp StateCapturePuck.cpp StateSearchGoal.cpp StateTrackGoal.cpp StateReleasePuck.cpp StateDepositPuck.cpp StateEvade.cpp -o test 
