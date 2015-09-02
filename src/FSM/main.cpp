#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "FSM.h"

using namespace std;

int main(){

  // seePuck, havePuck, seeGoal, atGoal, movingForward, puck2Close2Goal, prox
  bool stimuli[7] = {0,0,0,0,0,0,0};
  string stimuliArray[7] = {"seePuck", "havePuck", "seeGoal", "atGoal", 
			    "movingForward", "puck2Close2Goal", "prox"}; 
  
  StateManager* fsm = new StateManager();
  
  bool keepGoing = true;

  char response;

  while(keepGoing){

    printf("Current State = ");
    fsm->PrintCurrentState();
        
    for(int i = 0; i < 7; i++){
      printf("%s: ", stimuliArray[i].c_str());
      char c; 
      scanf("%s", &c);
      if(c == 'y')
	stimuli[i] = true;
      else
	stimuli[i] = false;
    }

    fsm->UpdateBehaviour(stimuli);
    
    fsm->ExecuteBehaviour();

    printf("Keep Going? ");
    scanf("%s", &response);
    if(response == 'y')
      keepGoing = true;
    else
      keepGoing = false;
  
    printf("=============================\n");
  }
  
  printf("Exiting...\n");
  return 0;
}
