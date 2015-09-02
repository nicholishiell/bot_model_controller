#ifndef STATE_SP
#define STATE_SP

#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "State.h"

#include "StateTrackPuck.h"
#include "StateEvade.h"

//#include "fsm.h"

class StateSearchPuck: public State{

 public:

  StateSearchPuck();

  void Enter();
  void Execute(StateManager * fsm);
  void Exit();
  
  // seePuck, havePuck, seeGoal, atGoal, movingForward, puck2Close2Goal, prox
  State * Transition(bool* stimuli);

  void Print();
  
  std::string GetNameString();
 
  private:

  string name;

};
#endif
