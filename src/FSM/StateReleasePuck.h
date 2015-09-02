#ifndef STATE_RP
#define STATE_RP

#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "State.h"

#include "StateDepositPuck.h"
#include "StateEvade.h"

//#include "fsm.h"

class StateReleasePuck: public State{

 public:

  StateReleasePuck();

  void Enter();
  void Execute(StateManager* fsm);
  void Exit();
  
  // seePuck, havePuck, seeGoal, atGoal, movingForward, puck2Close2Goal, prox
  State * Transition(bool* stimuli);

  std::string GetNameString();
  
  void Print();

 private:

  string name;

};
#endif
