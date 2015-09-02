#ifndef STATE_SG
#define STATE_SG

#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "State.h"

#include "StateTrackGoal.h"
#include "StateEvade.h"

//#include "fsm.h"

class StateSearchGoal: public State{

 public:

  StateSearchGoal();

  void Enter();
  void Execute(StateManager * fsm);
  void Exit();
  
  State * Transition(bool* stimuli);

  std::string GetNameString();
  
  void Print();

 private:

  string name;

};
#endif
