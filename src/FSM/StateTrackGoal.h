#ifndef STATE_TG
#define STATE_TG

#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "State.h"

#include "StateTrackGoal.h"
#include "StateSearchGoal.h"
#include "StateReleasePuck.h"

class StateTrackGoal: public State{

 public:

  StateTrackGoal();

  void Enter();
  void Execute(StateManager* fsm);
  void Exit();
  
  State * Transition(bool* stimuli);
  
  std::string GetNameString();
  
  void Print();

 private:

  string name;

};
#endif
