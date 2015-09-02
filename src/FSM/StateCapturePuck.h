#ifndef STATE_CP
#define STATE_CP

#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "State.h"

#include "StateEvade.h"
#include "StateTrackPuck.h"
#include "StateTrackGoal.h"
#include "StateSearchGoal.h"

using namespace std;

class StateCapturePuck: public State{

 public:

  StateCapturePuck();

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
