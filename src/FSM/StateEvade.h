#ifndef STATE_EV
#define STATE_EV

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "State.h"

#include "StateSearchPuck.h"
#include "StateTrackPuck.h"
#include "StateCapturePuck.h"
#include "StateSearchGoal.h"
#include "StateTrackGoal.h"
#include "StateReleasePuck.h"
#include "StateDepositPuck.h"

//#include "fsm.h"

using namespace std;

class StateEvade: public State{

 public:

  StateEvade();

  void Enter();
  void Execute(StateManager* fsm);
  void Exit();
  
  State * Transition(bool* stimuli);

  std::string GetNameString();
  
  void Print();

 private:

  string name;
   
  float deltaT;
  time_t timeStamp;
  bool timerExpired;

  bool first;
};
#endif
