#ifndef STATE_DP
#define STATE_DP

#include <stdio.h>
#include <stdlib.h>
#include <time.h> 

#include <string>

#include "State.h"
#include "StateSearchPuck.h"
#include "StateEvade.h"

using namespace std;

class StateDepositPuck: public State{

 public:

  StateDepositPuck();
  
  void Enter();
  void Execute(StateManager* fsm);
  void Exit();
  
  // seePuck, havePuck, seeGoal, atGoal, movingForward, puck2Close2Goal, prox
  State * Transition(bool* stimuli);
  
  std::string GetNameString();
  
  void Print();
  
 private:
  
  string name;
  
  float deltaT;
  time_t timeStamp;
  bool timerExpired;
};
#endif
