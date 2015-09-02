#ifndef FSM_H
#define FSM_H

#include <stdio.h>
#include <stdlib.h>
#include <string>

// Abstract base class for state
#include "State.h"

// State classes
#include "StateSearchPuck.h"

using namespace std;

class StateManager{

public:

  StateManager();

  void UpdateBehaviour(bool* stimuli);

  void ExecuteBehaviour(float& trans, float& rot, bool& servoOpen);
 
  void PrintCurrentState();
  string GetCurrentStateName();

  void SetPuckControlError(float e);
  void SetGoalControlError(float e);

  float GetRotControlErrorPuck();
  float GetRotControlErrorGoal();

  float GetProportionalGain();

  void SetRotSpeed(float speed);
  void SetTransSpeed(int speed);
  
  void CloseServo();
  void OpenServo();

  bool MovingForward();

private:
  
  State * currentState;

  float trans_speed;
  float rot_speed;
  bool openServo;

  float rotControlErrorPuck;
  float rotControlErrorGoal;

  float kp;
};
#endif
