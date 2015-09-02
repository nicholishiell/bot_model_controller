#ifndef STATE_HT
#define STATE_HT

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "State.h"

using namespace std;


class StateHalt: public State{

 public:

  StateHalt(){
    name = "Halt"; 
  }

  void Enter(){};
  void Execute(){};
  void Exit(){};
  
  State * Transition(bool* stimuli){
    
  };

  void Print(){
    printf("%s\n",name.c_str());
  }

 private:

  string name;

};
#endif
