#include "StateMachine.h"

State::State(void (*updateFunction)())
{
  userEnter = 0;
  loopFunction = updateFunction;
  userExit = 0;
}

State::State(void (*enterFunction)(), void (*updateFunction)(), void (*exitFunction)())
{
  userEnter = enterFunction;
  loopFunction = updateFunction;
  userExit = exitFunction;
}

void State::enter()
{
  if (userEnter)
  {
    userEnter();
  }
}

void State::loop()
{
  if (loopFunction)
  {
    loopFunction();
  }
}

void State::exit()
{
  if (userExit)
  {
    userExit();
  }
}

State* StateMachine::ANY = new State(0);

StateMachine::StateMachine(State& initialState, Transition* trans, int transitionsCount) {
  currentState = &initialState;
  transitions = trans;
  transCount = transitionsCount;
}

void StateMachine::loop() {
  int i;
  for(i=0; i<transCount; i++) {
    if ((&(transitions[i].from) == currentState || &(transitions[i].from) == StateMachine::ANY) && transitions[i].transitionFunction()) {
      this->performTransitionNow(transitions[i]);
      break;
    }
  }
  this->currentState->loop();
}

State& StateMachine::getCurrentState() const {
  return *currentState;
}

bool StateMachine::isInState(State &state) const {
  return (currentState == &state);
}

void StateMachine::performTransitionNow(Transition &transition) {
  transition.from.exit();
  transition.to.enter();
  currentState = &(transition.to);
}
