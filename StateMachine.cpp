/*

Copyright 2016 Marco Cosentino

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

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
