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

#ifndef STATEMACHINE_H
#define STATEMACHINE_H

class State
{
  public:

    State(void (*updateFunction)());
    State(void (*enterFunction)(), void (*updateFunction)(), void (*exitFunction)());

    void enter();
    void loop();
    void exit();

  private:
    void (*userEnter)();
    void (*loopFunction)();
    void (*userExit)();
};


typedef struct Transition {
  State& from;
  State& to;

  // This function has to return true to make the state machine perform the transition during that loop
  bool (*transitionFunction)();
};

class StateMachine
{
  public:
    // GLOBAL state can be used to perform transitions to a state starting from any state
    static State* ANY;


    StateMachine(State& initialState, Transition* trans, int transitionsCount);
    void loop();
    State& getCurrentState() const;
    bool isInState(State &state) const;

    // Can be used to force a transition even if the transition function would not return true
    void performTransitionNow(Transition &transition);

  private:
    State* currentState;
    Transition* transitions;
    int transCount;
};

#endif
// STATEMACHINE_H
