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
