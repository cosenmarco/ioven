# iOven

This is my little Arduino project to make my cooking oven smart.

iOven stands for "intelligent oven".

## Interesting parts

I borrowed a couple of concepts from other languages / design patterns.
Here is what's most interesting to find in the code.

* State machine -- The Oven works like a state machine. The small library I wrote (StateMachine.h, StateMachine.ccp)
  allows to define states (made of an enter function, and exit function and a loop function) and state transitions
  (made of a "from" state, a "to" state and a transition function which returns true if a transition has to happen)
  Additionally it provides a meta-state ANY to perform transitions from any state to a specific state.
* Timeouts -- this makes things like blinking something or reacting to some stimulus after some time much more enjoyable.
  Like in Javascript you can pass a value in ms and a function that will be executed when the timer expires.
* The "world's most" accurate cheap encoder reading routine ever -- applying the priciples explained in [this youtube video](https://www.youtube.com/watch?v=f-Tjcx4b8_4) I came up with my own solution (also thanks to the suggestion of coupling previous and current state in a variable I found in [this page](https://hifiduino.wordpress.com/2010/10/20/rotaryencoder-hw-sw-no-debounce/)) and it seems working pretty well.
* MCP23017 expander
* MAX6675 thermocouple
* DS1307 Real time clock -- with adjustment of date / time
* LCD char display 4X20 -- interesting to see function display_printf_P to printf strings where the format string is stored in PROGMEM
  