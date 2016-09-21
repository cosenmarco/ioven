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

// Date and time functions using a DS1307 RTC connected via I2C and Wire lib
#include <Wire.h>
#include "RTClib.h"
#include "max6675.h"
#include <LiquidCrystal.h>
#include <math.h>
#include "StateMachine.h"
#include "SimpleTimer.h"
#include <avr/pgmspace.h>

// MCP23017
#define MCP23017_ADDRESS 0x20
#define IODIRA 0x00
#define GPIOA 0x12
#define GPIOB 0x13

// Outputs: on GPA
#define DISPLAY_MASK 0x01 // The display bit is GPA0
#define BUZZER_MASK 0x02 // The buzzer bit is GPA1
#define LIGHT_MASK 0x04 // The Oven light is on GPA2
#define HEAT_ELEMENT_TOP 0x08
#define HEAT_ELEMENT_CENTER 0x10
#define HEAT_ELEMENT_BOTTOM 0x20
#define HEAT_ELEMENT_GRILL 0x40
#define VENTILATOR 0x80
// Useful to turn everything off
#define ALL_HEATING_ELEMENTS (HEAT_ELEMENT_TOP | HEAT_ELEMENT_CENTER | HEAT_ELEMENT_BOTTOM | HEAT_ELEMENT_GRILL)

// Inputs: on GPA
#define SWITCH1_MASK 0x20 // Switch 1 is on GPIOB 5
#define SWITCH2_MASK 0x40 // Switch 2 is on GPIOB 6
#define SWITCH3_MASK 0x80 // Switch 3 is on GPIOB 7 (active low)

#define ENCODER_CLK_PIN 3
#define ENCODER_DT_PIN 4

// Knob positions
#define OFF_POSITION 0
#define LIGHT_POSITION 1
#define CHICKEN_POSITION 2
#define THREE_ELEMENTS_POSITION 3
#define PIZZA_POSITION 4
#define GRILL_POSITION 5
#define GRILL_VENT_POSITION 6
#define CAKE_POSITION 7

// Output Bit Config by Program
#define CHICKEN_PROGRAM (HEAT_ELEMENT_TOP | HEAT_ELEMENT_BOTTOM)
#define THREE_ELEMENTS_PROGRAM (HEAT_ELEMENT_TOP | HEAT_ELEMENT_CENTER | HEAT_ELEMENT_BOTTOM)
#define PIZZA_PROGRAM (HEAT_ELEMENT_BOTTOM)
#define GRILL_PROGRAM (HEAT_ELEMENT_GRILL)
#define GRILL_VENT_PROGRAM (HEAT_ELEMENT_GRILL)
#define CAKE_PROGRAM (HEAT_ELEMENT_CENTER)

// ################# Strings stored in program memory #########################
// The labels must be 10 chars wide (excluded terminator)
const char empty_button_label[] PROGMEM = "          ";
const char timer_button_label[] PROGMEM = "   timer  ";
const char start_button_label[] PROGMEM = "   start  ";
const char cancel_button_label[] PROGMEM = " annulla  ";
const char continue_button_label[] PROGMEM = " continua ";
const char shut_down_button_label[] PROGMEM = "  spegni  ";

// Status string. These strings must be 8 chars wide (excluded terminator)
const char off[] PROGMEM = " SPENTO ";
const char light[] PROGMEM = "  LUCE  ";
const char chicken[] PROGMEM = "  POLLO ";
const char three[] PROGMEM = " 3 ELEM ";
const char pizza[] PROGMEM = "  PIZZA ";
const char grill[] PROGMEM = "  GRILL ";
const char grillvent[] PROGMEM = " GRILL V";
const char cake[] PROGMEM = "  TORTA ";


const char date_row_fmt[] PROGMEM = "%s %02d/%02d/%02d %02d:%02d:%02d";
const char temperature_row_fmt[] PROGMEM = "%3ld\1C->%3ld\1C%s";
const char timer_row_fmt[] PROGMEM = "%2d:%02d:%02d -> %2d:%02d:%02d";

const char time_over_message[] PROGMEM = "   TEMPO  SCADUTO   ";
const char oven_message[] PROGMEM = "       FORNO        ";
const char banner_row_fmt[] PROGMEM = {' ', 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, ' ', 'i', 'O', 'v', 'e', 'n', ' ', 0x7E,
  0x7E, 0x7E, 0x7E, 0x7E, ' ', ' ', 0x0};

LiquidCrystal lcd(10, 9, 8, 7, 6, 5);
RTC_DS1307 rtc;
MAX6675 thermocouple(13, 12, 11);
DateTime now, end;

byte mcp23017_GPIOA, mcp23017_GPIOB;

double currentTemperature;
double temperatureGoal;
bool currentHeaterStatus;
bool previousLoopHeaterStatus;

const char * currentProgramString = off;

byte currentPosition;
byte previousLoopPosition;
byte currentAcceptedPosition;
byte previousLoopAcceptedPosition;

byte currentOutputConfiguration;
byte previousLoopOutputConfiguration;

char daysOfTheWeek[7][3] = {"Do", "Lu", "Ma", "Me", "Gi", "Ve", "Sa"};
byte deg[8] = {
  B01110,
  B01010,
  B01110,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
};

SimpleTimer timer;
int timerSetTimeout = -1; // This timer is used to cancel automatically if we stay too long in timerSetState
int timerDecrementerInterval = -1; // This interval decrements the timer every second
int alarmInterval = -1; // This is used to toggle between beep on and beep off state to make an alarm
int alarmTimeoutTimerId = -1;
int blinkerInterval = -1;
int positionChangeTimer = -1; // Change in position knob is considered only after 2 seconds of stable knob position

unsigned int timerSeconds;
bool isMinutes;
bool blinkStatus;

// Rotary encoder data and handler
volatile int encoderPositionSinceLastLoop = 0;
void doEncoderClkTick() {
  bool clk = digitalRead(ENCODER_CLK_PIN);
  bool up;
  if (clk) {
    up = digitalRead(ENCODER_DT_PIN);
  } else {
    up = !digitalRead(ENCODER_DT_PIN);
  }
  if (up) {
    encoderPositionSinceLastLoop --;
  } else {
    encoderPositionSinceLastLoop ++;
  }
}

bool previousLoopButtonStatus[3] = { false, false, false };

// ################# The State Machine #################
State offState(&offStateEnter, NULL, &offStateExit);
State ovenState(&ovenStateEnter, &ovenLoop, NULL);
State timerSetState(&timerSetEnter, &timerSetLoop, &timerSetExit);
State timerRunState(&timerRunEnter, &timerRunLoop, &timerRunExit); // Note: same loop function of timerSetState
State alarmState(&alarmEnter, NULL, &alarmExit);

Transition fromAnyToOff = {
  *StateMachine::ANY,
  offState,
  &from_any_to_off
};

Transition fromOffToOven = {
  offState,
  ovenState,
  &from_off_to_oven
};

Transition fromOvenToTimerSet = {
  ovenState,
  timerSetState,
  &switch1Pressed
};

Transition fromOffToTimerSet = {
  offState,
  timerSetState,
  &switch1Pressed
};

Transition fromTimerSetToOff = {
  timerSetState,
  offState,
  &from_timer_to_off
};

Transition fromTimerSetToOven = {
  timerSetState,
  ovenState,
  &from_timer_to_oven
};

Transition fromTimerSetToTimerRun = {
  timerSetState,
  timerRunState,
  &switch1Pressed
};

Transition fromTimerRunToTimerSet = {
  timerRunState,
  timerSetState,
  &switch1Pressed
};

Transition fromTimerRunToOven = {
  timerRunState,
  ovenState,
  &from_timer_to_oven
};

Transition fromTimerRunToOff = {
  timerRunState,
  offState,
  &from_timer_to_off
};

Transition fromTimerRunToAlarm = {
  timerRunState,
  alarmState,
  &from_timer_run_to_alarm
};

Transition fromAlarmToOff = {
  alarmState,
  offState,
  &switch1Pressed
};

Transition fromAlarmToOven = {
  alarmState,
  ovenState,
  &from_timer_to_oven
};

#define TRANSITIONS_NUM 13
Transition transitions[TRANSITIONS_NUM] = {
  fromAnyToOff,
  fromOffToOven,
  fromOvenToTimerSet,
  fromOffToTimerSet,
  fromTimerSetToOff,
  fromTimerSetToOven,
  fromTimerSetToTimerRun,
  fromTimerRunToTimerSet,
  fromTimerRunToOven,
  fromTimerRunToOff,
  fromTimerRunToAlarm,
  fromAlarmToOff,
  fromAlarmToOven
};

StateMachine iOvenStateMachine(offState, transitions, TRANSITIONS_NUM);

// Setup function. Executed once at startup
void setup () {
  Serial.begin(9600);

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // NOTE: Initially wanted to use square wave generator attached to interrupt pin 2 to have some sort of
  // high resolution reliable clock.
  // NOW Using library SimpleTimer. SQW is still used for the beeper.
  // attachInterrupt(digitalPinToInterrupt(2), tick, FALLING);
  rtc.writeSqwPinMode(SquareWave4kHz);

  // Setup LCD
  lcd.createChar(1, deg);
  lcd.begin(20, 4);

  // Initialize MCP23017
  Wire.begin();

  Wire.beginTransmission(MCP23017_ADDRESS);
  Wire.write(IODIRA);
  Wire.write(0x00); // set all of port A to outputs
  Wire.write(0xFF); // Next register is IODIRB. Set all of port B to inputs
  Wire.write(0x00); // Next register is IPOLA. Set all of ports to normal
  Wire.write(0x80); // Next register is IPOLB. Set GPB7 as active low
  Wire.endTransmission();

  // Setup rotary encoder
  pinMode(ENCODER_CLK_PIN, INPUT);
  pinMode(ENCODER_DT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK_PIN), doEncoderClkTick, FALLING);


  // wait for thermocouple chip to stabilize
  delay(500);

  readInputs();
  offStateEnter(); // We start from the state OFF
  resetPreviousVariables();
}


// ############## LOOP and related functions ################
void loop () {
  readInputs();

  timer.run(); // This has to happen first because the result of some timouts needs to be caught by logic checks
  iOvenStateMachine.loop();

  updateDisplayClockLine();
  updateDisplayStatusLine();

  knobPositionLoop();
  temperatureControlLoop(); // This must be the last as devisions are taken earlier about currentOutputConfiguration

  resetPreviousVariables();
}

void resetPreviousVariables() {
  previousLoopOutputConfiguration = currentOutputConfiguration;
  previousLoopHeaterStatus = currentHeaterStatus;
  previousLoopPosition = currentPosition;
  previousLoopAcceptedPosition = currentAcceptedPosition;
  encoderPositionSinceLastLoop = 0;
  previousLoopButtonStatus[0] = (mcp23017_GPIOB & SWITCH1_MASK) != 0;
  previousLoopButtonStatus[1] = (mcp23017_GPIOB & SWITCH2_MASK) != 0;
  previousLoopButtonStatus[2] = (mcp23017_GPIOB & SWITCH3_MASK) != 0;
}

void readInputs() {
  now = rtc.now();
  currentTemperature = thermocouple.readCelsius();

  // Read the status of switches
  Wire.beginTransmission(MCP23017_ADDRESS);
  Wire.write(GPIOB);
  Wire.requestFrom(MCP23017_ADDRESS,1);
  mcp23017_GPIOB = Wire.read();
  Wire.endTransmission();

  currentPosition = mcp23017_GPIOB & 0x07; // The first 3 bits of GPIOB
}

void updateDisplayClockLine() {
  display_printf_P(0, date_row_fmt, daysOfTheWeek[now.dayOfTheWeek()], now.day(),
    now.month(), now.year() % 100, now.hour(), now.minute(), now.second());
}

void updateDisplayStatusLine() {
  char buffer[11];
  strcpy_P(buffer, currentProgramString);
  display_printf_P(1, temperature_row_fmt, lround(currentTemperature), lround(temperatureGoal), buffer);
}

void temperatureControlLoop() {
  // Temperature control logic: turn on heater unless temp equal or greater than the goal
  currentHeaterStatus = currentTemperature < temperatureGoal;
  if(currentHeaterStatus != previousLoopHeaterStatus || currentOutputConfiguration != previousLoopOutputConfiguration) {
    // Either heater status or output config changed. We have to update the output config on GPA
    mcp23017_GPIOA &= ~ALL_HEATING_ELEMENTS;
    if(currentHeaterStatus && currentOutputConfiguration > 0) {
      mcp23017_GPIOA |= currentOutputConfiguration;
      debug(F("HEATING ON"));
    } else {
      debug(F("HEATING OFF"));
    }
    sendGPIOA();
  }
}

void knobPositionLoop() {
  if(currentPosition != previousLoopPosition) {
    debug(F("Position changed"));
    if(positionChangeTimer >= 0) {
      timer.restartTimer(positionChangeTimer);
    } else {
      positionChangeTimer = timer.setTimeout(2000, &acceptPosition);
    }
  }
}

// Callback fired when there is a change in the positions knob
void acceptPosition() {
  currentAcceptedPosition = currentPosition;
  positionChangeTimer = -1;
  debug(F("Accepted New Position"));
  updateOutputsBasedOnCurrentProgram();
}

void updateOutputsBasedOnCurrentProgram() {
  switch(currentAcceptedPosition) {
    case OFF_POSITION:
    default:
      turnGPIOAFlagOff(LIGHT_MASK | VENTILATOR);
      currentProgramString = off;
      currentOutputConfiguration = 0;
      break;
    case LIGHT_POSITION:
      turnGPIOAFlagOff(VENTILATOR);
      turnGPIOAFlagOn(LIGHT_MASK);
      currentProgramString = light;
      currentOutputConfiguration = 0;
      break;
    case CHICKEN_POSITION:
      turnGPIOAFlagOn(LIGHT_MASK | VENTILATOR);
      currentProgramString = chicken;
      currentOutputConfiguration = CHICKEN_PROGRAM;
      break;
    case THREE_ELEMENTS_POSITION:
      turnGPIOAFlagOn(LIGHT_MASK | VENTILATOR);
      currentProgramString = three;
      currentOutputConfiguration = THREE_ELEMENTS_PROGRAM;
      break;
    case PIZZA_POSITION:
      turnGPIOAFlagOn(LIGHT_MASK | VENTILATOR);
      currentProgramString = pizza;
      currentOutputConfiguration = PIZZA_PROGRAM;
      break;
    case GRILL_POSITION:
      turnGPIOAFlagOff(VENTILATOR);
      turnGPIOAFlagOn(LIGHT_MASK);
      currentProgramString = grill;
      currentOutputConfiguration = GRILL_PROGRAM;
      break;
    case GRILL_VENT_POSITION:
      turnGPIOAFlagOn(LIGHT_MASK | VENTILATOR);
      currentProgramString = grillvent;
      currentOutputConfiguration = GRILL_VENT_PROGRAM;
      break;
    case CAKE_POSITION:
      turnGPIOAFlagOn(LIGHT_MASK | VENTILATOR);
      currentProgramString = cake;
      currentOutputConfiguration = CAKE_PROGRAM;
      break;
  }
}

// ###############  State Machine functions ###############
void offStateEnter() {
  debug(F("offStateEnter"));
  longBeep();
  turnGPIOAFlagOff(DISPLAY_MASK | LIGHT_MASK | VENTILATOR);
  display_print_P(2, 0, banner_row_fmt);
  setButtonLabel_P(0, timer_button_label);
  setButtonLabel_P(1, empty_button_label);

  currentProgramString = off;
  currentOutputConfiguration = 0;

  // Some init that are useful for the next time the oven is turned on
  timerSeconds = 60;
  temperatureGoal = 60.0;
}

void offStateExit() {
  debug(F("offStateExit"));
  turnGPIOAFlagOn(DISPLAY_MASK);
  beep();
}

void ovenStateEnter() {
  debug(F("ovenEnter"));
  display_print_P(2, 0, oven_message);
  setButtonLabel_P(0, timer_button_label);
  setButtonLabel_P(1, empty_button_label);
}

void timerSetEnter() {
  debug(F("timerSetEnter"));
  timerSetTimeout = timer.setTimeout(20000, &invokeCancelTimerSet);
  blinkerInterval = timer.setInterval(500, &blinkCallback);
  isMinutes = true;
  blinkStatus = true;
  setButtonLabel_P(0, start_button_label);
  setButtonLabel_P(1, cancel_button_label);
}

void timerSetExit() {
  debug(F("timerSetExit"));
  if(timerSetTimeout >= 0) {
    timer.deleteTimer(timerSetTimeout);
    timerSetTimeout = -1;
  }
  timer.deleteTimer(blinkerInterval);
  blinkerInterval = -1;
  blinkStatus = true;
}

void invokeCancelTimerSet() {
  iOvenStateMachine.performTransitionNow(fromTimerSetToOff);
  timerSetTimeout = -1;
}

void timerSetLoop() {
  char buffer[21];

  if(encoderPositionSinceLastLoop != 0) {
    int sum;
    if(isMinutes) {
      sum = encoderPositionSinceLastLoop * 60 + timerSeconds; // Every tick is 1 minute
    } else {
      sum = encoderPositionSinceLastLoop      + timerSeconds; // Every tick is 5 seconds
    }

    if(sum >= 0) {
      timerSeconds = sum;
    } else {
      timerSeconds = 0;
    }

    if(timerSetTimeout >= 0) {
      timer.restartTimer(timerSetTimeout);
    }

  }

  isMinutes ^= switch3Pressed();

  end = now + TimeSpan(timerSeconds);
  sprintf_P(buffer, timer_row_fmt, timerSeconds / 3600, (timerSeconds / 60) % 60,
    timerSeconds % 60, end.hour(), end.minute(), end.second());

  if(!blinkStatus) {
    if(isMinutes) {
      buffer[3] = ' ';
      buffer[4] = ' ';
    } else {
      buffer[6] = ' ';
      buffer[7] = ' ';
    }
  }

  lcd.setCursor(0, 2);
  lcd.print(buffer);
}

void ovenLoop() {
  temperatureGoal += encoderPositionSinceLastLoop;
  if(temperatureGoal < 0) {
    temperatureGoal = 0;
  }
}

void decrementTimer() {
  if (timerSeconds > 0) {
    timerSeconds--;
  }
}

void timerRunEnter() {
  debug(F("timerRunEnter"));
  timerDecrementerInterval = timer.setInterval(1000, &decrementTimer);
  setButtonLabel_P(0, timer_button_label);
}

void timerRunExit() {
  debug(F("timerRunExit"));
  timer.deleteTimer(timerDecrementerInterval);
  timerDecrementerInterval = -1;
}

void timerRunLoop() {
  ovenLoop();
  display_printf_P(2, timer_row_fmt, timerSeconds / 3600, (timerSeconds / 60) % 60,
    timerSeconds % 60, end.hour(), end.minute(), end.second());
}

bool from_timer_run_to_alarm() {
  return timerSeconds <= 0;
}

void toggleAlarm() {
  mcp23017_GPIOA ^= BUZZER_MASK;
  sendGPIOA();
}

void alarmEnter() {
  debug(F("alarmEnter"));
  alarmInterval = timer.setInterval(300, &toggleAlarm);
  alarmTimeoutTimerId = timer.setTimeout(15000, &alarmTimeout);
  setButtonLabel_P(0, shut_down_button_label);
  if(currentAcceptedPosition == OFF_POSITION) {
    setButtonLabel_P(1, empty_button_label);
  } else {
    setButtonLabel_P(1, continue_button_label);
  }
  display_print_P(2, 0, time_over_message);
  currentOutputConfiguration = 0;
}

void alarmExit() {
  debug(F("alarmExit"));
  timer.deleteTimer(alarmInterval);
  alarmInterval = -1;
  if(alarmTimeoutTimerId >= 0) {
    timer.deleteTimer(alarmTimeoutTimerId);
    alarmTimeoutTimerId = -1;
  }
  turnGPIOAFlagOff(BUZZER_MASK);
  updateOutputsBasedOnCurrentProgram();
}

void alarmTimeout() {
  iOvenStateMachine.performTransitionNow(fromAlarmToOff);
  alarmTimeoutTimerId = -1;
}

bool from_any_to_off() {
  return  positionChanged() && currentAcceptedPosition == OFF_POSITION;
}

bool from_off_to_oven() {
  return positionChanged() && currentAcceptedPosition != OFF_POSITION;
}

bool from_timer_to_off() {
  return switch2Pressed() && currentAcceptedPosition == OFF_POSITION;
}

bool from_timer_to_oven() {
  return switch2Pressed() && currentAcceptedPosition != OFF_POSITION;
}

// ############### Helper functions ###############
void turnGPIOAFlagOff(byte mask) {
  mcp23017_GPIOA &= ~mask;
  sendGPIOA();
}

void turnGPIOAFlagOn(byte mask) {
  mcp23017_GPIOA |= mask;
  sendGPIOA();
}

bool switch1Pressed() {
  return (mcp23017_GPIOB & SWITCH1_MASK) && !previousLoopButtonStatus[0];
}

bool switch2Pressed() {
  return (mcp23017_GPIOB & SWITCH2_MASK) && !previousLoopButtonStatus[1];
}

bool switch3Pressed() {
  return (mcp23017_GPIOB & SWITCH3_MASK) && !previousLoopButtonStatus[2];
}

bool positionChanged() {
  return currentAcceptedPosition != previousLoopAcceptedPosition;
}

void sendGPIOA() {
  Wire.beginTransmission(MCP23017_ADDRESS);
  Wire.write(GPIOA);
  Wire.write(mcp23017_GPIOA);
  Wire.endTransmission();
}

void debug(const __FlashStringHelper * string) {
  Serial.println(string);
}

void turnBuzzerOff() {
  turnGPIOAFlagOff(BUZZER_MASK);
}

void beep() {
  debug(F("beep"));
  turnGPIOAFlagOn(BUZZER_MASK);
  timer.setTimeout(100, &turnBuzzerOff);
}

void longBeep() {
  debug(F("longbeep"));
  turnGPIOAFlagOn(BUZZER_MASK);
  timer.setTimeout(300, &turnBuzzerOff);
}

void blinkCallback() {
  blinkStatus ^= true;
}

// Sets the string on the last row of the display reading data from program memory
void setButtonLabel_P(bool button, const char * label) {
  int position = button ? 10 : 0;
  display_print_P(3, position, label);
}

// Row is the display row in the range 0 to 3
void display_printf_P(byte row, const char * fmt, ...) {
  char buffer[21];
  va_list ap;

  va_start(ap, fmt);
  vsprintf_P(buffer, fmt, ap);
  va_end(ap);

  lcd.setCursor(0, row);
  lcd.print(buffer);
}

void display_print_P(byte row, byte pos, const char * content) {
  char buffer[21];
  strcpy_P(buffer, content);
  lcd.setCursor(pos, row);
  lcd.print(buffer);
}
