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

#include <avr/pgmspace.h>
#include <pins_arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <max6675.h>
#include <LiquidCrystal.h>
#include <math.h>
#include "MCP23017.h"
#include "StateMachine.h"
#include "SimpleTimer.h"

#define MAX_TEMP 230
#define HYSTERESIS_DEGREES 3

// Relais are inverted logic so we have to negate the relais pins
#define RELAIS_MASK_GPIOB B00001111

#define BUZZER_PIN 13
#define DISPLAY_RS_PIN 4
#define DISPLAY_E_PIN 5
#define DISPLAY_D4_PIN 6
#define DISPLAY_D5_PIN 7
#define DISPLAY_D6_PIN 8
#define DISPLAY_D7_PIN 9

#define THERMO_SCK_PIN 12
#define THERMO_CS_PIN 11
#define THERMO_SO_PIN 10

#define ENCODER_CLK_PIN 2
#define ENCODER_DT_PIN 3

// Outputs: on GPB
#define DISPLAY_MASK        B00000001 // GPIOA0
#define LIGHT               B00000011 // GPIOB0 and GPB1
#define INDICATOR           B00000100 // GPIOB2
#define VENTILATOR          B00001000 // GPIOB3
#define R1                  B00010000 // GPIOB4
#define R2                  B00100000 // GPIOB5
#define R3                  B01000000 // GPIOB6
#define R4                  B10000000 // GPIOB7

// Useful to turn everything off
#define ALL_HEATING_ELEMENTS (R1 | R2 | R3 | R4)
#define ALL_OVEN_OUTPUTS_GPIOA (ALL_HEATING_ELEMENTS | VENTILATOR | INDICATOR)

// Inputs: on GPA
#define SWITCH1_MASK B00100000 // Switch 1 is on GPIOA 5
#define SWITCH2_MASK B01000000 // Switch 2 is on GPIOA 6
#define SWITCH3_MASK B10000000 // Switch 3 is on GPIOA 7 (active low)

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
#define CHICKEN_ELEMENTS (R1 | R3)
#define THREE_ELEMENTS (R2 | R4)
#define PIZZA_ELEMENTS (R1 | R2 | R3)
#define GRILL_ELEMENTS (R3 | R4)
#define GRILL_VENT_ELEMENTS (R3 | R4)
#define CAKE_ELEMENTS (R2)

// Programs
#define OFF_PROGRAM 0
#define LIGHT_PROGRAM 1
#define CHICKEN_PROGRAM 2
#define THREE_ELEMENTS_PROGRAM 3
#define PIZZA_PROGRAM 4
#define GRILL_PROGRAM 5
#define GRILL_VENT_PROGRAM 6
#define CAKE_PROGRAM 7

// ################# Strings stored in program memory #########################
// The labels must be 10 chars wide (excluded terminator)
const char empty_button_label[] PROGMEM = "          ";
const char clock_adj_button_label[] PROGMEM = " set clock";
const char timer_button_label[] PROGMEM = "   timer  ";
const char start_button_label[] PROGMEM = "   start  ";
const char cancel_button_label[] PROGMEM = " annulla  ";
const char continue_button_label[] PROGMEM = " continua ";
const char shut_down_button_label[] PROGMEM = "  spegni  ";
const char save_button_label[] PROGMEM = "  salva   ";
const char arrow_button_label[] PROGMEM = "     \x7E    ";

// Status string. These strings must be 8 chars wide (excluded terminator)
const char off[] PROGMEM = "  -OFF- ";
const char light[] PROGMEM = "  LUCE  ";
const char chicken[] PROGMEM = "  POLLO ";
const char three[] PROGMEM = " 3 ELEM ";
const char pizza[] PROGMEM = "  PIZZA ";
const char grill[] PROGMEM = "  GRILL ";
const char grillvent[] PROGMEM = " GRILL V";
const char cake[] PROGMEM = "  TORTA ";


const char date_row_fmt[] PROGMEM = "%s %02d/%02d/%02d %02d:%02d:%02d";
const char temperature_row_fmt[] PROGMEM = "%3d\1C->%3d\1C%s";
const char timer_row_fmt[] PROGMEM = "%2d:%02d:%02d -> %2d:%02d:%02d";

const char time_over_message[] PROGMEM = "   TEMPO  SCADUTO   ";
const char oven_message[] PROGMEM = "       FORNO        ";
const char banner_row_fmt[] PROGMEM = " \x7F\x7F\x7F\x7F\x7F iOven \x7E\x7E\x7E\x7E\x7E  ";
// ################# END Strings stored in program memory #########################

// Peripherals
LiquidCrystal lcd(DISPLAY_RS_PIN,
  DISPLAY_E_PIN,
  DISPLAY_D4_PIN,
  DISPLAY_D5_PIN,
  DISPLAY_D6_PIN,
  DISPLAY_D7_PIN);
RTC_DS1307 rtc;
MAX6675 thermocouple(THERMO_SCK_PIN, THERMO_CS_PIN, THERMO_SO_PIN);
MCP23017 expander;

DateTime now, otherDateTime;

byte mcp23017_GPIOA = 0, mcp23017_GPIOB = 0;

int currentTemperature;
int temperatureGoal;

bool currentHeaterStatus;
bool previousLoopHeaterStatus;

byte currentProgram = OFF;
byte previousLoopProgram;

int currentPosition = OFF_POSITION;
int previousLoopPosition;

byte currentAcceptedPosition;
byte previousLoopAcceptedPosition;

byte heatingElementsConfigurationByProgram[] = { 0, 0, CHICKEN_ELEMENTS, THREE_ELEMENTS, PIZZA_ELEMENTS, GRILL_ELEMENTS,
  GRILL_VENT_ELEMENTS, CAKE_ELEMENTS};
bool ventilatorByProgram[] = { false, false, true, true, true, false, true, true };
const char * programStringsByProgram[] = { off, light, chicken, three, pizza, grill, grillvent, cake };

char daysOfTheWeek[7][3] = {"Do", "Lu", "Ma", "Me", "Gi", "Ve", "Sa"};
byte degreeSymbol[8] = {
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
int temporaryStateTimeout = -1; // This timer is used to cancel automatically if we stay too long in some temporary state
int timerDecrementerInterval = -1; // This interval decrements the timer every second
int alarmTimeoutTimerId = -1;
int positionChangeTimer = -1; // Change in position knob is considered only after 2 seconds of stable knob position

unsigned int timerSeconds;
bool isMinutes;
bool blinkStatus = true;
byte clockAdjustPosition = 0;

// Rotary encoder data and handler
volatile int encoderPositionSinceLastLoop = 0;
void doEncoderClkTick() {
  if(digitalRead(ENCODER_CLK_PIN) == digitalRead(ENCODER_DT_PIN)) {
    encoderPositionSinceLastLoop ++;
  } else {
    encoderPositionSinceLastLoop --;
  }
}

bool previousLoopButtonStatus[3] = { false, false, false };
char printBuffer[25];

// ################# The State Machine #################
State offState(&offStateEnter, 0, &offStateExit);
State ovenState(&ovenStateEnter, &ovenLoop, 0);
State timerSetState(&timerSetEnter, &timerSetLoop, &timerSetExit);
State timerRunState(&timerRunEnter, &timerRunLoop, &timerRunExit); // Note: same loop function of timerSetState
State alarmState(&alarmEnter, &alarmLoop, &alarmExit);
State clockAdjust(&clockAdjustEnter, &clockAdjustLoop, &clockAdjustExit);

Transition fromAnyToOff = {
  *StateMachine::ANY,
  offState,
  &from_any_to_off
};

Transition fromOffToClockAdjust = {
  offState,
  clockAdjust,
  &switch2Pressed
};

Transition fromClockAdjustToOff = {
  clockAdjust,
  offState,
  &from_clock_adjust_to_off
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

#define TRANSITIONS_NUM 15
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
  fromAlarmToOven,
  fromOffToClockAdjust,
  fromClockAdjustToOff
};

StateMachine iOvenStateMachine(offState, transitions, TRANSITIONS_NUM);

void display_printf_P(byte row, const char * fmt, ...);


// ############## Setup function. Executed once at startup ################
void setup () {
  Serial.begin(9600);
  Wire.begin();

  delay(500);

  if (!rtc.isrunning()) {
    Serial.println(F("RTC is NOT running!"));
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Setup rotary encoder
  pinMode(ENCODER_CLK_PIN, INPUT);
  pinMode(ENCODER_DT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK_PIN), doEncoderClkTick, FALLING);

  // Setup Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  rtc.writeSqwPinMode(SquareWave4kHz);

  // Setup LCD
  lcd.createChar(1, degreeSymbol);
  lcd.begin(20, 4);

  // Initialize MCP23017
  expander.init(
    B11111110, // IODIRA Set all of port A to inputs except 0
    B00000000, // IODIRB Set all of port B to output
    B11100000, // IPOLA Set GPA7,6,5 as active low
    B00000000, // IPOLB Set all of ports to normal
    B11100000, // GPPUA Set GPA7,6,5 with pull up resistors
    B00000000  // GPPUB No pull up needed
  );

  sendGPIOA();
  sendGPIOB();

  readInputs();
  offStateEnter(); // We start from the state OFF
  resetPreviousVariables();

  // Setup sample timers to read inputs and update display on a regular basis
  timer.setInterval(250, &sampledReadInputs);
  timer.setInterval(250, &sampledUpdateDisplay);

  // Universal blinker. Used to blink something on the display.
  timer.setInterval(400, &blinkCallback);

  Serial.println(F("Init complete!!!"));
}


// ############## LOOP and related functions ################
void loop () {
  readInputs();

  timer.run(); // This has to happen first because the result of some timouts needs to be caught by logic checks
  iOvenStateMachine.loop();
  currentPositionLoop();
  currentProgramLoop(); // This must be the last because decisions are taken earlier about currentProgram

  resetPreviousVariables();
}

void resetPreviousVariables() {
  previousLoopHeaterStatus = currentHeaterStatus;
  previousLoopProgram = currentProgram;
  previousLoopPosition = currentPosition;
  previousLoopAcceptedPosition = currentAcceptedPosition;
  encoderPositionSinceLastLoop = 0;
  previousLoopButtonStatus[0] = (mcp23017_GPIOA & SWITCH1_MASK) != 0;
  previousLoopButtonStatus[1] = (mcp23017_GPIOA & SWITCH2_MASK) != 0;
  previousLoopButtonStatus[2] = (mcp23017_GPIOA & SWITCH3_MASK) != 0;
}

void sampledReadInputs() {
  // Affected by the problem described in https://forum.arduino.cc/index.php?topic=189283.0
  DateTime result = rtc.now();
  if(result.month() < 13) {
    now = result;
  }
  currentTemperature = (int) lround(thermocouple.readCelsius());
}

void sampledUpdateDisplay() {
  updateDisplayClockLine();
  updateDisplayStatusLine();
}

void readInputs() {
  // Read the status of switches
  mcp23017_GPIOA = expander.readGPIOA();
}

void currentPositionLoop() {
  if( switch3CurrentlyPressed() && encoderPositionSinceLastLoop != 0) {
    // When SW3 is pressed and at same time encoder is rotated: we change position
    currentPosition = (((currentPosition + encoderPositionSinceLastLoop) % 8) + 8) % 8;
    // Do not affect other states: we took the encoder turns for us here.
    encoderPositionSinceLastLoop = 0;
  }
  if(currentPosition != previousLoopPosition) {
    Serial.print(F("Position changed to "));
    Serial.println(currentPosition);
    if(positionChangeTimer >= 0) {
      timer.restartTimer(positionChangeTimer);
    } else {
      positionChangeTimer = timer.setTimeout(2000, &acceptPosition);
    }
  }
}

// Callback fired when there is a change in the positions knob
void acceptPosition() {
  if(currentAcceptedPosition != currentPosition) {
    currentAcceptedPosition = currentPosition;
    Serial.print(F("Accepted New Position: "));
    Serial.println(currentAcceptedPosition);
  }
  positionChangeTimer = -1;
}

void updateDisplayClockLine() {
  if(!iOvenStateMachine.isInState(clockAdjust)) {
    display_printf_P(0, date_row_fmt, daysOfTheWeek[now.dayOfTheWeek()], now.day(),
      now.month(), now.year() % 100, now.hour(), now.minute(), now.second());
  }
}

void updateDisplayStatusLine() {
  char buffer[9];
  strcpy_P(buffer, programStringsByProgram[currentPosition]);
  display_printf_P(1, temperature_row_fmt,
    currentTemperature,
    temperatureGoal,
    buffer
  );
}

void currentProgramLoop() {
  // Temperature control logic: hysteresis
  if(previousLoopHeaterStatus) {
    currentHeaterStatus = currentTemperature < temperatureGoal;
  } else {
    currentHeaterStatus = currentTemperature < (temperatureGoal - HYSTERESIS_DEGREES);
  }
  currentHeaterStatus = currentHeaterStatus && isHeatState();

  if(currentHeaterStatus != previousLoopHeaterStatus || currentProgram != previousLoopProgram) {
    // Either heater status or program changed. We have to update the outputs.
    // First: shut down everything
    mcp23017_GPIOB = 0;

    // Then turn on stuff one by one again
    if(currentHeaterStatus) {
      mcp23017_GPIOB |= heatingElementsConfigurationByProgram[currentProgram];
      if(heatingElementsConfigurationByProgram[currentProgram] != 0) {
        mcp23017_GPIOB |= INDICATOR;
      }
    }
    if(ventilatorByProgram[currentProgram]) {
      mcp23017_GPIOB |= VENTILATOR;
    }
    if(currentProgram != OFF_PROGRAM) {
      mcp23017_GPIOB |= LIGHT;
    }

    // Finally send outputs
    sendGPIOB();

    Serial.print(F("currentHeaterStatus: "));
    Serial.print(currentHeaterStatus);
    Serial.print(F("  currentProgram: "));
    Serial.println(currentProgram);
  }
}

// ########################################################
// ###############  State Machine functions ###############
// ########################################################

void offStateEnter() {
  Serial.println(F("offStateEnter"));
  longBeep();
  turnGPIOAFlagOff(DISPLAY_MASK);
  display_print_P(2, 0, banner_row_fmt);
  setButtonLabel_P(0, timer_button_label);
  setButtonLabel_P(1, clock_adj_button_label);

  currentProgram = OFF_PROGRAM;
  currentPosition = OFF_POSITION;
  currentAcceptedPosition = OFF_POSITION;

  // Some init that are useful for the next time the oven is turned on
  timerSeconds = 60;
  temperatureGoal = 60.0;
}

void offStateExit() {
  Serial.println(F("offStateExit"));
  turnGPIOAFlagOn(DISPLAY_MASK);
  beep();
}

void ovenStateEnter() {
  Serial.println(F("ovenEnter"));
  display_print_P(2, 0, oven_message);
  setButtonLabel_P(0, timer_button_label);
  setButtonLabel_P(1, empty_button_label);
}

void ovenLoop() {
  temperatureAdjustLoop();
  currentProgramActuationLoop();
}

void temperatureAdjustLoop() {
  if(!switch3CurrentlyPressed()) {
    temperatureGoal += encoderPositionSinceLastLoop;
    if(temperatureGoal < 0) {
      temperatureGoal = 0;
    }
    if(temperatureGoal > MAX_TEMP) {
      temperatureGoal = MAX_TEMP;
    }
  }
}

void currentProgramActuationLoop() {
  currentProgram = currentAcceptedPosition;
}

void timerSetEnter() {
  Serial.println(F("timerSetEnter"));
  temporaryStateTimeout = timer.setTimeout(20000, &invokeCancelTimerSet);
  isMinutes = true;
  setButtonLabel_P(0, start_button_label);
  setButtonLabel_P(1, cancel_button_label);
}

void timerSetExit() {
  Serial.println(F("timerSetExit"));
  if(temporaryStateTimeout >= 0) {
    timer.deleteTimer(temporaryStateTimeout);
    temporaryStateTimeout = -1;
  }
}

void invokeCancelTimerSet() {
  Serial.println(F("invokeCancelTimerSet"));
  if(currentProgram == OFF_PROGRAM) {
    iOvenStateMachine.performTransitionNow(fromTimerSetToOff);
  } else {
    iOvenStateMachine.performTransitionNow(fromTimerSetToOven);
  }
  temporaryStateTimeout = -1;
}

void timerSetLoop() {
  if(!switch3CurrentlyPressed() && encoderPositionSinceLastLoop != 0) {
    int sum;
    if(isMinutes) {
      sum = encoderPositionSinceLastLoop * 60 + timerSeconds; // Every tick is 1 minute
    } else {
      sum = encoderPositionSinceLastLoop      + timerSeconds; // Every tick is 1 second
    }

    if(sum >= 0) {
      timerSeconds = sum;
    } else {
      timerSeconds = 0;
    }

    if(temporaryStateTimeout >= 0) {
      timer.restartTimer(temporaryStateTimeout);
    }
  }

  isMinutes ^= switch3Pressed();

  otherDateTime = now + TimeSpan(timerSeconds);
  sprintf_P(printBuffer, timer_row_fmt, timerSeconds / 3600, (timerSeconds / 60) % 60,
    timerSeconds % 60, otherDateTime.hour(), otherDateTime.minute(), otherDateTime.second());

  if(!blinkStatus) {
    if(isMinutes) {
      printBuffer[3] = ' ';
      printBuffer[4] = ' ';
    } else {
      printBuffer[6] = ' ';
      printBuffer[7] = ' ';
    }
  }

  lcd.setCursor(0, 2);
  lcd.print(printBuffer);

  currentProgramActuationLoop();
}

void decrementTimer() {
  if (timerSeconds > 0) {
    timerSeconds--;
  }
}

void timerRunEnter() {
  Serial.println(F("timerRunEnter"));
  timerDecrementerInterval = timer.setInterval(1000, &decrementTimer);
  setButtonLabel_P(0, timer_button_label);
}

void timerRunExit() {
  Serial.println(F("timerRunExit"));
  timer.deleteTimer(timerDecrementerInterval);
  timerDecrementerInterval = -1;
}

void timerRunLoop() {
  temperatureAdjustLoop();
  currentProgramActuationLoop();
  display_printf_P(2, timer_row_fmt, timerSeconds / 3600, (timerSeconds / 60) % 60,
    timerSeconds % 60, otherDateTime.hour(), otherDateTime.minute(), otherDateTime.second());
}

bool from_timer_run_to_alarm() {
  return timerSeconds <= 0;
}

void alarmEnter() {
  Serial.println(F("alarmEnter"));
  alarmTimeoutTimerId = timer.setTimeout(15000, &alarmTimeout);
  setButtonLabel_P(0, shut_down_button_label);
  if(currentProgram == OFF_PROGRAM) {
    setButtonLabel_P(1, empty_button_label);
  } else {
    setButtonLabel_P(1, continue_button_label);
  }
  display_print_P(2, 0, time_over_message);
}

void alarmExit() {
  Serial.println(F("alarmExit"));
  if(alarmTimeoutTimerId >= 0) {
    timer.deleteTimer(alarmTimeoutTimerId);
    alarmTimeoutTimerId = -1;
  }
  turnBuzzerOff();
}

void alarmTimeout() {
  Serial.println(F("alarmTimeout"));
  iOvenStateMachine.performTransitionNow(fromAlarmToOff);
  alarmTimeoutTimerId = -1;
}

void alarmLoop() {
  ovenLoop();
  if(blinkStatus) {
    turnBuzzerOn();
  } else {
    turnBuzzerOff();
  }
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

void clockAdjustEnter() {
  Serial.println(F("clockAdjustEnter"));
  otherDateTime = DateTime(now);
  setButtonLabel_P(0, arrow_button_label);
  setButtonLabel_P(1, save_button_label);
  temporaryStateTimeout = timer.setTimeout(20000, &invokeCancelTimerSet);
}

void invokeCancelClockAdjust() {
  iOvenStateMachine.performTransitionNow(fromClockAdjustToOff);
  temporaryStateTimeout = -1;
  clockAdjustPosition = 0;
}

void clockAdjustLoop() {
  byte blinkStartPos = 3 + clockAdjustPosition * 3;
  int newValue;

  if(encoderPositionSinceLastLoop != 0) {
    timer.restartTimer(temporaryStateTimeout);
    switch(clockAdjustPosition) {
      case 0: // Day
        otherDateTime = otherDateTime + TimeSpan(encoderPositionSinceLastLoop, 0, 0, 0);
        break;
      case 1: // Month
        newValue = otherDateTime.month() + encoderPositionSinceLastLoop;
        if(newValue < 1) {
          newValue = 1;
        }
        if(newValue > 12) {
          newValue = 12;
        }
        otherDateTime = DateTime(otherDateTime.year(), newValue, otherDateTime.day(),
          otherDateTime.hour(), otherDateTime.minute(), otherDateTime.second());
        break;
      case 2: // Year
        newValue = otherDateTime.year() + encoderPositionSinceLastLoop;
        if(newValue < 2000) {
          newValue = 2000;
        }
        otherDateTime = DateTime(newValue, otherDateTime.month(), otherDateTime.day(),
          otherDateTime.hour(), otherDateTime.minute(), otherDateTime.second());
        break;
      case 3: // Hour
        otherDateTime = otherDateTime + TimeSpan(0, encoderPositionSinceLastLoop, 0, 0);
        break;
      case 4: // Min
        otherDateTime = otherDateTime + TimeSpan(0, 0, encoderPositionSinceLastLoop, 0);
        break;
      case 5: // Sec
        otherDateTime = otherDateTime + TimeSpan(0, 0, 0, encoderPositionSinceLastLoop);
        break;
    }
  }

  if(switch1Pressed()) {
    timer.restartTimer(temporaryStateTimeout);
    clockAdjustPosition ++;
    if(clockAdjustPosition > 5) {
      clockAdjustPosition = 0;
    }
  }

  sprintf_P(printBuffer, date_row_fmt, daysOfTheWeek[otherDateTime.dayOfTheWeek()], otherDateTime.day(),
    otherDateTime.month(), otherDateTime.year() % 100, otherDateTime.hour(), otherDateTime.minute(),
    otherDateTime.second());

  if(!blinkStatus) {
    printBuffer[blinkStartPos] = ' ';
    printBuffer[blinkStartPos + 1] = ' ';
  }
  lcd.setCursor(0, 0);
  lcd.print(printBuffer);
}

void clockAdjustExit() {
  Serial.println(F("clockAdjustExit"));
  if(temporaryStateTimeout >= 0) {
    timer.deleteTimer(temporaryStateTimeout);
    temporaryStateTimeout = -1;
  }
}

bool from_clock_adjust_to_off() {
  if(switch2Pressed()) {
    rtc.adjust(otherDateTime);
    return true;
  }
  return false;
}


// ################################################
// ############### Helper functions ###############
// ################################################
void turnGPIOAFlagOff(byte mask) {
  mcp23017_GPIOA &= ~mask;
  sendGPIOA();
}

void turnGPIOAFlagOn(byte mask) {
  mcp23017_GPIOA |= mask;
  sendGPIOA();
}

bool switch1Pressed() {
  return (mcp23017_GPIOA & SWITCH1_MASK) && !previousLoopButtonStatus[0];
}

bool switch2Pressed() {
  return (mcp23017_GPIOA & SWITCH2_MASK) && !previousLoopButtonStatus[1];
}

bool switch3Pressed() {
  return switch3CurrentlyPressed() && !previousLoopButtonStatus[2];
}

bool switch3CurrentlyPressed() {
  return (mcp23017_GPIOA & SWITCH3_MASK);
}

bool positionChanged() {
  return currentAcceptedPosition != previousLoopAcceptedPosition;
}

void sendGPIOA() {
  expander.writeGPIOA(mcp23017_GPIOA);
}

void sendGPIOB() {
  expander.writeGPIOB(mcp23017_GPIOB ^ RELAIS_MASK_GPIOB);
}

void turnBuzzerOn() {
  digitalWrite(BUZZER_PIN, HIGH);
}

void turnBuzzerOff() {
  digitalWrite(BUZZER_PIN, LOW);
}

void beep() {
  turnBuzzerOn();
  timer.setTimeout(50, &turnBuzzerOff);
}

void longBeep() {
  turnBuzzerOn();
  timer.setTimeout(150, &turnBuzzerOff);
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
  va_list ap;

  va_start(ap, fmt);
  vsprintf_P(printBuffer, fmt, ap);
  va_end(ap);

  if(strlen(printBuffer) > 20) {
    Serial.print(F("LONG STRING!!!: "));
    Serial.println(printBuffer);
  }

  lcd.setCursor(0, row);
  lcd.print(printBuffer);
}

void display_print_P(byte row, byte pos, const char * content) {
  strcpy_P(printBuffer, content);
  lcd.setCursor(pos, row);
  lcd.print(printBuffer);
}

bool isHeatState() {
  return iOvenStateMachine.isInState(ovenState) ||
    iOvenStateMachine.isInState(timerSetState) ||
    iOvenStateMachine.isInState(timerRunState);
}
