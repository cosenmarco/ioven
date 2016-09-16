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

#define DISPLAY_MASK 0x01 // The display bit is GPA0
#define BUZZER_MASK 0x02 // The buzzer bit is GPA1
#define SWITCH1_MASK 0x20 // Switch 1 is on GPIOB 5
#define SWITCH2_MASK 0x40 // Switch 2 is on GPIOB 6
#define SWITCH3_MASK 0x80 // Switch 3 is on GPIOB 7 (active low)

#define ENCODER_CLK_PIN 3
#define ENCODER_DT_PIN 4

// String pool in program memory
const char empty_button_label[] PROGMEM = "          ";
const char timer_button_label[] PROGMEM = "   timer  ";
const char start_button_label[] PROGMEM = "   start  ";
const char cancel_button_label[] PROGMEM = " annulla  ";

const char date_row_fmt[] PROGMEM = "%s %02d/%02d/%02d %02d:%02d:%02d";
const char temperature_row_fmt[] PROGMEM = "%3d\1C";
const char timer_row_fmt[] PROGMEM = "      %1d:%02d:%02d";
const char empty_row_fmt[] PROGMEM = "                    ";

LiquidCrystal lcd(10, 9, 8, 7, 6, 5);
RTC_DS1307 rtc;
MAX6675 thermocouple(13, 12, 11);
DateTime now;

byte mcp23017_GPIOA, mcp23017_GPIOB;
double currentTemperature;

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
int justTimerSetTimeout = -1; // This timer is used to cancel automatically if we stay too long in justTimerSetState
int justTimerDecrementerInterval = -1; // This interval decrements the timer every second
int alarmInterval = -1; // This is used to toggle between beep on and beep off state to make an alarm
int alarmTimeoutTimerId = -1;

int justTimerSeconds;
bool isMinutes;

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
State justTimerSetState(&justTimerSetEnter, &justTimerSetRunLoop, &justTimerSetExit);
State justTimerRunState(&justTimerRunEnter, &justTimerSetRunLoop, &justTimerRunExit); // Note: same loop function of justTimerSetState
State alarmState(&alarmEnter, NULL, &alarmExit);

Transition fromOffToJustTimerSet = {
  offState,
  justTimerSetState,
  &switch1Pressed
};

Transition fromJustTimerSetToOff = {
  justTimerSetState,
  offState,
  &switch2Pressed
};

Transition fromJustTimerSetToJustTimerRun = {
  justTimerSetState,
  justTimerRunState,
  &switch1Pressed
};

Transition fromJustTimerRunToOff = {
  justTimerRunState,
  offState,
  &switch2Pressed
};

Transition fromJustTimerRunToAlarm = {
  justTimerRunState,
  alarmState,
  &from_justttimer_run_to_alarm
};

Transition fromAlarmToOff = {
  alarmState,
  offState,
  &from_alarm_to_off
};

#define TRANSITIONS_NUM 6
Transition transitions[TRANSITIONS_NUM] = {
  fromOffToJustTimerSet, fromJustTimerSetToOff, fromJustTimerSetToJustTimerRun, fromJustTimerRunToOff,
  fromJustTimerRunToAlarm, fromAlarmToOff
};

StateMachine iOvenStateMachine(offState,transitions, TRANSITIONS_NUM);

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
  Wire.write(0xE0); // Next register is IODIRB. Set all of port B to outputs except 5,6,7
  Wire.write(0x00); // Next register is IPOLA. Set all of ports to normal
  Wire.write(0x80); // Next register is IPOLB. Set GPB7 as active low
  Wire.endTransmission();

  // Setup rotary encoder
  pinMode(ENCODER_CLK_PIN, INPUT);
  pinMode(ENCODER_DT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK_PIN), doEncoderClkTick, FALLING);


  // wait for thermocouple chip to stabilize
  delay(500);

  offStateEnter(); // We start from the state OFF
}

void loop () {
  readInputs();

  updateDisplayClock();
  updateDisplayTemperature();

  iOvenStateMachine.loop();

  //printDisplay();
  encoderPositionSinceLastLoop = 0;
  timer.run();
  setPreviousLoopButtonsStatus();
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
}

void setPreviousLoopButtonsStatus() {
  previousLoopButtonStatus[0] = (mcp23017_GPIOB & SWITCH1_MASK) != 0;
  previousLoopButtonStatus[1] = (mcp23017_GPIOB & SWITCH2_MASK) != 0;
  previousLoopButtonStatus[2] = (mcp23017_GPIOB & SWITCH3_MASK) != 0;
}

void updateDisplayTemperature() {
  // Second line: temperature
  display_printf_P(1, temperature_row_fmt, round(currentTemperature));
}

void updateDisplayClock() {
  // First line: clock
  display_printf_P(0, date_row_fmt, daysOfTheWeek[now.dayOfTheWeek()], now.day(),
    now.month(), now.year() % 100, now.hour(), now.minute(), now.second());
}

// ###############  State Machine functions ###############
void offStateEnter() {
  debug(F("offStateEnter"));
  turnGPIOAFlagOff(DISPLAY_MASK);
  longBeep();
  display_printf_P(2, empty_row_fmt);
  setButtonLabel(0, timer_button_label);
  setButtonLabel(1, empty_button_label);
}

void offStateExit() {
  debug(F("offStateExit"));
  turnGPIOAFlagOn(DISPLAY_MASK);
  beep();
}

void offStateLoop() {
}

void justTimerSetEnter() {
  debug(F("justTimerSetEnter"));
  justTimerSetTimeout = timer.setTimeout(10000, &invokeCancelJustTimerSet);
  justTimerSeconds = 60; // Init timer to default 60 seconds
  isMinutes = false;
  setButtonLabel(0, start_button_label);
  setButtonLabel(1, cancel_button_label);
}

void justTimerSetExit() {
  debug(F("justTimerSetExit"));
  if(justTimerSetTimeout >= 0){
    timer.deleteTimer(justTimerSetTimeout);
    justTimerSetTimeout = -1;
  }
}

void invokeCancelJustTimerSet() {
  iOvenStateMachine.performTransitionNow(fromJustTimerSetToOff);
  justTimerSetTimeout = -1;
}

void justTimerSetRunLoop() {
  int increment;
  if(isMinutes) {
    increment = encoderPositionSinceLastLoop * 60; // Every tick is 1 minute
  } else {
    increment = encoderPositionSinceLastLoop * 5; // Every tick is 5 seconds
  }

  if(increment != 0) {
    int sum = increment + justTimerSeconds;

    if(sum >= 5) {
      justTimerSeconds = sum;
    } else {
      justTimerSeconds = 5;
    }

    if(justTimerSetTimeout >= 0) {
      timer.restartTimer(justTimerSetTimeout);
    }
  }

  if(!isMinutes && switch3Pressed()) {
    isMinutes = true;
  }

  display_printf_P(2, timer_row_fmt, justTimerSeconds / 3600, (justTimerSeconds / 60) % 60,
    justTimerSeconds % 60 );
}

void decrementTimer() {
  if (justTimerSeconds > 0) {
    justTimerSeconds--;
  }
}

void justTimerRunEnter() {
  debug(F("justTimerRunEnter"));
  justTimerDecrementerInterval = timer.setInterval(1000, &decrementTimer);
  setButtonLabel(0, empty_button_label);
}

void justTimerRunExit() {
  debug(F("justTimerRunExit"));
  timer.deleteTimer(justTimerDecrementerInterval);
}

bool from_justttimer_run_to_alarm() {
  return justTimerSeconds == 0;
}

void toggleAlarm() {
  mcp23017_GPIOA ^= BUZZER_MASK;
  sendGPIOA();
}

void alarmEnter() {
  debug(F("alarmEnter"));
  alarmInterval = timer.setInterval(300, &toggleAlarm);
  alarmTimeoutTimerId = timer.setTimeout(15000, &alarmTimeout);
  setButtonLabel(0, cancel_button_label);
}

void alarmExit() {
  debug(F("alarmExit"));
  timer.deleteTimer(alarmInterval);
  if(alarmTimeoutTimerId >= 0) {
    timer.deleteTimer(alarmTimeoutTimerId);
    alarmTimeoutTimerId = -1;
  }
  turnGPIOAFlagOff(BUZZER_MASK);
}

void alarmTimeout() {
  iOvenStateMachine.performTransitionNow(fromAlarmToOff);
  alarmTimeoutTimerId = -1;
}

bool from_alarm_to_off() {
  return switch1Pressed() || switch2Pressed();
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
  turnGPIOAFlagOn(BUZZER_MASK);
  timer.setTimeout(100, &turnBuzzerOff);
}

void longBeep() {
  turnGPIOAFlagOn(BUZZER_MASK);
  timer.setTimeout(300, &turnBuzzerOff);
}

// Sets the string on the last row of the display reading data from program memory
void setButtonLabel(bool button, const char * label) {
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
