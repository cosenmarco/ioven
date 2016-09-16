// Date and time functions using a DS1307 RTC connected via I2C and Wire lib
#include <Wire.h>
#include "RTClib.h"
#include "max6675.h"
#include <LiquidCrystal.h>
#include <math.h>
#include "StateMachine.h"
#include "SimpleTimer.h"

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
char displayContent[4][21];

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
  bool dt = digitalRead(ENCODER_DT_PIN);
  if (dt) {
    encoderPositionSinceLastLoop --;
  } else {
    encoderPositionSinceLastLoop ++;
  }
}

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
  pinMode(ENCODER_CLK_PIN,INPUT);
  pinMode(ENCODER_DT_PIN,INPUT);
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

  printDisplay();
  encoderPositionSinceLastLoop = 0;
}

void readInputs() {
  now = rtc.now();
  currentTemperature = thermocouple.readCelsius();

  // Read the status of switches & knob
  Wire.beginTransmission(MCP23017_ADDRESS);
  Wire.write(GPIOB);
  Wire.requestFrom(MCP23017_ADDRESS,1);
  mcp23017_GPIOB = Wire.read();
  Wire.endTransmission();
}

void updateDisplayTemperature() {
  // Second line: temperature
  sprintf(displayContent[1], "%3d\1C", round(currentTemperature));
}

void updateDisplayClock() {
  // First line: clock
  sprintf(displayContent[0], "%s %02d/%02d/%02d %02d:%02d:%02d", daysOfTheWeek[now.dayOfTheWeek()], now.day(),
    now.month(), now.year() % 100, now.hour(), now.minute(), now.second());
}

void printDisplay() {
  lcd.setCursor(0, 0);
  lcd.print(displayContent[0]);

  lcd.setCursor(0, 1);
  lcd.print(displayContent[1]);

  lcd.setCursor(0, 2);
  lcd.print(displayContent[2]);

  lcd.setCursor(0, 3);
  lcd.print(displayContent[3]);
}

// ###############  State Machine functions ###############
void offStateEnter() {
  debug("offStateEnter");
  turnGPIOAFlagOff(DISPLAY_MASK);
  longBeep();
  sprintf(displayContent[2], "  timer");
}

void offStateExit() {
  debug("offStateExit");
  turnGPIOAFlagOn(DISPLAY_MASK);
  beep();
}

void offStateLoop() {
}

void justTimerSetEnter() {
  debug("justTimerSetEnter");
  justTimerSetTimeout = timer.setTimeout(30000, &invokeCancelJustTimerSet);
  justTimerSeconds = 60; // Init timer to default 60 seconds
  isMinutes = false;
  sprintf(displayContent[2], "   start    cancel");
}

void justTimerSetExit() {
  debug("justTimerSetExit");
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

  int sum = increment + justTimerSeconds;

  if(sum >= 5) {
    justTimerSeconds = sum;
  } else {
    justTimerSeconds = 5;
  }

  if(increment != 0 && justTimerSetTimeout >= 0) {
    timer.restartTimer(justTimerSetTimeout);
  }

  if(!isMinutes && switch3Pressed()) {
    isMinutes = true;
  }

  sprintf(displayContent[2], "      %1d:%02d:%02d", floor(justTimerSeconds / 3600), floor((justTimerSeconds / 60) % 60),
    justTimerSeconds % 60 );
}

void decrementTimer() {
  if (justTimerSeconds > 0) {
    justTimerSeconds--;
  }
}

void justTimerRunEnter() {
  debug("justTimerRunEnter");
  justTimerDecrementerInterval = timer.setInterval(1000, &decrementTimer);
}

void justTimerRunExit() {
  debug("justTimerRunExit");
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
  debug("alarmEnter");
  alarmInterval = timer.setInterval(300, &toggleAlarm);
  alarmTimeoutTimerId = timer.setTimeout(15000, &alarmTimeout);
}

void alarmExit() {
  debug("alarmExit");
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
  mcp23017_GPIOA &= !mask;
  sendGPIOA();
}

void turnGPIOAFlagOn(byte mask) {
  mcp23017_GPIOA |= mask;
  sendGPIOA();
}

bool switch1Pressed() {
  return (mcp23017_GPIOB & SWITCH1_MASK) != 0;
}

bool switch2Pressed() {
  return (mcp23017_GPIOB & SWITCH2_MASK) != 0;
}

bool switch3Pressed() {
  return (mcp23017_GPIOB & SWITCH3_MASK) != 0;
}

void sendGPIOA() {
  Wire.beginTransmission(MCP23017_ADDRESS);
  Wire.write(GPIOA);
  Wire.write(mcp23017_GPIOA);
  Wire.endTransmission();
}

void debug(char * string) {
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
