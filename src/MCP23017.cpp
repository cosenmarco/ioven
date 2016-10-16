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

#include <Wire.h>
#include "MCP23017.h"

MCP23017::MCP23017() {}

void MCP23017::init(uint8_t iodira, uint8_t iodirb, uint8_t ipola, uint8_t ipolb,
    uint8_t gppua, uint8_t gppub) {
  Wire.beginTransmission(MCP23017_I2C_ADDRESS);
  Wire.write(IODIRA_ADDRESS);
  Wire.write(iodira);
  Wire.write(iodirb);
  Wire.write(ipola);
  Wire.write(ipolb);
  Wire.endTransmission();

  Wire.beginTransmission(MCP23017_I2C_ADDRESS);
  Wire.write(GPPUA_ADDRESS);
  Wire.write(gppua);
  Wire.write(gppub);
  Wire.endTransmission();
}

void MCP23017::writeTo(uint8_t address, uint8_t value) {
  Wire.beginTransmission(MCP23017_I2C_ADDRESS);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission();
}

void MCP23017::writeGPIOA(uint8_t value) {
  writeTo(GPIOA_ADDRESS, value);
}

void MCP23017::writeGPIOB(uint8_t value) {
  writeTo(GPIOB_ADDRESS, value);
}

uint8_t MCP23017::readFrom(uint8_t address) {
  Wire.beginTransmission(MCP23017_I2C_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(MCP23017_I2C_ADDRESS, 1);
  return Wire.read();
}

uint8_t MCP23017::readGPIOA() {
  return readFrom(GPIOA_ADDRESS);
}

uint8_t MCP23017::readGPIOB() {
  return readFrom(GPIOB_ADDRESS);
}
