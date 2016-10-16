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

#ifndef MCP23017_H
#define MCP23017_H

#define MCP23017_I2C_ADDRESS 0x20
#define IODIRA_ADDRESS 0x00
#define GPPUA_ADDRESS 0x0C
#define GPIOA_ADDRESS 0x12
#define GPIOB_ADDRESS 0x13


class MCP23017 {
  public:
    MCP23017();

    void init(uint8_t iodira, uint8_t iodirb, uint8_t iopola, uint8_t iopolb,
      uint8_t gppua, uint8_t gppub);

    void writeGPIOA(uint8_t value);
    void writeGPIOB(uint8_t value);

    uint8_t readGPIOA();
    uint8_t readGPIOB();

  private:
    void writeTo(uint8_t address, uint8_t value);
    uint8_t readFrom(uint8_t address);
};

#endif
// MCP23017_H
