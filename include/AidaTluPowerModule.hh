#ifndef H_AIDATLUPOWERMODULE_HH
#define H_AIDATLUPOWERMODULE_HH

#include <string>
#include <iostream>
#include <vector>
#include "AidaTluI2c.hh"
#include "AidaTluHardware.hh"
#include <array>

class AidaTluPowerModule{
private:
  char m_pwr_i2c_DACaddr;
  char m_pwr_i2c_exp1Add;
  char m_pwr_i2c_exp2Add;
  char m_pwr_i2c_eeprom;
  i2cCore * pwr_i2c_core;
  AD5665R pwr_zeDAC;
  PCA9539PW pwr_ledExp1, pwr_ledExp2;
  std::array<std::array<int, 3>, 11> indicatorXYZ;

public:
  AidaTluPowerModule();
  void led_allBlue();
  void led_allGreen();
  void led_allRed();
  void led_allOff();
  void led_allWhite();
  void initI2Cslaves(bool intRef, uint8_t verbose);
  uint32_t _set_bit(uint32_t v, int index, bool x);
  void setIndicatorRGB(int indicator, const std::array<int, 3>& RGB, uint8_t verbose);
  void setI2CPar( i2cCore  *mycore , char DACaddr, char Exp1Add, char Exp2Add, char IdAdd, uint8_t verbose);
  void setVchannel(int channel, float voltage, uint8_t verbose);
  void testLED();
};
#endif
