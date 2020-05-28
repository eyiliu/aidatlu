#include "AidaTluController.hh"

#include <cstdio>
#include <csignal>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>

#include <unistd.h>



void InitTlu(tlu::AidaTluController* m_tlu){
  uint8_t m_verbose = 2;

  // Define constants
  m_tlu->DefineConst(4, 6);

  // Import I2C addresses for hardware
  // Populate address list for I2C elements
  m_tlu->SetI2C_core_addr( 0x21);
  m_tlu->SetI2C_clockChip_addr(0x68);
  m_tlu->SetI2C_DAC1_addr(0x13);
  m_tlu->SetI2C_DAC2_addr(0x1f);
  m_tlu->SetI2C_EEPROM_addr(0x50);
  m_tlu->SetI2C_expander1_addr(0x74);
  m_tlu->SetI2C_expander2_addr(0x75);
  m_tlu->SetI2C_pwrmdl_addr( 0x1C,  0x76, 0x77, 0x51);
  m_tlu->SetI2C_disp_addr(0x3A);

  // Initialize TLU hardware
  m_tlu->InitializeI2C(m_verbose);
  m_tlu->InitializeIOexp(m_verbose);

  m_tlu->InitializeDAC(false, 1.3, m_verbose);

  // Initialize the Si5345 clock chip using pre-generated file
  int clkres = m_tlu->InitializeClkChip( m_verbose  );

  // Reset IPBus registers
  m_tlu->ResetSerdes();
  m_tlu->ResetCounters();
  m_tlu->SetTriggerVeto(1, m_verbose);
  m_tlu->ResetFIFO();
  m_tlu->ResetEventsBuffer();
  m_tlu->ResetTimestamp();
}


void ConfTlu(tlu::AidaTluController* m_tlu){
  uint8_t m_verbose = 2;

  m_tlu->SetTriggerVeto(1, m_verbose);
  
  // Enable HDMI connectors
  // 0 CONT, 1 SPARE, 2 TRIG, 3 BUSY (1 = driven by TLU, 0 = driven by DUT)
  m_tlu->configureHDMI(1, 0b0111, m_verbose);
  m_tlu->configureHDMI(2, 0b0111, m_verbose);
  m_tlu->configureHDMI(3, 0b0111, m_verbose);
  m_tlu->configureHDMI(4, 0b0111, m_verbose);

  // Select clock to HDMI
  // 0 = DUT, 1 = Si5434, 2 = FPGA 
  m_tlu->SetDutClkSrc(1, 1, m_verbose);
  m_tlu->SetDutClkSrc(2, 1, m_verbose);
  m_tlu->SetDutClkSrc(3, 1, m_verbose);
  m_tlu->SetDutClkSrc(4, 1, m_verbose);

  //Set lemo clock
  m_tlu->enableClkLEMO(true, m_verbose);

  // Set thresholds
  m_tlu->SetThresholdValue(0, -0.03, m_verbose);
  m_tlu->SetThresholdValue(1, -0.03, m_verbose);
  m_tlu->SetThresholdValue(2, -0.03, m_verbose);
  m_tlu->SetThresholdValue(3, -0.03, m_verbose);
  m_tlu->SetThresholdValue(4, -0.03, m_verbose);
  m_tlu->SetThresholdValue(5, -0.03, m_verbose);

  // Set trigger stretch and delay
  m_tlu->SetPulseStretchPack({2,2,2,2,2,2}, m_verbose); //stretch the width of the signal; to get coincidence
  m_tlu->SetPulseDelayPack({0,0,0,0,0,0}, m_verbose);    //delay, e.g. compensate

  // Set triggerMask
  m_tlu->SetTriggerMask( 0x00000000,  0x00000000 ); // MaskHi, MaskLow, # trigMaskLo = 0x00000008 (pmt1 2),  0x00000002 (1)   0x00000004 (2) 

  // Set PMT power
  m_tlu->pwrled_setVoltages( 0.8, 0.8, 0.8, 0.8, m_verbose);

  m_tlu->SetDUTMask(0b0001, m_verbose); // Which DUTs are on
  m_tlu->SetDUTMaskMode(0b0101010101, m_verbose); // AIDA (x1) or EUDET (x0)
  m_tlu->SetDUTMaskModeModifier(0xff, m_verbose); // Only for EUDET
  m_tlu->SetDUTIgnoreBusy(0b0000, m_verbose); // Ignore busy in AIDA mode
  m_tlu->SetDUTIgnoreShutterVeto(1, m_verbose); //

  m_tlu->SetShutterParameters( false, 0, 0, 0, 0, 0, m_verbose);

  
  m_tlu->SetInternalTriggerFrequency(0, m_verbose );
  
  m_tlu->SetEnableRecordData( 1 );
  m_tlu->GetEventFifoCSR();
  m_tlu->GetEventFifoFillLevel();

}





int main(int /*argc*/, char ** argv) {
  std::unique_ptr<tlu::AidaTluController> tlu( new tlu::AidaTluController("chtcp-2.0://localhost:10203?target=192.168.200.30:50001"));
  InitTlu(tlu.get());
  ConfTlu(tlu.get());
  std::cout<< "sleeping"<<std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(10));
  std::cout<< "done"<<std::endl;
  return 0;
};
