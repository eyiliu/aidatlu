#include "AidaTluController.hh"

#include <cstdio>
#include <csignal>

#include <string>
#include <fstream>
#include <sstream>
#include <memory>
#include <chrono>
#include <thread>
#include <regex>

#include <unistd.h>
#include <getopt.h>

static const std::string help_usage = R"(
Usage:
  -help       help message
  -verbose    verbose flag

  -file       save file with trigger numbers and timestamps
  -quit       quit after configuration. TLU will keep running.

  -url [addr]
              example, using controlhub proxy
                -url chtcp-2.0://172.17.0.1:10203?target=192.168.200.30:50001
              IMPORTANT NOTE:
                   controlhub is required to run as deamon
                   controlhub startup command >  /opt/cactus/bin/controlhub_start
                   controlhub manual: https://ipbus.web.cern.ch/ipbus/doc/user/html/software/ControlHub.html

              example, using raw udp
                -url ipbusudp-2.0://192.168.200.30:50001

  -hz [n]     internal trigger frequency for test when exteral trigger(pmt) is not avalible

  -tmaskh [n] trigger mask high word
  -tmaskl [n] trigger mask low word
              example:
                -tmaskh 0x00000000 -tmaskl 0x00000008
                    Trigger = pmtA && pmtB && !pmtC && !pmtD && !pmtE && !pmtF
                -tmaskh 0x00000000 -tmaskl 0x00000002
                    Trigger = pmtA && !pmtB && !pmtC && !pmtD && !pmtE && !pmtF
                -tmaskh 0x00000000 -tmaskl 0x00000004
                    Trigger = !pmtA && pmtB && !pmtC && !pmtD && !pmtE && !pmtF

  -tA [n]     discriminator threshold for pmtA port, range[-1.3, 1.3] (input)
  -tB [n]     discriminator threshold for pmtB port, range[-1.3, 1.3] (input)
  -tC [n]     discriminator threshold for pmtC port, range[-1.3, 1.3] (input)
  -tD [n]     discriminator threshold for pmtD port, range[-1.3, 1.3] (input)
  -tE [n]     discriminator threshold for pmtE port, range[-1.3, 1.3] (input)
  -tF [n]     discriminator threshold for pmtF port, range[-1.3, 1.3] (input)

  -vA [n]     bias voltage pmtA port, range[0, 1] (output)
  -vB [n]     bias voltage pmtB port, range[0, 1] (output)
  -vC [n]     bias voltage pmtC port, range[0, 1] (output)
  -vD [n]     bias voltage pmtD port, range[0, 1] (output)

  -dA [eudet|aida|aida_id] [with_busy|without_busy]        modes for dutA port (IO)
  -dB [eudet|aida|aida_id] [with_busy|without_busy]        modes for dutB port (IO)
  -dC [eudet|aida|aida_id] [with_busy|without_busy]        modes for dutC port (IO)
  -dD [eudet|aida|aida_id] [with_busy|without_busy]        modes for dutD port (IO)

quick examples:
  aidatluTool -url ipbusudp-2.0://192.168.200.30:50001 -dA aida_id without_busy -hz 1000
  aidatluTool -url chtcp-2.0://localhost:10203?target=192.168.200.30:50001 -dA aida_id with_busy -dB eudet with_busy -dC aida without_busy -hz 1000
  aidatluTool -url chtcp-2.0://192.168.21.1:10203?target=192.168.200.30:50001 -vA 0.75 -vB 0.75 -tA -0.025 -tB -0.025 -hz 0 -tmaskl 0x00000008 -dA aida_id with_busy
)";

void OverwriteBits(uint16_t &dst, uint16_t src, int pos, int len) {
    uint16_t mask = (((uint16_t)1 << len) - 1) << pos;
    dst = ( dst & ~mask )|( (src<<pos) & mask );
}

static sig_atomic_t g_done = 0;
int main(int argc, char ** argv) {

  int do_quit = false;
  int do_help = false;
  uint32_t verbose_level = 0;

  struct option longopts[] =
    {
     { "help",     no_argument,       &do_help,      1  },
     { "verbose",  optional_argument, NULL,         'v' },
     { "file",     required_argument, NULL,         'f' },
     { "quit",     no_argument,       &do_quit,      1  },

     { "url",      required_argument, NULL,         'm' },
     { "tmaskh",   required_argument, NULL,         'b' },
     { "tmaskl",   required_argument, NULL,         'a' },
     { "hz",       required_argument, NULL,         'i' },

     { "tA",       required_argument, NULL,         'A' },
     { "tB",       required_argument, NULL,         'B' },
     { "tC",       required_argument, NULL,         'C' },
     { "tD",       required_argument, NULL,         'D' },
     { "tE",       required_argument, NULL,         'E' },
     { "tF",       required_argument, NULL,         'F' },

     { "vA",       required_argument, NULL,         'O' },
     { "vB",       required_argument, NULL,         'P' },
     { "vC",       required_argument, NULL,         'Q' },
     { "vD",       required_argument, NULL,         'R' },

     { "dA",       required_argument, NULL,         'H' },
     { "dB",       required_argument, NULL,         'I' },
     { "dC",       required_argument, NULL,         'J' },
     { "dD",       required_argument, NULL,         'K' },
     { 0, 0, 0, 0 }};

  uint32_t hz = 0;
  uint32_t tmaskh = 0;
  uint32_t tmaskl = 0;
  std::FILE* fp = 0;
  std::string sname;
  std::string url;
  uchar_t  ipbusDebug = 2;

  uint16_t dut_enable = 0b0000;
  uint16_t dut_mode = 0b00000000;
  uint16_t dut_modifier = 0b0000;
  uint16_t dut_nobusy = 0b0000;
  uint16_t dut_hdmi[4] = {0b0111, 0b0111, 0b0111, 0b0111}; // bit 0= CTRL, bit 1= SPARE/SYNC/T0/AIDA_ID, bit 2= TRIG/EUDET_ID, bit 3= BUSY
  uint16_t dut_clk[4] = {0b00, 0b00, 0b00, 0b00};

  float vthresh[6] = {0, 0, 0, 0, 0, 0};
  float vpmt[4] = {0, 0, 0, 0};

  int c;
  opterr = 1;
  while ((c = getopt_long_only(argc, argv, "", longopts, NULL))!= -1) {
    switch (c) {
    case 'v':{
      verbose_level = 2;
      if(optarg != NULL){
        verbose_level = static_cast<uint32_t>(std::stoull(optarg, 0, 10));
      }
      break;
    }
    case 'f':
        sname = optarg;
      break;
    case 'm':
      url = optarg;
      break;
    case 'i':
      hz = static_cast<uint32_t>(std::stoull(optarg, 0, 10));
      break;
    case 'b':
      if(std::regex_match(optarg, std::regex("\\s*(?:(0[Xx])?([0-9]+))\\s*")) ){
        std::cmatch mt;
        std::regex_match(optarg, mt, std::regex("\\s*(?:(0[Xx])?([0-9]+))\\s*"));
        tmaskh = std::stoull(mt[2].str(), 0, mt[1].str().empty()?10:16);
      }
      else{
        std::fprintf(stderr, "required argument must be Hexadecimal or Decimial number");
      }
      break;
    case 'a':
      if(std::regex_match(optarg, std::regex("\\s*(?:(0[Xx])?([0-9]+))\\s*")) ){
        std::cmatch mt;
        std::regex_match(optarg, mt, std::regex("\\s*(?:(0[Xx])?([0-9]+))\\s*"));
        tmaskl = std::stoull(mt[2].str(), 0, mt[1].str().empty()?10:16);
      }
      else{
        std::fprintf(stderr, "required argument must be Hexadecimal or Decimial number");
      }
      break;
    case 'A':
    case 'B':
    case 'C':
    case 'D':
    case 'E':
    case 'F':{
      uint16_t ch = c - 'A'; //First ch, A case
      try{
        vthresh[ch] = std::stof(optarg);
      }
      catch(...){
        std::fprintf(stderr, "error of stod\n");
        exit(1);
      }
      break;
    }
    case 'O':
    case 'P':
    case 'Q':
    case 'R':{
      uint16_t ch = c - 'O';
      try{
        vpmt[ch] = std::stof(optarg);
      }
      catch(...){
        std::fprintf(stderr, "error of stod\n");
        exit(1);
      }
      break;
    }
    case 'H':
    case 'I':
    case 'J':
    case 'K':{
      optind--;
      uint16_t ch = c - 'H'; //dut0, H case
      for( ;optind < argc && *argv[optind] != '-'; optind++){
        const char* mode = argv[optind];
        if(std::regex_match(mode, std::regex("\\s*(eudet)\\s*"))){
          OverwriteBits(dut_enable, 0b1, ch, 1);
          OverwriteBits(dut_mode, 0b00, ch*2, 2);
          OverwriteBits(dut_modifier, 0b1, ch*2, 1);
          dut_hdmi[ch] = 0b0111;
          dut_clk[ch] = 0b00;
        }
        else if(std::regex_match(mode, std::regex("\\s*(aida)\\s*"))){
          OverwriteBits(dut_enable, 0b1, ch, 1);
          OverwriteBits(dut_mode, 0b11, ch*2, 2);
          OverwriteBits(dut_modifier, 0b0, ch*2, 1);
          dut_hdmi[ch] = 0b0111;
          dut_clk[ch] = 0b01;
        }
        else if(std::regex_match(mode, std::regex("\\s*(aida_id)\\s*"))){
          OverwriteBits(dut_enable, 0b1, ch, 1);
          OverwriteBits(dut_mode, 0b01, ch*2, 2);
          OverwriteBits(dut_modifier, 0b0, ch*2, 1);
          dut_hdmi[ch] = 0b0111;
          dut_clk[ch] = 0b01;
        }
        else if(std::regex_match(mode, std::regex("\\s*(busy|enable_busy|with_busy)\\s*"))){
          OverwriteBits(dut_nobusy, 0b0, ch, 1);
        }
        else if(std::regex_match(mode, std::regex("\\s*(nobusy|ignore_busy|no_busy|disable_busy|without_busy)\\s*"))){
          OverwriteBits(dut_nobusy, 0b1, ch, 1);
        }
        else{
          std::fprintf(stderr, "dut%u unknow dut parmaters: %s", ch, mode);
          exit(1);
        }
      }
      break;
    }
      ////////////////
    case 0: /* getopt_long() set a variable, just keep going */
      break;
    case 1:
      std::fprintf(stderr,"case 1\n");
      exit(1);
      break;
    case ':':
      std::fprintf(stderr,"case :\n");
      exit(1);
      break;
    case '?':
      std::fprintf(stderr,"case ?\n");
      std::printf("%s\n", help_usage.c_str());
      exit(1);
      break;
    default:
      std::fprintf(stderr, "case default, missing branch in switch-case\n");
      exit(1);
      break;
    }
  }

  if(do_help){
    std::printf("%s\n", help_usage.c_str());
    exit(0);
  }

  signal(SIGINT, [](int){g_done+=1;});

  std::string url_default("chtcp-2.0://localhost:10203?target=192.168.200.30:50001");
  // std::string url_default("ipbusudp-2.0://192.168.200.30:50001");
  if(url.empty()){
    url=url_default;
    std::printf("using default tlu url location:   %s \n\n", url.c_str());
  }


  std::printf("\n--------------------------------------\n");
  std::printf("device_url:            %s\n", url.c_str());
  std::printf("file_data:             %s\n", sname.c_str());
  std::printf("verbose:               %lu\n", verbose_level);
  std::printf("interal_trigger:       %luHz\n", hz);
  std::printf("tmaskh:                %#010lx\n", tmaskh);
  std::printf("tmaskl:                %#010lx\n", tmaskl);
  std::printf("dut_enable:            %#010lx\n", dut_enable);
  std::printf("dut_modifier:          %#010lx\n", dut_modifier);
  std::printf("dut_nobusy:            %#010lx\n", dut_nobusy);
  std::printf("dut_hdmi[A,B,C,D]:     %#06lx   %#06lx   %#06lx   %#06lx\n", dut_hdmi[0], dut_hdmi[1], dut_hdmi[2], dut_hdmi[3]);
  std::printf("dut_clk[A,B,C,D]:      %#06lx   %#06lx   %#06lx   %#06lx\n", dut_clk[0], dut_clk[1], dut_clk[2], dut_clk[3]);
  std::printf("vpmt[A,B,C,D]:         %06f     %06f     %06f     %06f\n", vpmt[0], vpmt[1], vpmt[2], vpmt[3]);
  std::printf("vthresh[A,B,C,D,E,F]:  %06f     %06f     %06f     %06f     %06f     %06f\n", vthresh[0], vthresh[1], vthresh[2], vthresh[3], vthresh[4], vthresh[5]);
  std::printf("\n--------------------------------------\n");

  tlu::AidaTluController TLU(url);
  uint8_t verbose = verbose_level;
  // Define constants
  TLU.DefineConst(4, 6);

  // Import I2C addresses for hardware
  // Populate address list for I2C elements
  TLU.SetI2C_core_addr( 0x21);
  TLU.SetI2C_clockChip_addr(0x68);
  TLU.SetI2C_DAC1_addr(0x13);
  TLU.SetI2C_DAC2_addr(0x1f);
  TLU.SetI2C_EEPROM_addr(0x50);
  TLU.SetI2C_expander1_addr(0x74);
  TLU.SetI2C_expander2_addr(0x75);
  TLU.SetI2C_pwrmdl_addr( 0x1C,  0x76, 0x77, 0x51);
  TLU.SetI2C_disp_addr(0x3A);

  // Initialize TLU hardware
  TLU.InitializeI2C(verbose);
  TLU.InitializeIOexp(verbose);
  TLU.InitializeDAC(false, 1.3, verbose);
  // Initialize the Si5345 clock chip using pre-generated file
  TLU.InitializeClkChip( verbose  );

  // reset status
  TLU.SetTriggerVeto(1, verbose);
  TLU.SetRunActive(0, 1);
  TLU.SetEnableRecordData(0);
  TLU.SetInternalTriggerFrequency(0, verbose );
  TLU.SetTriggerMask( 0,  0);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  TLU.ResetSerdes();
  TLU.ResetCounters();
  TLU.ResetTimestamp();
  TLU.ResetFIFO();
  TLU.ResetEventsBuffer();

  //conf
  TLU.pwrled_setVoltages( vpmt[0], vpmt[1], vpmt[2], vpmt[3],verbose);
  TLU.SetThresholdValue(0, vthresh[0], verbose);
  TLU.SetThresholdValue(1, vthresh[1], verbose);
  TLU.SetThresholdValue(2, vthresh[2], verbose);
  TLU.SetThresholdValue(3, vthresh[3], verbose);
  TLU.SetThresholdValue(4, vthresh[4], verbose);
  TLU.SetThresholdValue(5, vthresh[5], verbose);

  TLU.SetTriggerMask( tmaskh,  tmaskl ); // MaskHi, MaskLow, # trigMaskLo = 0x00000008 (pmt1 2),  0x00000002 (1)   0x00000004 (2)

  TLU.SetDUTMask(dut_enable, verbose);
  TLU.SetDUTMaskMode(dut_mode, verbose);
  TLU.SetDUTMaskModeModifier(dut_modifier, verbose);
  TLU.SetDUTIgnoreBusy(dut_nobusy, verbose);

  TLU.configureHDMI(1, dut_hdmi[0], verbose);
  TLU.configureHDMI(2, dut_hdmi[1], verbose);
  TLU.configureHDMI(3, dut_hdmi[2], verbose);
  TLU.configureHDMI(4, dut_hdmi[3], verbose);

  // Select clock to HDMI
  // 0 = DUT, 1 = Si5434, 2 = FPGA
  TLU.SetDutClkSrc(1, dut_clk[0], verbose);
  TLU.SetDutClkSrc(2, dut_clk[1], verbose);
  TLU.SetDutClkSrc(3, dut_clk[2], verbose);
  TLU.SetDutClkSrc(4, dut_clk[3], verbose);

  TLU.SetDUTIgnoreShutterVeto(1, verbose);
  TLU.enableClkLEMO(true, verbose);

  TLU.SetShutterParameters( false, 0, 0, 0, 0, 0, verbose);
  TLU.SetInternalTriggerFrequency(hz, verbose );

  TLU.SetUhalLogLevel(verbose_level);
  std::printf("Hardware ID: %#012x\n  Firmware version: %d\n Url: %s \n", TLU.GetBoardID(), TLU.GetFirmwareVersion(), url.c_str() );

  if(do_quit){
    TLU.SetRunActive(1, 1);
    TLU.SetTriggerVeto(0, verbose);
    std::printf("Trigger generating.\n Quit.\n");
    return 0;
  }

  if(!sname.empty()){
    std::fopen(sname.c_str(), "w");
    if(!fp) {
      std::fprintf(stderr, "File opening failed: %s \n", sname.c_str());
      exit(1);
    }
  }

  TLU.SetEnableRecordData(1);
  TLU.SetRunActive(1, 1);
  TLU.SetTriggerVeto(0, verbose);

  std::printf("Enter RunLoop\n");
  uint32_t ev_total = 0;
  while (!g_done) {
    TLU.ReceiveEvents(verbose);
    while (!TLU.IsBufferEmpty()){
      ev_total++;
      tlu::fmctludata *data = TLU.PopFrontEvent();
      uint32_t evn = data->eventnumber; //trigger_n
      uint64_t t = data->timestamp;
      if(fp){
        //TODO: std::fwrite
      }
      delete data;
    }
  }
  std::printf("Quit RunLoop\n");
  TLU.SetTriggerVeto(1, verbose);
  TLU.SetRunActive(0, 1);
  TLU.SetEnableRecordData(0);
  std::printf("\nTrigger generating stops.\n");
  std::printf("\n\n");
  if(fp){
    std::fclose(fp);
  }
  return 0;
}
