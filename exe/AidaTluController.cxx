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

  -quit       after configuring TLU
  -update     output message interval in millisecond
  -pause      for user input before starting triggers
  -save       file with trigger numbers and timestamps

  -url [addr] 
              example, using controlhub
                IMPORTANT start controlhub:    /opt/cactus/bin/controlhub_start
                -url chtcp-2.0://localhost:10203?target=192.168.200.30:50001

              example, using raw udp  
                -url ipbusudp-2.0://192.168.200.30:50001

  -hz [n]     internal trigger frequency
  -tmaskh [n] trigger mask low word
  -tmaskl [n] trigger mask high word, trigMaskLo = 0x00000008 (pmtA B),  0x00000002 (A)   0x00000004 (B)

  -tA [n]     discriminator threshold for pmtA port (input)
  -tB [n]     discriminator threshold for pmtB port (input)
  -tC [n]     discriminator threshold for pmtC port (input)
  -tD [n]     discriminator threshold for pmtD port (input)
  -tE [n]     discriminator threshold for pmtE port (input)
  -tF [n]     discriminator threshold for pmtF port (input)

  -vA [n]     voltage pmtA port (output)
  -vB [n]     voltage pmtB port (output)
  -vC [n]     voltage pmtC port (output)
  -vD [n]     voltage pmtD port (outpur)
 
  -dA [eudet|aida|aida_id] [with_busy|without_busy]        modes for dutA port (IO)
  -dB [eudet|aida|aida_id] [with_busy|without_busy]        modes for dutB port (IO)
  -dC [eudet|aida|aida_id] [with_busy|without_busy]        modes for dutC port (IO)
  -dD [eudet|aida|aida_id] [with_busy|without_busy]        modes for dutD port (IO)

quick examples:
  mytlu_ctrl -url ipbusudp-2.0://192.168.200.30:50001 -dA aida_id without_busy -hz 1000 
  mytlu_ctrl -url chtcp-2.0://localhost:10203?target=192.168.200.30:50001 -dA aida_id with_busy -dB eudet with_busy -dC aida without_busy -hz 1000

)";

void OverwriteBits(uint16_t &dst, uint16_t src, int pos, int len) {
    uint16_t mask = (((uint16_t)1 << len) - 1) << pos;
    dst = (dst & ~mask) | (src & mask);
}

static sig_atomic_t g_done = 0;
int main(int argc, char ** argv) {

  int do_quit = false;
  int do_pause = false;
  int do_help = false;
  int do_verbose = false;

  struct option longopts[] =
    {
     { "help",       no_argument,       &do_help,      1  },
     { "verbose",    no_argument,       &do_verbose,   1  },

     { "quit",       no_argument,       &do_quit,      1  },
     { "pause",      no_argument,       &do_pause,     1  },
     { "save",       optional_argument, NULL,         's' },
     { "update",     required_argument, NULL,         'u' },
   
     { "url",        required_argument, NULL,         'm' },
     { "tmaskh",     required_argument, NULL,         'v' },
     { "tmaskl",     required_argument, NULL,         'a' },
     { "hz",         required_argument, NULL,         'i' },
     
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
  uchar_t tmaskh = 0;
  uchar_t tmaskl = 0;
  
  uint32_t update = 1000;
  std::string sname;
  std::string url;
  uchar_t  ipbusDebug = 2;  

  uint16_t dut_enable = 0b0000;
  uint16_t dut_mode = 0b00000000;
  uint16_t dut_modifier = 0b0000;
  uint16_t dut_nobusy = 0b0000;
  //uint16_t dut_noveto =
  uint16_t dut_hdmi[4] = {0b0111, 0b0111, 0b0111, 0b0111}; // bit 0= CTRL, bit 1= SPARE/SYNC/T0/AIDA_ID, bit 2= TRIG/EUDET_ID, bit 3= BUSY
  uint16_t dut_clk[4] = {0b00, 0b00, 0b00, 0b00};

  float vthresh[6] = {0, 0, 0, 0, 0, 0};
  float vpmt[6] = {0, 0, 0, 0, 0, 0};
  
  int c;
  opterr = 1;
  while ((c = getopt_long_only(argc, argv, "", longopts, NULL))!= -1) {
    switch (c) {
    case 'h':
      do_help = 1;
      break;
    case 'q':
      do_quit = 1;
      break;
    case 'p':
      do_pause = 1;
      break;
    case 'u':
      update  = static_cast<uint32_t>(std::stoull(optarg, 0, 10));
      break;
    case 'm':
      url = optarg;
      break;
    case 's':
      if(optarg != NULL)
        sname = optarg;
      else
        sname = "data.txt";
      break;
    case 'i':
      hz = static_cast<uint32_t>(std::stoull(optarg, 0, 10));
      break;
    case 'v':
      tmaskh = static_cast<uchar_t>(std::stoull(optarg, 0, 16));
      break;
    case 'a':
      tmaskl = static_cast<uchar_t>(std::stoull(optarg, 0, 16));
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
        fprintf(stderr, "error of stod\n");
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
        fprintf(stderr, "error of stod\n");
        exit(1);
      }
      break;
    }
    case 'H':
    case 'I':
    case 'J':
    case 'K':{
      fprintf(stdout, "case %c\n", c);
      optind--;
      uint16_t ch = c - 'H'; //dut0, H case
      for( ;optind < argc && *argv[optind] != '-'; optind++){
        std::cout<<argv[optind]<<std::endl;
        const char* mode = argv[optind];
        if(std::regex_match(mode, std::regex("\\s*(eudet)\\s*"))){
          OverwriteBits(dut_enable, 0b1, ch, 1);
          OverwriteBits(dut_mode, 0b00, ch*2, 2);
          OverwriteBits(dut_modifier, 0b1, ch, 1);
          dut_hdmi[ch] = 0b0111;
          dut_clk[ch] = 0b00;
        }
        else if(std::regex_match(mode, std::regex("\\s*(aida)\\s*"))){
          OverwriteBits(dut_enable, 0b1, ch, 1);
          OverwriteBits(dut_mode, 0b11, ch*2, 2);
          OverwriteBits(dut_modifier, 0b0, ch, 1);
          dut_hdmi[ch] = 0b0111;
          dut_clk[ch] = 0b01;
        }
        else if(std::regex_match(mode, std::regex("\\s*(aida_id)\\s*"))){
          OverwriteBits(dut_enable, 0b1, ch, 1);
          OverwriteBits(dut_mode, 0b01, ch*2, 2);
          OverwriteBits(dut_modifier, 0b0, ch, 1);
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
          fprintf(stderr, "dut%u unknow dut parmaters: %s", ch, mode);
          exit(1);
        }
      }
      break;
    }
      ////////////////
    case 0: /* getopt_long() set a variable, just keep going */
      break;
    case 1:
      fprintf(stderr,"case 1\n");
      exit(1);
      break;
    case ':':
      fprintf(stderr,"case :\n");
      exit(1);
      break;
    case '?':
      fprintf(stderr,"case ?\n");
      exit(1);
      break;
    default:
      fprintf(stderr, "case default, missing branch in switch-case\n");
      exit(1);
      break;
    }
  }

  if(do_help){
    std::cout<<help_usage<<std::endl;
    exit(0);
  }
  //return 0;
  
  std::shared_ptr<std::ofstream> sfile;
  if (sname != "") {
    sfile = std::make_shared<std::ofstream>(sname.c_str());
    if (!sfile->is_open()) {
      std::cerr<<"Unable to open file: " << sname;
      return 1;
    }
  }
  signal(SIGINT, [](int){g_done+=1;});
  
  std::string url_default("chtcp-2.0://localhost:10203?target=192.168.200.30:50001");
  // std::string url_default("ipbusudp-2.0://192.168.200.30:50001");
  if(url.empty()){
    url=url_default;
    std::printf("using default tlu url location:   %s", url.c_str());
  }
  std::printf("open tlu at url location:   %s", url.c_str());
  tlu::AidaTluController TLU(url);
  uint8_t verbose = 2;
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

  TLU.ResetSerdes();
  TLU.ResetCounters();
  TLU.SetTriggerVeto(1, verbose);
  TLU.ResetFIFO();
  TLU.ResetEventsBuffer();
  TLU.ResetTimestamp();

  //conf
  TLU.pwrled_setVoltages( vpmt[0], vpmt[1], vpmt[2], vpmt[3],verbose);  
  TLU.SetThresholdValue(0, vthresh[0], verbose);
  TLU.SetThresholdValue(1, vthresh[1], verbose);
  TLU.SetThresholdValue(2, vthresh[2], verbose);
  TLU.SetThresholdValue(3, vthresh[3], verbose);
  TLU.SetThresholdValue(4, vthresh[4], verbose);
  TLU.SetThresholdValue(5, vthresh[5], verbose);

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

  TLU.SetEnableRecordData(1);

  TLU.SetTriggerMask( tmaskh,  tmaskl ); // MaskHi, MaskLow, # trigMaskLo = 0x00000008 (pmt1 2),  0x00000002 (1)   0x00000004 (2)   
  TLU.SetUhalLogLevel(ipbusDebug);

  uint32_t bid = TLU.GetBoardID();
  uint32_t fid = TLU.GetFirmwareVersion();
  std::printf("Board ID number  = %#012x\n ", bid);
  std::printf("Firmware version  = %d\n ", fid);
  
  if (do_quit) return 0;

  if (do_pause) {
    std::printf("Press enter to start triggers.\n");
    std::getchar();
  }
  TLU.SetTriggerVeto(0, verbose);
  std::printf( "FMC TLU Started!\n");
  
  uint32_t total = 0;
  while (!g_done) {
    TLU.ReceiveEvents(verbose);
    int nev = 0;
    while (!TLU.IsBufferEmpty()){
      nev++;
      tlu::fmctludata * data = TLU.PopFrontEvent();
      uint32_t evn = data->eventnumber;
      uint64_t t = data->timestamp;

      if (sfile.get()) {
        *sfile << *data;
      }
    }
    total += nev;
    if (update > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(update));
    }
  }
  std::printf("Quitting...\n\n");
  TLU.SetTriggerVeto(1, verbose);
  return 0;
}
