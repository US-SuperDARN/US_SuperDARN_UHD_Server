// Function uses GPIO of USRP to generate timing signals and control RXFE
//  On Kodiak, Outputs are:
//     
//  To control board
//     io_tx[00] 
//     io_tx[01] NOT_FAULT
//     io_tx[02]
//     io_tx[03]   TR_TX
//     io_tx[04]   TR_RX
//     io_tx[05]
//     io_tx[06]
//     Io_tx[07]
//      
//  Not connected
//     io_tx[08]   SYNC
//     io_tx[09]
//     io_tx[10]
//     io_tx[11]  MIMIC_TX
//     io_tx[12]  MIMIC_RX
//     io_tx[13]
//     io_tx[14]
//     io_tx[15]
//      
//  To RXFE
//     io_rx[00] ATT_D5    // -16  dB
//     io_rx[01] ATT_D4    // - 8  dB
//     io_rx[02] ATT_D3    // - 4  dB
//     io_rx[03] ATT_D2    // - 2  dB
//     io_rx[04] ATT_D1    // - 1  dB
//     io_rx[05] ATT_D0    // -0.5 dB
//     io_rx[06] AMP_2     // + 15 dB
//     io_rx[07] AMP_1     // + 15 dB
//
//   for newer style phasing cards, outputs are:
// 06 - MCM_SYNC
// 05 - MCM_TR
// 04 - AMP_A1
// 11 - AMP_A2
// 03 - AMP_B1
// 10 - AMP_B2
// 02 - AUX_IO_A
// 09 - AUX_IO_B
// 01 - ATT_A_LE
// 08 - ATT_B_LE
// 00 - ATT_SDI
// 07 - ATT_SCK
//     
// Functions to handle timing card/dio card on USRP
// broken out and expanded from Alex's recv_and_hold.cpp timing card code
// added rxfe dio and command issue time sorting
// JTK

#include <vector>
#include <queue>
#include <iostream>
#include <iomanip>

//Added by Alex for usrp
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/exception.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <thread>
#include <math.h>

#include "dio.h"
#include "usrp_utils.h"
 
// dio pin names to binary
#define IO_PIN_00 0x0001
#define IO_PIN_01 0x0002
#define IO_PIN_02 0x0004
#define IO_PIN_03 0x0008
#define IO_PIN_04 0x0010
#define IO_PIN_05 0x0020 
#define IO_PIN_06 0x0040
#define IO_PIN_07 0x0080

#define IO_PIN_08 0x0100
#define IO_PIN_09 0x0200
#define IO_PIN_10 0x0400
#define IO_PIN_11 0x0800
#define IO_PIN_12 0x1000
#define IO_PIN_13 0x2000
#define IO_PIN_14 0x4000
#define IO_PIN_15 0x8000 



#define MANUAL_CONTROL 0x00


// TIMING: LFTX to control board (agc) 
// mapping timing TX RX signals pins
#define TR_TX     IO_PIN_05
#define TR_RX     IO_PIN_09
#define TR_PINS   (TR_TX +TR_RX) 

#define NOT_FAULT_PIN  IO_PIN_01

// SYNC
//#define SYNC_PINS IO_PIN_08, for kodiak
#define SYNC_PINS IO_PIN_06

#define SYNC_OFFSET_START (-500e-6) // start of sync pulse
#define SYNC_OFFSET_END   (-400e-6) // start of sync pulse
#define SYNC_FOR_EACH_SEQUENCE 1

//  MIMIC (LFTX)
#define MIMIC_TX    IO_PIN_11
#define MIMIC_RX    IO_PIN_12
#define MIMIC_PINS  (MIMIC_TX + MIMIC_RX)


// RXFE CONTROL: LFRX to rxfe board
// map rxfe pins to USRP dio pins
#define AMP_1  IO_PIN_07   // + 15 dB
#define AMP_2  IO_PIN_06   // + 15 dB
#define ATT_D0 IO_PIN_05   // -0.5 dB
#define ATT_D1 IO_PIN_04   // - 1  dB
#define ATT_D2 IO_PIN_03   // - 2  dB
#define ATT_D3 IO_PIN_02   // - 4  dB
#define ATT_D4 IO_PIN_01   // - 8  dB
#define ATT_D5 IO_PIN_00   // -16  dB

// define rxfe masks
#define RXFE_AMP_MASK  (AMP_1 + AMP_2)
#define RXFE_ATT_MASK  (ATT_D0 + ATT_D1 + ATT_D2 + ATT_D3 + ATT_D4 + ATT_D5)
#define RXFE_MASK      (RXFE_ATT_MASK + RXFE_AMP_MASK)

#define MCM_SYNC            IO_PIN_06
#define MCM_TR              IO_PIN_05
#define MCM_AMP_A1          IO_PIN_04
#define MCM_AMP_A2          IO_PIN_11
#define MCM_AMP_B1          IO_PIN_03
#define MCM_AMP_B2          IO_PIN_10
#define MCM_AUX_IO_A        IO_PIN_02
#define MCM_AUX_IO_B        IO_PIN_09
#define MCM_ATT_A_LE    IO_PIN_01
#define MCM_ATT_B_LE    IO_PIN_08
#define MCM_ATT_SDI     IO_PIN_00
#define MCM_ATT_SCK     IO_PIN_07

#define MCM_RXFE_MASK  (MCM_SYNC + MCM_TR + MCM_AMP_A1 + MCM_AMP_A2 + MCM_AMP_B1 + MCM_AMP_B2 + MCM_AUX_IO_A + MCM_AUX_IO_B + MCM_ATT_A_LE + MCM_ATT_B_LE + MCM_ATT_SDI + MCM_ATT_SCK)
#define DEBUG 1
#ifdef DEBUG
#define DEBUG_PRINT(...) do{ fprintf( stderr, __VA_ARGS__ ); } while( false )
#else
#define DEBUG_PRINT(...) do{ } while ( false )
#endif


#define MCM_STYLE 1
#define TX_PORT "FP%c"
// for kodiak transmitters, this would be "TX%c"


void init_timing_signals(
    uhd::usrp::multi_usrp::sptr usrp,
    bool mimic_active,
    int nSides
) {
    char bank_name[4];
    int iSide;

    double debugt = usrp->get_time_now().get_real_secs();
    DEBUG_PRINT("DIO queing GPIO commands at usrp_time %2.4f\n", debugt);
    
    if (MCM_STYLE) {
        usrp->set_gpio_attr("FP0","CTRL",MANUAL_CONTROL, MCM_SYNC);
        usrp->set_gpio_attr("FP0","CTRL",MANUAL_CONTROL, MCM_TR);

        usrp->set_gpio_attr("FP0","CTRL",MANUAL_CONTROL, MCM_AUX_IO_A);
        usrp->set_gpio_attr("FP0","CTRL",MANUAL_CONTROL, MCM_AUX_IO_B);

        usrp->set_gpio_attr("FP0","DDR", MCM_SYNC, MCM_SYNC);
        usrp->set_gpio_attr("FP0","DDR", MCM_TR,   MCM_TR);
    }

    else {
        // setup gpio to manual control and direction (output)
        for (iSide=0; iSide<nSides; iSide++) {
           sprintf(bank_name, "TX%c",65 + iSide);
           fprintf( stderr, "Bank name: %s\n",bank_name ); 
           
           usrp->set_gpio_attr(bank_name,"CTRL",MANUAL_CONTROL, SYNC_PINS);
           usrp->set_gpio_attr(bank_name,"CTRL",MANUAL_CONTROL, TR_PINS);
           usrp->set_gpio_attr(bank_name,"CTRL",MANUAL_CONTROL, NOT_FAULT_PIN);

           usrp->set_gpio_attr(bank_name,"DDR" ,SYNC_PINS, SYNC_PINS);
           usrp->set_gpio_attr(bank_name,"DDR" ,TR_PINS,   TR_PINS);
           usrp->set_gpio_attr(bank_name,"DDR" ,      0,   NOT_FAULT_PIN); // NOT_FAULT as input
        }

       if (mimic_active) {
           usrp->set_gpio_attr("TXA","CTRL",MANUAL_CONTROL,MIMIC_PINS);
           usrp->set_gpio_attr("TXA","DDR" ,MIMIC_PINS, MIMIC_PINS);
           DEBUG_PRINT("DIO.cpp: init MIMIC target\n");
       }
    }


    debugt = usrp->get_time_now().get_real_secs();
    DEBUG_PRINT("DIO: set gpio attrs at usrp_time %2.4f\n", debugt);

}

bool read_FAULT_status_from_control_board(
    uhd::usrp::multi_usrp::sptr usrp,
    int iSide
) {
    if(MCM_STYLE) {
        // mcmurdo transmitters do not relay a fault signal
        return 0;
    }

    char bank_name[4];
    sprintf(bank_name, "TX%c",65 + iSide);
    uint32_t input = usrp->get_gpio_attr(bank_name, "READBACK");
    DEBUG_PRINT("Readback from control board: %d\n", input);
    bool notFAULT = (input & NOT_FAULT_PIN) != 0;
    DEBUG_PRINT("Returning FAULT = %d\n", !notFAULT);
    return !notFAULT;
}



struct GPIOCommand {
    uhd::time_spec_t cmd_time;
    std::string port;
    std::string gpiocmd;
    uint32_t value;
    uint32_t mask;
};


class CompareTime {
public:
    bool operator()(GPIOCommand& t1, GPIOCommand& t2)
    {  
       return (t1.cmd_time > t2.cmd_time);
    }
};


// CREATE QUEUE WITH ALL SIGNAL CHANGES
void send_timing_for_sequence(
    uhd::usrp::multi_usrp::sptr usrp,
    uhd::time_spec_t start_time,
    std::vector<uhd::time_spec_t> pulse_times,
    double pulseLength,
    bool mimic_active,
    float mimic_delay,
    int nSides
) {

    char bank_name[4];
    sprintf(bank_name, "FP0");
    int iSide;


    GPIOCommand c; // struct to hold command information so gpio commands can be created out of temporal order, sorted, and issued in order
    std::priority_queue<GPIOCommand, std::vector<GPIOCommand>, CompareTime> cmdq;
    
    // set SYNC to high
    c.gpiocmd  = "OUT";
    c.mask     = SYNC_PINS;
    c.value    = SYNC_PINS;
    c.cmd_time = offset_time_spec(start_time, SYNC_OFFSET_START);
    c.port     = bank_name;
    cmdq.push(c);

    // lower SYNC pin
    c.value = 0;
    c.cmd_time = offset_time_spec(start_time, SYNC_OFFSET_END);
    c.port     = bank_name;
    cmdq.push(c);
    
    //debug
    fprintf( stderr, "DIO setting start of SYNC to %2.11f\n", offset_time_spec(start_time, SYNC_OFFSET_START).get_real_secs() ); 
    fprintf( stderr, "DIO setting end   of SYNC to %2.11f\n", offset_time_spec(start_time, SYNC_OFFSET_END).get_real_secs() ); 


    // set TX and RX line for each pulse
    for(size_t iPulse = 0; iPulse < pulse_times.size() ; iPulse++) {
        // set TX high, RX low    
        c.mask     = TR_PINS;
        c.value    = TR_TX;   
        c.cmd_time = pulse_times[iPulse];
        c.port     = bank_name;
        cmdq.push(c);

        // set RX high and TX low
        c.value    = TR_RX;
        c.cmd_time = offset_time_spec(pulse_times[iPulse], pulseLength);
        c.port     = bank_name;
        cmdq.push(c);

        if (mimic_active) {
           // DEBUG_PRINT("DIO.cpp: using mimic target with %2.4f ms delay\n", mimic_delay*1000);
            // set mimic TX high, mimic RX low   
            c.port     = "TXA";
            c.mask     = MIMIC_PINS;
            c.value    = MIMIC_TX;   
            c.cmd_time = offset_time_spec(pulse_times[iPulse], mimic_delay);
            cmdq.push(c);

            // set mimic RX high and mimic TX low
            c.value    = MIMIC_RX;
            c.cmd_time = offset_time_spec(pulse_times[iPulse], mimic_delay + pulseLength);
            cmdq.push(c);

        }

    }


    float debugt = usrp->get_time_now().get_real_secs();


    boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
    std::cout << "GPIO command issue start time is: "<< to_iso_extended_string(now)<< std::endl;


    DEBUG_PRINT("DIO: pushed gpio commands at usrp_time %2.4f\n", debugt);
    // issue gpio commands in time sorted order 
    // set_command_time must be sent in temporal order, they are sent into a fifo queue on the usrp and will block until executed
    // clear_command_time does not clear or bypass the buffer as of 10/2014..
    while (!cmdq.empty()) {
        c = cmdq.top();
        usrp->set_command_time(c.cmd_time);
        usrp->set_gpio_attr(c.port,c.gpiocmd,c.value,c.mask);

        //debugt = usrp->get_time_now().get_real_secs();
        //DEBUG_PRINT("DIO: sending queue: val:%2.6u mask: %2.6u at usrp_time %2.6f (%2.4f) \n", c.value, c.mask,  c.cmd_time.get_real_secs(), debugt);

        cmdq.pop();
    }


    now = boost::posix_time::microsec_clock::universal_time();
    std::cout << "GPIO command issue end time is: "<< to_iso_extended_string(now)<< std::endl;

    debugt = usrp->get_time_now().get_real_secs();
    DEBUG_PRINT("All DIO commands sent! clock time %2.4f\n",debugt);

    usrp->clear_command_time();




    /*
    // commands to the USRP are now blocked until the pulse sequence is completed
    // usrp_set_command_time will delay any future commands until the command queue is empty (once the pulse sequence including any diagnostic targets is completed)
    
    // the maximum size of the queue is 64
    // a dual polarization radar with timing on separate daughtercards and a 7 pulse sequence plus sync and two mimic targets
    // would have 2 * 2 * 2 * 7 + 2 = 60 entries in the buffer..
    // for more complicated pulse sequences, we may need to fill the buffer partway through the pulse sequence.. 
    //

    */
}


void kodiak_set_rxfe(
    uhd::usrp::multi_usrp::sptr usrp,
    struct RXFESettings rf_settings,
    int nSides
) {
    // load up rxfe_dio with rf_setting struct
    uint16_t rxfe_dio = 0; 
    rxfe_dio |= AMP_1  * (rf_settings.amp1      != 0);
    rxfe_dio |= AMP_2  * (rf_settings.amp2      != 0); 
    rxfe_dio |= ATT_D0 * (rf_settings.att_05_dB != 0);
    rxfe_dio |= ATT_D1 * (rf_settings.att_1_dB  != 0);
    rxfe_dio |= ATT_D2 * (rf_settings.att_2_dB  != 0);
    rxfe_dio |= ATT_D3 * (rf_settings.att_4_dB  != 0);
    rxfe_dio |= ATT_D4 * (rf_settings.att_8_dB  != 0);
    rxfe_dio |= ATT_D5 * (rf_settings.att_16_dB != 0);

    rxfe_dio = (~rxfe_dio) & 0xFF; // invert all, since high means amp  and att off


    DEBUG_PRINT("DIO.cpp: received rf_settings: \n   amp1: %d\n   amp2: %d\n   att 0.5 dB: %d\n   att 1 dB  : %d\n   att 2 dB  : %d\n   att 4 dB  : %d\n   att 8 dB  : %d\n   att 16 dB : %d\n", rf_settings.amp1, rf_settings.amp2, rf_settings.att_05_dB, rf_settings.att_1_dB, rf_settings.att_2_dB, rf_settings.att_4_dB, rf_settings.att_8_dB, rf_settings.att_16_dB );
    DEBUG_PRINT("DIO.cpp: sending rxfe_dio:  %d\n", rxfe_dio);

    usrp->set_gpio_attr("RXA", "OUT", rxfe_dio, RXFE_MASK);
    if (nSides ==2) {
        usrp->set_gpio_attr("RXB", "OUT", rxfe_dio, RXFE_MASK);
    }

}


void kodiak_init_rxfe(uhd::usrp::multi_usrp::sptr usrp, int nSides)
{
    char bank_name[4];

    for (int iSide=0; iSide<nSides; iSide++) {

         sprintf(bank_name, "RX%c",65 + iSide);
         // hardcode RXFE settings...
         usrp->set_gpio_attr(bank_name, "CTRL", MANUAL_CONTROL, RXFE_MASK); // set GPIO to manual control
         usrp->set_gpio_attr(bank_name, "DDR", RXFE_MASK, RXFE_MASK); // set everybody as outputs

         // start up with rxfe disabled, rx mode
         usrp->set_gpio_attr(bank_name, "OUT", 0xFF, RXFE_AMP_MASK);
         usrp->set_gpio_attr(bank_name, "OUT", 0xFF, RXFE_ATT_MASK);

         // set to 1/2 full attenuation, both amps online 
         usrp->set_gpio_attr(bank_name, "OUT", (255 - (AMP_1 + AMP_2 + ATT_D5)), RXFE_MASK);
    }
}


void mcm_init_rxfe(uhd::usrp::multi_usrp::sptr usrp)
{
    // use set FP0 (front panel dsub15) as outputs with manual control
    usrp->set_gpio_attr("FP0", "CTRL", MANUAL_CONTROL, MCM_RXFE_MASK);
    usrp->set_gpio_attr("FP0", "DDR", MCM_RXFE_MASK, MCM_RXFE_MASK);

    // start with 1/2 full attenuation, both amps offline
    struct RXFESettings rxfe_init_settings;

    // initialize LE, SCK, SDI, and TR into safe states
    usrp->set_gpio_attr("FP0", "OUT", MCM_ATT_A_LE, MCM_ATT_A_LE);
    usrp->set_gpio_attr("FP0", "OUT", MCM_ATT_B_LE, MCM_ATT_B_LE);
    usrp->set_gpio_attr("FP0", "OUT", 0, MCM_ATT_SDI);
    usrp->set_gpio_attr("FP0", "OUT", 0, MCM_ATT_SCK);

    usrp->set_gpio_attr("FP0", "OUT", 0, MCM_TR);
    
    // initialize rxfe with no amplifiers or added attenuation
    rxfe_init_settings.amp1 = 0;
    rxfe_init_settings.amp2 = 0;
    
    rxfe_init_settings.att_05_dB = 0;
    rxfe_init_settings.att_1_dB = 0;
    rxfe_init_settings.att_2_dB = 0;
    rxfe_init_settings.att_4_dB = 0;
    rxfe_init_settings.att_8_dB = 0;
    rxfe_init_settings.att_8_dB = 0;
    
    mcm_set_rxfe(usrp, rxfe_init_settings); 
}

void mcm_set_rxfe(
    uhd::usrp::multi_usrp::sptr usrp,
    struct RXFESettings rf_settings)
{
    uint16_t rxfe_dio = 0;
    uint16_t rxfe_dio_mask = MCM_AMP_A1 + MCM_AMP_B1 + MCM_AMP_A2 + MCM_AMP_B2;

    if (rf_settings.amp1) {
        rxfe_dio |= MCM_AMP_A1;
        rxfe_dio |= MCM_AMP_B1;
    }

    if (rf_settings.amp2) {
        rxfe_dio |= MCM_AMP_A2;
        rxfe_dio |= MCM_AMP_B2;
    }
    

    uint8_t att_payload = rf_settings.att_05_dB | \
                          rf_settings.att_1_dB << 1 | \
                          rf_settings.att_2_dB << 2 | \
                          rf_settings.att_4_dB << 3 | \
                          rf_settings.att_8_dB << 4 | \
                          rf_settings.att_16_dB << 5;

    // bring both LE to enable
    // bang out rxfe settings
    // return LE
    usrp->set_gpio_attr("FP0", "OUT", 0, MCM_ATT_A_LE);
    usrp->set_gpio_attr("FP0", "OUT", 0, MCM_ATT_B_LE);
   
    // shift attenuator settings into PE4312 shift register
    // MSB first, 6 bits
    // max SPI clock frequency is 10 MHz
    // TODO: check if delay is needed in bit banging
    for(int i = 5; i >= 0; i--) {
        usrp->set_gpio_attr("FP0", "OUT", 0, MCM_ATT_SCK);

        if(att_payload & (1 << i)) {
            usrp->set_gpio_attr("FP0", "OUT", MCM_ATT_SDI, MCM_ATT_SDI);
        }
        else {
            usrp->set_gpio_attr("FP0", "OUT", 0, MCM_ATT_SDI);
        }

        usrp->set_gpio_attr("FP0", "OUT", MCM_ATT_SCK, MCM_ATT_SCK);

    }

    usrp->set_gpio_attr("FP0", "OUT", MCM_ATT_A_LE, MCM_ATT_A_LE);
    usrp->set_gpio_attr("FP0", "OUT", MCM_ATT_B_LE, MCM_ATT_B_LE);

    // TODO: check if delay is needed, LE pulse must be at least 30 nanoseconds
    usrp->set_gpio_attr("FP0", "OUT", 0, MCM_ATT_A_LE);
    usrp->set_gpio_attr("FP0", "OUT", 0, MCM_ATT_B_LE);

    usrp->set_gpio_attr("FP0", "OUT", rxfe_dio, rxfe_dio_mask);


    
    

}

