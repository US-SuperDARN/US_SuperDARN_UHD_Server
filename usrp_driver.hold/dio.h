void kodiak_set_rxfe(uhd::usrp::multi_usrp::sptr usrp, struct RXFESettings rf_settings, int nSides);
void kodiak_init_rxfe(uhd::usrp::multi_usrp::sptr usrp, int nSides);

void mcm_set_rxfe(uhd::usrp::multi_usrp::sptr usrp, struct RXFESettings rf_settings);
void mcm_init_rxfe(uhd::usrp::multi_usrp::sptr usrp);

void send_timing_for_sequence(uhd::usrp::multi_usrp::sptr usrp, uhd::time_spec_t start_time, std::vector<uhd::time_spec_t> pulse_times, double pulseLength, bool mimic_active, float mimic_delay, int nSides);
void init_timing_signals( uhd::usrp::multi_usrp::sptr usrp, bool mimic_active, int nSides);
bool read_FAULT_status_from_control_board(  uhd::usrp::multi_usrp::sptr usrp, int iSide ); 

struct RXFESettings {
     uint32_t amp1;    /* Amp +20 db  */
     uint32_t amp2;    /* Amp +20 db */
     uint32_t att_05_dB;   /* 0.5 dB:*/
     uint32_t att_1_dB;   
     uint32_t att_2_dB;  
     uint32_t att_4_dB; 
     uint32_t att_8_dB;
     uint32_t att_16_dB;
};

