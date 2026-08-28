char *get_log_time();
char *format_usrp_time(uhd::time_spec_t usrp_time);
uhd::time_spec_t offset_time_spec(uhd::time_spec_t t0, double toffset);
