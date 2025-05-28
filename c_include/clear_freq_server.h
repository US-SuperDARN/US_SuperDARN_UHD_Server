
#ifndef CLR_BANDS_MAX
#define CLR_BANDS_MAX 6
#endif


typedef struct sample_meta_data {
    int *antenna_list;
    int num_antennas;
    int number_of_samples;
    double x_spacing;
    int usrp_rf_rate;
    int usrp_fcenter;
} sample_meta_data;

typedef struct freq_data {
    double *restricted_freq;
    double *clear_freq_range;
} freq_data;

typedef struct freq_band {
    int f_start;
    int f_end;
    double noise;
    bool is_selected;
} freq_band;


typedef struct clear_freq {
    double noise;
    double tfreq;
} clear_freq;

void file_access_error(const char *filepath) {
    printf("[CFS] ERROR: accessing filepath: %s\n", filepath);
}

