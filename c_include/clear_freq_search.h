#include <stdbool.h>
#include <fftw3.h>      // FFT transform library
#include "ini_parser.h"

#ifndef CLR_BANDS_MAX
#define CLR_BANDS_MAX 6
#endif
#ifndef RESTRICT_NUM
#define RESTRICT_NUM            50                  // Number of restricted freq bands in the restrict.dat.inst
#endif

#define VERBOSE 1

typedef struct sample_meta_data {
    int antenna_list[30];
    int num_antennas;
    int number_of_samples;
    double x_spacing;
    int usrp_rf_rate;
    int usrp_fcenter;
} sample_meta_data;

typedef struct freq_band {
    int f_start;
    int f_end;
    double noise;
    bool is_selected;
} freq_band;

typedef struct radar_freq_data {
    int clear_freq_range[2];
    freq_band clr_band;
} radar_freq_data;

typedef struct clear_freq {
    double noise;
    double tfreq;
} clear_freq;



typedef struct {
    int radar_stid;
    double x_spacing;
    int nradars;
    int nbeams;
    double beam_sep;
} ArrayInfo;

typedef struct {
    double max_tpulse;
    double min_chip;
    double max_dutycycle;
    double max_integration;
    double minimum_tfreq;
    double maximum_tfreq;
    double min_tr_to_pulse;
} HardwareLimits;

typedef struct {
    ArrayInfo array_info;
    HardwareLimits hardware_limits;
} Config;


#pragma once

int ini_parse(const char* filename, ini_handler handler, void* user);


// Define Constants
#define CLR_NOISE_THRESHOLD 100000  // Noise Threshold for a clear band to be considered a valid usable band
#define GB_MULT 4                   // Guard Band Multiplier (Transmission bandwidth * GB_MULT = clear_bw)
#define MIN_FREQ_SEP 6000           // Minimum Frequency Separation (in Hz) between Clear Freq Bands (If guard band + transmission bandwidth is less than this, 
                                    // then minimum frequency separation is used then ceiling-ed to be a factor of (# of avgs * freq per sample))

#define MIN_ANT_PWR 10              // Minimum Antenna Power to consider antenna as active

#define IDX_LAST_IA 19              // Last Interferrometer Array
#define IDX_LAST_MA 15              // Last Main Array
#define PI 3.14159265358979323846
#define C  3e8
#ifndef CLK_TCK
#define CLK_TCK 60
#endif


// Config and Debug Flags
#define BIN_OR_CSV_LOG  1   // 0 for Bin, otherwise CSV

#define TEST_SAMPLES 0
#define TEST_CLR_RANGE 1

// Config Filepaths
#define SPECTRAL_LOG_FILE   "save_spectra"
#define LOG_PATH            "/data/log/"
#define SPECTRUM_FILE       "/data/log/fft_spectrum/fft_spectrum.%s.%s"
#define CLR_FREQ_FILE       "/data/log/clr_freq/clr_freq.%s.%s"
#define SAMPLE_RE_FILE      "/data/log/sample_re.csv"
#define SAMPLE_IM_FILE      "/data/log/sample_im.csv"
