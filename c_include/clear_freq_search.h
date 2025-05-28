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
void phasing_and_beamforming(double beam_angle, int *clear_freq_range, sample_meta_data *meta_data, fftw_complex *phasing_vector, int *antennas, int num_samples, fftw_complex *raw_samples, fftw_complex *beamformed_samples);

int ini_parse(const char* filename, ini_handler handler, void* user);
