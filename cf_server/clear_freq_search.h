#include <stdbool.h>
#include <fftw3.h>      // FFT transform library
#include "clear_frequency_server.h"
#include "ini_parser.h"

#ifndef RESTRICT_NUM
#define RESTRICT_NUM 50    // Number of restricted freq bands in the restrict.dat.inst
#endif

#define VERBOSE 1

typedef struct sample_meta_data {
    int antenna_list[30];
    int num_antennas;
    int number_of_samples;
    double x_spacing;
    int usrp_rf_rate;
    int usrp_fcenter;
    double if_rate;
} sample_meta_data;

typedef struct freq_band {
    int f_start;
    int f_end;
    double noise;
} freq_band;

typedef struct radar_freq_data {
    int clear_freq_range[2];
    freq_band clr_band;
    time_t last_time;
} radar_freq_data;


#pragma once

void clear_freq_search(fftw_complex *raw_samples, int *active_antennas,
                       int clear_freq_range[], int cur_beam, int smsep,
                       int avg_ratio, freq_band *restricted_bands,
                       int restrict_num, sample_meta_data meta_data,
                       Config config, freq_band prev_clr_band, freq_band *clr_band,
                       char *fft_file, char *clr_file,
                       char *ststr, int radar, int channel);

int ini_parse(const char* filename, ini_handler handler, void* user);

void process_avg_ant_pwr(fftw_complex *raw_samples, int num_samples,
                         sample_meta_data *meta_data, int *muted_config_ants,
                         int num_muted_config_ants, int *ant_active_ct,
                         int *active_antennas, long *acculated_pwrs);

void process_avg_beam_spectra(fftw_complex *beamformed_spectra, int avg_ratio,
                              int num_samples, int cur_beam, int beam_num,
                              int spectra_num, sample_meta_data *meta_data,
                              double **avg_beam_spectra, double *avg_freq_vector,
                              char *fft_file, char *ststr, int channel);

void process_all_beamformed_spectras(fftw_complex *raw_samples, int *active_antennas,
                                     int clear_freq_range[], int smsep,
                                     sample_meta_data *meta_data, Config config,
                                     fftw_complex *beamformed_spectra);

void process_beam_clr_freq(double **avg_beam_spectra, int cur_beam, int clear_freq_range[],
                           int smsep, freq_band *restricted_bands, int restricted_num,
                           double *avg_freq_vector, int num_avg_samples,
                           sample_meta_data *meta_data, freq_band prev_clr_band, freq_band *clr_band,
                           char *clr_file, char *ststr, int radar, int channel);


// Define Constants
#define GB_MULT 1.25                // Guard Band Multiplier (Transmission bandwidth * GB_MULT = clear_bw = clear_freq bandwidth)
#define MIN_FREQ_SEP 1500           // Minimum Frequency Separation (in Hz), guard band will be kept at and above this value.

#define RFIF_ATTEN 30               // Attenuation (in dB) applied by RF-IF filter to out of band signals

#define CLR_BAND_MULT 0.90          // Cutoff point relative to noise in previously selected band. Noise must be below cutoff for
                                    //   a new band to be selected; otherwise the previous clear band will be re-used (0 to disable)

#define MIN_ANT_PWR_MULT .1         // Cutoff point relative to overall average antenna power. Anything below cutoff will be muted.
#define MAX_ANT_PWR 25000           // Debug: used to flag high power samples during TCS's process_beamformed_samples

#define IDX_LAST_IA 19              // Last Interferrometer Array
#define IDX_LAST_MA 15              // Last Main Array
#define PI 3.14159265358979323846
#define C  299792458.0

// Config and Debug Flags
#define BIN_OR_CSV_LOG  0   // 0 for Bin, otherwise CSV

// Config Filepaths
#define SPECTRAL_LOG_FILE   "save_spectra"
#define SPECTRUM_FILE       "/data/log/fft_spectrum/%s.%s.%c.fft%s"
#define CLR_FREQ_FILE       "/data/log/clr_freq/%s.%s.%c.clr%s"
#define CLR_STOR_FILE       "/data/log/clr_freq/%s.%s.%c.clrlog.csv"
