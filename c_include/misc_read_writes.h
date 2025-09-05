#ifndef MISC_READ_WRITES_H
#define MISC_READ_WRITES_H

#include <stdbool.h>
#include <fftw3.h>
#include "clear_freq_search.h"

// Function Prototypes

// Error handling
void file_access_error(const char *filepath);

// Pointer management
void add_ptr_no_global(void **ptr, void **temp_ptrs, int *temp_ptrs_num);
void update_ptr_no_global(void *old_ptr, void *new_ptr, void **temp_ptrs, int temp_ptrs_num);

// Spectrum writing
void write_spectrum_csv(char *filename, fftw_complex *spectrum, double *freq_vector, int num_samples);
void write_spectrum_mag_csv(char *filename, char *ststr, int channel, int beam_num, double *spectrum, double *freq_vector, int num_samples);
void write_spectrum_mag_bin(char *filename, char *ststr, int channel, int beam_num, double *spectrum, double *freq_vector, int num_samples);

// Clear frequency writing
void write_clr_freq_csv(char *filename, char *ststr, int channel, int beam_num, freq_band *clr_band, int *clr_range);
void write_clr_freq_bin(char *filename, char *ststr, int channel, int beam_num, freq_band *clr_band, int *clr_range);

// Sample writing
void write_sample_mag_csv(char *filename, int **raw_samples_mag, double *freq_vector, sample_meta_data *meta_data);

// Spectrum reading
void read_spectrum_mag_bin(char *filename, double *spectrum, double *freq_vector);

// Clear frequency reading
void read_clr_freq_bin(char *filename, freq_band *clr_band, int *clr_start, int *clr_end);

// Restriction reading
int read_restrict(char *filepath, freq_band *restricted_freq, int *restricted_num);

// Radar configuration reading
void read_radar_config(char *filepath, int *clr_freq_res);

// Timestamp and file utilities
void gen_filename(char *name_template, char *ext, char *name);
void gen_filename_to_hour(char *name_template, char *ext, char *name);
void get_timestamp(char *buffer);
void get_file_name(char *filename, char *filepath);
FILE* get_log_file(char *filepath);
FILE* init_log(int level, int file_level, char *filepath);

#endif // MISC_READ_WRITES_H
