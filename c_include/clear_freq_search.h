#pragma once
void phasing_and_beamforming(double beam_angle, int *clear_freq_range, sample_meta_data *meta_data, fftw_complex *phasing_vector, int *antennas, int num_samples, fftw_complex **raw_samples, int **sample_im, int **sample_re, fftw_complex *beamformed_samples);
