#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <complex.h>
#include <fftw3.h>      // FFT transform library
#include <string.h>
#include <time.h>
#include <signal.h>
#include "clear_freq_search.h"
#include "log.h"


// Define Constants
#define IDX_LAST_IA 19      // Last Interferrometer Array
#define IDX_LAST_MA 15      // Last Main Array
#define PI 3.14159265358979323846
#define C  3e8
#define CLR_BANDS_MAX 6
#define CLRFREQ_RES 2e3          
#define GB_MULT 1.5         // Guard Band Multiplier (Transmission bandwidth * GB_MULT = clear_bw)
#ifndef CLK_TCK
#define CLK_TCK 60
#endif


// Config and Debug Flags
#define SPECTRAL_AVGING 1
#define BIN_OR_CSV_LOG  0   // 0 for Bin, otherwise CSV

#define TEST_SAMPLES 0
#define TEST_CLR_RANGE 1

// Config Filepaths
#define SPECTRAL_LOG_FILE   "save_spectra"
#define LOG_PATH            "log/"
#define SPECTRUM_FILE       "log/fft_spectrum/fft_spectrum.%s.%s"
#define CLR_FREQ_FILE       "log/clr_freq/clr_freq.%s.%s"
#define SAMPLE_RE_FILE      "log/sample_re.csv"
#define SAMPLE_IM_FILE      "log/sample_im.csv"


// TODO: Pass in clr_freq_range via restrict actual file
// #define RESTRICT_FILE = '/home/radar/repos/SuperDARN_MSI_ROS/linux/home/radar/ros.3.6/tables/superdarn/site/site.sps/restrict.dat.inst'

// TODO: Pass in x_spacing, etc. via config actual file

// TODO: Store Clear Freq Bands for each Beam 

/*
* NOTE: 
* When compiling make sure to add "-lm" and "-lfftw3" to link the libraries 
* to the compilier. 
*/

// TODO: Find GCC optimization flags


/**
 * @brief  Calculates Beam Azimuth Angle.
 * @note   By DF
 * @param  n_beams:     Total number of beams
 * @param  beam_num:    Beam number for calculating Beam Azimuth
 * @param  beam_sep:    Beam Seperation (in degrees)
 * @retval Returns the Beam Azumuth Angle (in radians)
 */
double calc_beam_angle(int n_beams, int beam_num, double beam_sep) {
    // Calculate Beamforming shift
    double center_beam = ( (double) n_beams - 1) / 2;

    // Calculate Beam Azimuth
    double b_azi = ((beam_num - center_beam) * beam_sep) * PI / 180;
    if (VERBOSE){
        log_trace("n_beams: %d, beam_num: %d, beam_sep: %lf", n_beams, beam_num, beam_sep);
        log_trace("    beam = %lf degree", (b_azi / PI * 180));
    }
    return b_azi;
}

/**
 * @brief  Calculates phase increment between antennas to produce a mainlobe sterring 
 *         of beam_angle at center_frequency.
 * @note   By DF
 * @param  beam_angle:          Distrubution of the beam (in radians)
 * @param  center_frequency:    Frequency at the center of the beam; 
 * * used to phase-shift allign the other frequencys (in Hz)
 * @param  x_spacing:           Spacing in-between antennas
 *         
 * @retval Returns the Phase Shift (in degrees)
 */
float calc_phase_increment(float beam_angle, double center_frequency, double x_spacing) {
    double wavelength = C / center_frequency;
    double phase_shift = (2 * PI * x_spacing * sin(beam_angle)) / wavelength;
    if (VERBOSE) {
        log_trace("search_center_freq: %lfx_spacing: %lfphase_shift: %lf degree", center_frequency, x_spacing, phase_shift * 180 / PI);
    }
    return phase_shift; 
}

/**
 * Converts radians to rectangular form
 */
double complex rad_to_rect(double phase) {
    return cexp(I * phase);
}

/**
 * @brief  Fast Fourier Transform (FFT) conversion from Time fft_spectrum  
 * * into Frequency spectrum.
 * @note    By DF
 * @param  *fft_spectrum: Array of fft_spectrum that have already been 
 * * prepared for FFT
 * @param  number_of_samples: Number of fft_spectrum
 * @param  *spectrum: Output array for the resultant spectrum
 * @retval None
 * 
 * @deprecated At the time of writing, each component of the function is used 
 * * separately for fft averaging
 */
void fft_samples(fftw_complex *fft_spectrum, int num_samples, fftw_complex *spectrum) {
    fftw_plan plan = fftw_plan_dft_1d(num_samples, fft_spectrum, spectrum, FFTW_FORWARD, FFTW_ESTIMATE);
    fftw_execute(plan);
    fftw_destroy_plan(plan);
}

/**
 * * @brief  Convolves 'u' array with 'v' array.
 * @note   By DF and B. Bristrow
 * @param  u: 1st Array
 * @param  u_size: Size of array 'u' 
 * @param  v: 2nd Array 
 * @param  v_size: Size of array 'v'
 * @param  *result: Array for Convolution Result
 * @retval N/A
 */
void convolve(double* u, int u_size, int* v, int v_size, double* result) {  
    for (int i = 0; i < u_size - v_size; i++) {
        result[i] = 0;
        for (int j = 0; j < v_size; j++) {
            result[i] += v[j] * u[i + j]; // result[i + (int) v_size] += ...
        }
        result[i] /= v_size;
    }
}

/**
 * @brief  Masks restricted frequency bands in the spectrum by applying RAND_MAX.
 * @note   By DF
 * @param  *spectrum: Spectrum the mask will be applied to.
 * @param  *freq_vector: Used to determine if the mask can be applied and where to start applying the mask element-wise.
 * @param  delta_f: Element step size, used to round the mask start and end bounds
 * @param  num_samples: Number of samples in the spectrum
 * @param  *restricted_bands: The restricted frequencies bands that should not
 * *  be transmitted on and should be masked.
 * @param  restricted_num: Number of restricted frequency bands in restricted_bands. 
 * @retval None
 */
void mask_restricted_freq(double *spectrum, double *freq_vector, int delta_f, int num_samples, freq_band *restricted_bands, int restricted_num) { 
    log_debug("    [mask_restricted] Masking restricted bands...");
    bool is_applied = false;

    // log_trace("spect range | %f -- %f |",  freq_vector[0], freq_vector[num_samples - 1]);

    // Mask each restricted band
    for (int i = 0; i < restricted_num; i++) {
        // Identify start and end of mask
        int mask_start = (int) restricted_bands[i].f_start;
        int mask_end = (int) restricted_bands[i].f_end;

        // For masks intersecting spectrum's freq range, apply mask
        if (( mask_end <= freq_vector[num_samples - 1] && mask_end > freq_vector[0] ) ||
            ( mask_start < freq_vector[num_samples - 1] && mask_start >= freq_vector[0])) {
                // Debug: Show masks applied
                if (VERBOSE) log_trace("    [MASK] Applying...  | %d -- %d|", mask_start, mask_end);

                // Apply spectrum freq range's floor or ceiling to mask's bounds
                int mask_sample_start, mask_sample_end;
                if (mask_start < freq_vector[0]) mask_sample_start = 0;
                else mask_sample_start = (mask_start - freq_vector[0]) / delta_f;
                
                if (mask_end >= freq_vector[num_samples - 1]) mask_sample_end = num_samples - 1;
                else mask_sample_end = (mask_end - freq_vector[0]) / delta_f;
                // log_trace("            Sample bounds... | %d -- %d|", mask_sample_start, mask_sample_end);
               
                // Apply mask
                for (int j = mask_sample_start; j <= mask_sample_end && j <= num_samples; j++) spectrum[j] = RAND_MAX;
                if (VERBOSE) log_trace("    [MASK]              | %d -- %d|", mask_sample_start, mask_sample_end);


                is_applied = true;
        }
    }
    if (is_applied) log_info("    [mask_restricted] Mask(s) applied!");
    else log_info("    [mask_restricted] No masks applied");

    // // Debug: Print Restricted Freqs
    // for (int i = 0; i < restricted_num; i++) {
    //     log_trace("Restricted[%d]: %d -- %d", i, restricted_bands[i].f_start, restricted_bands[i].f_end);
    // }      
}

// TODO: Parse radar_config_constants.py for CLRFREQ_RES
// TODO: Strike balance b/w speed and time using clear_sample_bw
// Try convolve with filter then find min of convolve
//      gnu or intel scientific library
/**
 * @brief  Processes Spectrum data to find the lowest noise frequency bands 
 * * (returned as clr_freq_bands). Steps are as follows: 
 * * (1) Setup Freq Search parameter data,
 * * (2) Scan Search Range with Bandpass Filter by way of convolution, 
 * * (3) Find Noise of BPF and compare w/ current Clear Freq Bands, and
 * * (4) If appropriate spot found, insert BPF's Range as New Clear Freq Band.
 * * * Overwriting any pre-existing, Intersecting Clear Freq Bands as necesary.  
 * @note   By DF
 * @param  *spectrum: Spectrum Data (Power per Sample)
 * @param  meta_data: Misc info on operating Radar parameters  
 * @param  delta_f: Frequency step per Sample 
 * @param  f_start: Clear Freq Search Bound start
 * @param  f_end: Clear Freq Search Bound end
 * @param  clear_bw: Bandwidth of the Clear Frequency Bands. If 0, default to 40kHz.
 * @param  *lowest_freq_bands: Passed by reference; Overwritten with an array of the 
 * * lowest noise freq_bands.  
 * @retval None
 */
void find_clear_freqs(double *spectrum, sample_meta_data meta_data, double delta_f, double f_start, double f_end, int clear_bw, freq_band *clr_bands) {
    
    log_debug("[find_clear_freqs()] Entered find_clear_freqs()...");
    if (clear_bw == 0) clear_bw = 5e3;
    clear_bw *= GB_MULT;
    int clear_sample_bw = (clear_bw) / CLRFREQ_RES; 


    // Define Range of Clear Freq Search 
    int spectrum_sample_start = (int) ((meta_data.usrp_fcenter * 1000 - meta_data.usrp_rf_rate / 2) / delta_f);
    int spectrum_sample_end = (int) ((meta_data.usrp_fcenter * 1000 + meta_data.usrp_rf_rate / 2) / delta_f);
    int clr_search_sample_start = (int) (f_start / delta_f) - spectrum_sample_start;
    int clr_search_sample_end = (int) (f_end / delta_f) - spectrum_sample_start;
    if (clr_search_sample_start < 0) clr_search_sample_start = 0;
    else if (clr_search_sample_start > spectrum_sample_end) clr_search_sample_start = spectrum_sample_end;
    if (clr_search_sample_end < 0) clr_search_sample_end = 0;
    else if (clr_search_sample_end > spectrum_sample_end) clr_search_sample_end = spectrum_sample_end;

    log_trace("spectrum_sample_start: %d     f_start: %f", spectrum_sample_start, f_start);

    // Trim Spectrum Data to only Clear Search Range (Used for convolving)
    int clr_search_sample_bw = clr_search_sample_end - clr_search_sample_start;
    double clr_search_band[clr_search_sample_bw];
    memset(clr_search_band, 0, clr_search_sample_bw * sizeof(double));
    for (int i = 0; i < clr_search_sample_bw; i++) {
        clr_search_band[i] = spectrum[i + clr_search_sample_start];

        // Debug: Check the Clear Search Range 
        // if (i < 2  || i > clr_search_sample_bw - 2) {
        //     log_trace("clr_search_band[%d]: %f", i, clr_search_band[i]);
        //     log_trace("                   : %f ", spectrum[i + clr_search_sample_start]);
        // }
    }

    // Scan Search range w/ Bandpass Filter (BPF) to find Clear Freq Band
    // log_debug("[find_clear_freqs()] Scanning Search Range w/ Bandpass...");
    int bpf[clear_sample_bw];
    // bpf = (int *) malloc(sizeof(int) * clear_sample_bw);
    for (int band_i = 0; band_i < clear_sample_bw; band_i++) {
        bpf[band_i] = 1;
    }
    if (VERBOSE) log_trace("    clr_search_sample_start: %d    clr_search_sample_end: %d    clr_search_sample_bw: %d", clr_search_sample_start, clr_search_sample_end, clr_search_sample_bw);

    // Convolve BPF with Search Range
    int convolve_bw = clr_search_sample_bw - clear_sample_bw;
    double *convolve_result = NULL;
    convolve_result = calloc(convolve_bw, sizeof(double));
    convolve(clr_search_band, clr_search_sample_bw, bpf, clear_sample_bw, convolve_result);
    log_debug("    Convolved Scan Band and BPF...");

    // Debug: Check convolve result
    // for (int i = 0; i < clr_search_sample_bw; i++)
    // {
    //     if (i < 10) log_trace("convolve[%d]: %f", i, convolve_result[i]);
    // }
    
    // Initialize Clear Freq Bands
    for (int i = 0; i < CLR_BANDS_MAX; i++) {
        clr_bands[i].f_start = clr_search_sample_start * delta_f - (meta_data.usrp_rf_rate / 2) + meta_data.usrp_fcenter * 1000;
        clr_bands[i].f_end = clr_search_sample_end * delta_f - (meta_data.usrp_rf_rate / 2) + meta_data.usrp_fcenter * 1000;
        clr_bands[i].noise = RAND_MAX; // XXX: Logic Flip
    };
    int min_idx[CLR_BANDS_MAX];
    
    // Identify lowest noise bands from convolve results...
    freq_band curr_band;
    for (int i = 0; i < convolve_bw; i++) {
        curr_band.f_start = (spectrum_sample_start + clr_search_sample_start + i) * delta_f;
        curr_band.f_end = (spectrum_sample_start + clr_search_sample_start + i + clear_sample_bw) * delta_f;
        curr_band.noise = convolve_result[i];
        // if (i < 10) log_trace("[%d] | %d -- %f -- %d|", i, curr_band.f_start, curr_band.noise, curr_band.f_end);
        
        int insert_idx = -1;
        int intersect_idx = -1;
        // Find Insertion spot in clr_freq_bands
        // Compare curr power with min_powers...
        for (int j = CLR_BANDS_MAX - 1; j >= 0 ; j--) {
            // Update Insert Index; maintaining ascending order 
            if (curr_band.noise < clr_bands[j].noise && curr_band.noise > 0 && curr_band.noise < RAND_MAX) { // XXX: Logic Flip
                insert_idx = j;
            }
            // Check for Intersecting Band; get intersecting clr_band index
            if ( //intersect_idx == -1 && 
                ((clr_bands[j].f_start < curr_band.f_start && curr_band.f_start < clr_bands[j].f_end) ||
                    (clr_bands[j].f_start < curr_band.f_end && curr_band.f_end < clr_bands[j].f_end))) {
                intersect_idx = j;
            }
            // Continue Intersection Search 
        }
        // log_debug("    Intersection Search finished...");
        // log_trace("    intersect_idx: %d    insert_idx: %d", intersect_idx, insert_idx);

        // Insertion Point was Found...
        if (insert_idx != -1) {
            // Intersection w/ curr_band was also Found...
            if (intersect_idx != -1) {
                // Special: If Intersect has worse noise, do not place/skip
                if (insert_idx > intersect_idx) continue;
                // log_trace("    Intersecting Insertion found w/...");
                freq_band inter_band = clr_bands[intersect_idx];

                // log_trace("        i-band = | %d -- %f -- %d|", inter_band.f_start, inter_band.noise, inter_band.f_end);

                // Special: Shift right till the Intersecting band is overwritten 
                if (insert_idx < intersect_idx) {
                    // Debug: verify bands shift properly @ sample
                    // if (i == 10) for (int j = 0; j < CLR_BANDS_MAX; j++) {
                    //     log_trace("Clear Freq Band[%d]: | %dMHz -- Noise: %f -- %dMHz |", j, clr_bands[j].f_start, clr_bands[j].noise, clr_bands[j].f_end);
                    // }
                    
                    // log_trace("        shifting clr_bands for intersect...");
                    for (int j = intersect_idx - 1; j >= insert_idx; j--) {
                        if (j + 1 < CLR_BANDS_MAX) {
                            clr_bands[j + 1] = clr_bands[j];
                            min_idx[j + 1] = min_idx[j];
                        }
                    }
                }
            } 
            // Only Insertion Point Found...
            else {
                // log_trace("    Insertion found...");
                // Special: Keep pre-existing bands by shifting them to right
                for (int j = CLR_BANDS_MAX - 2; j >= insert_idx; j--) {
                    // log_trace("        shifting clr_bands for insert...");
                    if (j + 1 < CLR_BANDS_MAX) {
                        clr_bands[j + 1] = clr_bands[j];
                        min_idx[j + 1] = min_idx[j];
                    }
                }
            }

            // Insert curr_band and store its sample index
            clr_bands[insert_idx] = curr_band;
            min_idx[insert_idx] = i;

            // Debug: verify shifting @ sample   
            // if (i == 10) for (int j = 0; j < CLR_BANDS_MAX; j++) {
                // log_trace("Clear Freq Band[%d]: | %dMHz -- Noise: %f -- %dMHz |", j, clr_bands[j].f_start, clr_bands[j].noise, clr_bands[j].f_end);
            // }
        }
    }

    // log_debug("    Convolution Results packed up...");

    // Debug: Output Final Clear Freq Bands
    // for (int i = 0; i < CLR_BANDS_MAX; i++)
    //     log_trace("Clear Freq Band[%d]: | %dMHz -- Noise: %f -- %dMHz |", i, clr_bands[i].f_start, clr_bands[i].noise, clr_bands[i].f_end);

    // Free allocated memory
    free(convolve_result);
    // free(bpf);

    log_info("[find_clear_freqs()] Exiting find_clear_freqs()...");
}


// HACK apply efficient matrix multi via cblas_dgemm
void calc_clear_freq_on_raw_samples(fftw_complex **raw_samples, sample_meta_data *meta_data, freq_band *restricted_bands, int restricted_num, int *clear_freq_range, double beam_angle, double smsep, freq_band *clr_bands) {
    int **sample_re = NULL;
    int **sample_im = NULL;
    
    // Extract meta data
    int num_samples = meta_data->number_of_samples;
    int *antennas = meta_data->antenna_list;

    // Ensure inputs exist
    if (!raw_samples || !meta_data || !antennas) {
        log_fatal(stderr, "Error: Null input detected.");
        exit(EXIT_FAILURE);
    }

    // Allocate memory for Variables
    fftw_complex *phasing_vector = NULL;
    fftw_complex *beamformed_samples = NULL;
    phasing_vector = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * meta_data->num_antennas);
    beamformed_samples = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * num_samples);
    double freq_vector[num_samples];
    memset(freq_vector, 0, num_samples * sizeof(double));
    sample_im = (int**) malloc(sizeof(int*) * meta_data->num_antennas);
    sample_re = (int**) malloc(sizeof(int*) * meta_data->num_antennas);
    for (int i = 0; i < meta_data->num_antennas; i++) {
        sample_im[i] = (int *)malloc(meta_data->number_of_samples * sizeof(int));
        sample_re[i] = (int *)malloc(meta_data->number_of_samples * sizeof(int));
    }   
    if (phasing_vector == NULL || beamformed_samples == NULL) {
        perror("Error allocating memory.");
        exit(EXIT_FAILURE);
    }

    // if (VERBOSE) {   
    //     for (int i = 0; i < num_samples; i++) if (i < 5) {
    //         log_trace("raw_samples[0][%d]: %f + %fi", i, creal(raw_samples[0][i]), cimag(raw_samples[0][i]));
    //         log_trace("raw_samples[1][%d]: %f + %fi", i, creal(raw_samples[1][i]), cimag(raw_samples[1][i]));
    //     }
    // }

    phasing_and_beamforming(beam_angle, clear_freq_range, meta_data, phasing_vector, antennas, num_samples, raw_samples, sample_im, sample_re, beamformed_samples);

    // if (VERBOSE) {for (int i = 0; i < num_samples; i++) if (i < 5) log_trace("beamformed[%d]    = %f + %fi", i, creal(beamformed_samples[i]), cimag(beamformed_samples[i]));}
    
    // Frequency Vector Calculation
    double delta_f = meta_data->usrp_rf_rate / num_samples;
    double f_start = meta_data->usrp_fcenter * 1000 - (meta_data->usrp_rf_rate / 2);
    for (int i = 0; i < num_samples; i++) {
        freq_vector[i] = i * delta_f + f_start;
    }

    /// Spectrum Calculation and Averaging; delinate Transmitters and filter out noise
    // Spectrum Averging (avg of 4 fft)
    // if (SPECTRAL_AVGING) {
    log_debug("=----Starting Spectral Average----=");
    int avg_freq_ratio = 4;     //(int) delta_f / CLRFREQ_RES;
    int num_avg_samples = num_samples / avg_freq_ratio; 

    // Determine Avg Freq Vector; used in Clear Freq Calculation
    double *freq_vector_avg = (double*) malloc(sizeof(double) * num_avg_samples);
    int delta_f_avg = delta_f * avg_freq_ratio;
    delta_f = delta_f_avg;
    for (int i = 0; i < num_avg_samples; i++) freq_vector_avg[i] = i * delta_f_avg + f_start;

    log_debug("[SpectAvg] done with avg freq vector");

    double *avg_spectrum = (double*) calloc(num_avg_samples, sizeof(double));
    fftw_complex *fft_spectrum = NULL;
    fft_spectrum = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * num_samples);
    if (fft_spectrum == NULL) {
        perror("Error allocating memory.");
        exit(EXIT_FAILURE);
    }
    if (VERBOSE) log_trace("num_avg_sample: %davg_freq_ratio: %d", num_avg_samples, avg_freq_ratio);
    
  
    clock_t t_avg_curr, t_avg;
    t_avg_curr = clock();

    // FFT Beamformed Samples
    // TODO: Optimize plan usage; Long-term plan storage (create and store in samples_server.c)
    fftw_plan plan = fftw_plan_dft_1d(num_samples, beamformed_samples, fft_spectrum, FFTW_FORWARD, FFTW_ESTIMATE);
    fftw_execute(plan);    

    // Spectral Averaging 
    // (0, 1, 2, 3 -> avg[0]), ..., (n-4, n-3, n-2, n-1 -> avg[n/4])
    // For every averaged element, ...
    for (int k = 0; k < num_avg_samples; k++) {
        // Avg the magnitude of four spectrum samples in a row 
        for (int j = 0; j < avg_freq_ratio; j++) {
            double re = creal(fft_spectrum[k * avg_freq_ratio + j]) * creal(fft_spectrum[k * avg_freq_ratio + j]);
            double im = cimag(fft_spectrum[k * avg_freq_ratio + j]) * cimag(fft_spectrum[k * avg_freq_ratio + j]);
            avg_spectrum[k] += sqrt(re + im);

            // if (k == 9) {
            //     log_trace("sample[%d][%d][%d]: %f + j%f", k, j, k + j, creal(fft_spectrum[k + j]), cimag(fft_spectrum[k + j]));
            // }
        }
        avg_spectrum[k] /= avg_freq_ratio;

        if (k == 9 && VERBOSE) log_trace("avg_spectrum[%d]: %f", k, avg_spectrum[k]);
    }
    t_avg = clock() - t_avg_curr;
    if (VERBOSE) log_info("====> Spectral Avg took (s): %lf", ((double) (t_avg)) / (CLOCKS_PER_SEC));
    if (VERBOSE) log_info("avg_spectrum[0]: %f", avg_spectrum[0]);


    // Dispose of temp variables
    fftw_destroy_plan(plan);
    fftw_free(fft_spectrum);

    /// END of Spectrum Calculations
    
    // if (VERBOSE) log_trace("delta_f: %fnum_samples: %dfcenter: %d", delta_f, num_samples, meta_data->usrp_fcenter * 1000);


    // Mask restricted frequencies
    if (restricted_bands != NULL) mask_restricted_freq(avg_spectrum, freq_vector_avg, delta_f_avg, num_avg_samples, restricted_bands, restricted_num);
    log_trace("------f_start: %f      f_end: %f",freq_vector_avg[0], freq_vector_avg[num_avg_samples - 1]);

    // Define Clear Freq Range from Hz to sample index
    int clear_sample_start = (int) round((clear_freq_range[0] - f_start) / delta_f);
    int clear_sample_end = (int) round((clear_freq_range[1] - f_start) / delta_f);
    log_trace("clear_range: | %d -- %d |", clear_freq_range[0], clear_freq_range[1]);
    log_trace("    samples: | %d -- %d |", clear_sample_start, clear_sample_end);
    // if (VERBOSE){ for (int i = clear_sample_start; i < clear_sample_end; i++) {
    //     // if (i < 2 + clear_sample_start || i > clear_sample_end - 3) 
    //     log_trace("spectrum_pow[%d]: %f", i, avg_spectrum[i]);   
    // }}

    // Find clear frequency
    double clear_bw = 4e6 / smsep; // ~ 300 us
    clear_bw = 0;
    clock_t t1, t2;
    t1 = clock();
    find_clear_freqs(avg_spectrum, *meta_data, delta_f_avg, clear_freq_range[0], clear_freq_range[1], clear_bw, clr_bands);
    t2 = clock();
    if (VERBOSE) log_info("find_clear_freqs (s): %lf", ((double) (t2 - t1)) / (CLOCKS_PER_SEC));

    // Debug: Output results
    for (int i = 0; i < CLR_BANDS_MAX; i++)
        log_debug("Clear Freq Band[%d][%s]: | %dHz -- Noise: %f -- %dHz |", i, clr_bands[i].is_selected ? "Selected" : "Free", clr_bands[i].f_start, clr_bands[i].noise, clr_bands[i].f_end);
    
    // // Debug: Print Restricted Freqs
    // for (int i = 0; i < restricted_num; i++) {
    //     log_trace("Restricted[%d]: %d -- %d", i, restricted_bands[i].f_start, restricted_bands[i].f_end);
    // }      
    

    // Save data to csv
    if (access(SPECTRAL_LOG_FILE, F_OK) == 0) {        
        // Write logs if its folder accessable
        if (BIN_OR_CSV_LOG == 0) {
            write_spectrum_mag_bin(SPECTRUM_FILE, avg_spectrum, freq_vector_avg, num_avg_samples);
            write_clr_freq_bin(CLR_FREQ_FILE, clr_bands);                                           // Used to plot Clear Freq Bands w/ spectrum_plot.clr_freq.py
        } else {
            // write_sample_mag_csv(sample_im_file, sample_im, freq_vector, meta_data);                                                     // Used to check complex Samples after Beamforming; ...
            // write_sample_mag_csv(sample_re_file, sample_re, freq_vector, meta_data);                                                     // Plot w/ sample_plot.py
            write_spectrum_mag_csv(SPECTRUM_FILE, avg_spectrum, freq_vector_avg, num_avg_samples);  // Spectrum after Spectrum FFT averaging; plot w/ spectrum_plot.py
            write_clr_freq_csv(CLR_FREQ_FILE, clr_bands);
        }
        log_warn("\'save_spectra\' found; Logged individual FFT Spectrum and Clear Frequency batches.");
    } else log_warn("\'save_spectra\' not found. Not logging spectra nor clr_frequency.");

    log_debug("Finished Clear Freq Search!");
    
    // Free allocated mem 
    fftw_free(phasing_vector);
    fftw_free(beamformed_samples);
    fftw_cleanup();
    log_trace("freed allocated fftw & its ptrs");
    free(freq_vector_avg);
    free(avg_spectrum);
    for (int i = 0; i < meta_data->num_antennas; i++) {
        free(sample_im[i]);
        free(sample_re[i]);
    }
    free(sample_im);
    free(sample_re);
    log_trace("freed allocated ptrs ");
}

void phasing_and_beamforming(double beam_angle, int *clear_freq_range, sample_meta_data *meta_data, fftw_complex *phasing_vector, int *antennas, int num_samples, fftw_complex **raw_samples, int **sample_im, int **sample_re, fftw_complex *beamformed_samples)
{
    // Calculate and Apply phasing vector
    float phase_increment = calc_phase_increment(beam_angle, (clear_freq_range[0] + clear_freq_range[1]) / 2, meta_data->x_spacing);
    if (VERBOSE) log_trace("phase_increment: %lf", phase_increment);

    for (int aidx = 0; aidx < meta_data->num_antennas; aidx++) {
        if (antennas[aidx] <= IDX_LAST_MA || antennas[aidx] > IDX_LAST_IA) {
            phasing_vector[aidx] = rad_to_rect(antennas[aidx] * phase_increment);
        } 
        else phasing_vector[aidx] = 0;

        if (VERBOSE) {
            // log_trace("ant[%d]: %d", i, antennas[i]);
            log_trace("phasing vec[%d]: %f + %fi", aidx, creal(phasing_vector[aidx]), cimag(phasing_vector[aidx]));
        }
    }

    // Apply beamforming
    for (int i = 0; i < num_samples; i++) {
        double real_sum = 0.0;
        double imag_sum = 0.0;

        for (int aidx = 0; aidx < meta_data->num_antennas; aidx++) {
            double real_sample = creal(raw_samples[aidx][i]);
            double imag_sample = cimag(raw_samples[aidx][i]);
            double real_phase = creal(phasing_vector[aidx]);
            double imag_phase = cimag(phasing_vector[aidx]);
            if (VERBOSE && i == 2499) {
                log_trace("sample[%d][2499]    = %f + %fi", aidx, real_sample, imag_sample);
                log_trace("phase[%d]           = %f + %fi", aidx, real_phase, imag_phase);
            }

            real_sum += real_sample * real_phase - imag_sample * imag_phase;
            imag_sum += real_sample * imag_phase + imag_sample * real_phase;

            // Store for debugging/logging
            sample_im[aidx][i] = (int) cimag(raw_samples[aidx][i]);
            sample_re[aidx][i] = (int) creal(raw_samples[aidx][i]);
        }
        beamformed_samples[i] = (fftw_complex) (real_sum + I * imag_sum);

        if (VERBOSE && i == 2499)
            log_trace("beamformed[%d]    = %f + %fi", i, creal(beamformed_samples[i]), cimag(beamformed_samples[i]));
    }
}

clear_freq clear_freq_search(
        fftw_complex **raw_samples, 
        int clear_freq_range[],
        int beam_num,
        int smsep,
        freq_band *restricted_bands, 
        int restrict_num,
        sample_meta_data meta_data,
        freq_band *clr_bands
    ) {
    const char *config_path = "../SuperDARN_UHD_Server/array_config.ini";              //"../Freq_Server/utils/clear_freq_input/array_config.ini";

    // Initial Data Variables
    int n_beams;
    double beam_sep;
    freq_data freq_data;

    // Scale parameters to Hz and ms
    // double clear_freq_range[] = { 12 * pow(10,6), 12.5 * pow(10,6) };
    // double beam_angle = 0.08482300164692443;        // in radians
    // double smsep = .0003; // 1 / (2 * 250 * pow(10, 3));      // ~4 ms
    smsep = smsep / 1000000;
    if (clear_freq_range[0] < 100000 || clear_freq_range[1] < 100000) {
        clear_freq_range[0] = clear_freq_range[0] * 1000; 
        clear_freq_range[1] = clear_freq_range[1] * 1000;
    }

    // Beam Angle Calculation
    read_array_config(config_path, &n_beams, &beam_sep);
    double beam_angle = calc_beam_angle(n_beams, beam_num, beam_sep);  

    // Debug: Display parameters
    log_trace("=--- Clear Freq Variables ---=");
    log_trace("num_samples: %dnum_antennas: %dx_spacing: %lfusrp_rf_rate: %dusrp_fcenter: %d",
        meta_data.number_of_samples,
        meta_data.num_antennas,
        meta_data.x_spacing,
        meta_data.usrp_rf_rate,
        meta_data.usrp_fcenter
    );       
    log_trace("n_beams: %dbeam_sep: %fbeam_num: %dbeam_angle: %f", n_beams, beam_sep, beam_num, beam_angle);


    // Stopwatch Start
    double t1,t2;
    t1 = clock();

    // Find Clear Frequency Bands
    calc_clear_freq_on_raw_samples(
        raw_samples, 
        &meta_data, 
        restricted_bands, 
        restrict_num, 
        clear_freq_range, 
        beam_angle, 
        smsep, 
        clr_bands
    );
    
    // Print processing time; Stopwatch End
    t2 = clock();
    log_info("clear_freq_search (s): %lf", ((double) (t2 - t1)) / (CLOCKS_PER_SEC));
};
