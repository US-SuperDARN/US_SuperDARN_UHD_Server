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
#include "clear_frequency_server.h"
#include "log.h"





// TODO: Pass in clr_freq_range via restrict actual file
// #define RESTRICT_FILE = '/home/radar/repos/SuperDARN_MSI_ROS/linux/home/radar/ros.3.6/tables/superdarn/site/site.sps/restrict.dat.inst'

// TODO: Pass in x_spacing, etc. via config actual file

// TODO: Store Clear Freq Bands for each Beam 

/*
* NOTE: 
* When compiling make sure to add "-lm" and "-lfftw3" to link the libraries 
* to the compilier. 
*/

// Global or static variable for the FFT plan
fftw_plan storage_fft_plan = NULL;

// Initialization function to create the plan
void init_storage_fft(int num_samples, int beam_total) {
    fftw_complex *input = fftw_alloc_complex(num_samples * beam_total);
    fftw_complex *output = fftw_alloc_complex(num_samples * beam_total);
    
    if (storage_fft_plan == NULL) {
        int n[] = {num_samples};
        storage_fft_plan = fftw_plan_many_dft(
            1,                // Rank (1D FFT for each beam)
            n,                // FFT size
            beam_total,       // Number of transforms (one per beam)
            input,            // Input array
            NULL,             // Input stride (NULL for contiguous data)
            1,                // Distance between successive input elements
            num_samples,      // Distance between successive input transforms
            output,           // Output array
            NULL,             // Output stride (NULL for contiguous data)
            1,                // Distance between successive output elements
            num_samples,      // Distance between successive output transforms
            FFTW_FORWARD,     // FFT direction
            FFTW_PATIENT      // Plan flag
        );        
        if (!storage_fft_plan) {
            perror("Error creating FFT plan");
            exit(EXIT_FAILURE);
        }
        log_info("FFT plan created and cached.");
    }

    fftw_free(input);
    fftw_free(output);
}

// Function to execute the FFT using the precomputed plan
void execute_storage_fft(fftw_complex *input, fftw_complex *output) {
    fftw_execute_dft(storage_fft_plan, input, output);
}

// Cleanup function to destroy the plan
void cleanup_storage_fft() {
    if (storage_fft_plan != NULL) {
        fftw_destroy_plan(storage_fft_plan);
    }
}

// Initialize FFTW multi-threading
void initialize_fftw_threads(int num_threads) {
    fftw_init_threads();
    fftw_plan_with_nthreads(num_threads);
    log_info("FFTW initialized with %d threads", num_threads);
}

// Cleanup FFTW multi-threading
void cleanup_fftw_threads() {
    fftw_cleanup_threads();
    log_info("FFTW threads cleaned up");
}

/**
 * @brief  Calculates Beam Azimuth Angle.
 * @note   By DF
 * @param  n_beams:     Total number of beams
 * @param  cur_beam:    Beam number for calculating Beam Azimuth
 * @param  beam_sep:    Beam Seperation (in degrees)
 * @retval Returns the Beam Azumuth Angle (in radians)
 */
double calc_beam_angle(int n_beams, int cur_beam, double beam_sep) {
    // Calculate Beamforming shift
    double center_beam = ( (double) n_beams - 1) / 2;

    // Calculate Beam Azimuth
    double b_azi = ((cur_beam - center_beam) * beam_sep) * PI / 180;
    if (VERBOSE){
        log_trace("n_beams: %d, cur_num: %d, beam_sep: %lf", n_beams, cur_beam, beam_sep);
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
float calc_phase_increment(float beam_angle, int center_frequency, double x_spacing) {
    double wavelength = C / center_frequency;
    double phase_shift = (2 * PI * x_spacing * sin(beam_angle)) / wavelength;
    if (VERBOSE) {
        log_trace("search_center_freq: %d x_spacing: %lf phase_shift: %lf degree", center_frequency/1000, x_spacing, phase_shift * 180 / PI);
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
    fftw_plan fft_plan = fftw_plan_dft_1d(num_samples, fft_spectrum, spectrum, FFTW_FORWARD, FFTW_ESTIMATE);
    fftw_execute(fft_plan);
    fftw_destroy_plan(fft_plan);
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
        // result[i] /= v_size;
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
                if (VERBOSE) log_trace("    [MASK] Applying...  | %5d -- %5d |", mask_start/1000, mask_end/1000);

                // Apply spectrum freq range's floor or ceiling to mask's bounds
                int mask_sample_start, mask_sample_end;
                if (mask_start < freq_vector[0]) mask_sample_start = 0;
                else mask_sample_start = (mask_start - freq_vector[0]) / delta_f;
                
                if (mask_end >= freq_vector[num_samples - 1]) mask_sample_end = num_samples - 1;
                else mask_sample_end = (mask_end - freq_vector[0]) / delta_f;
                // log_trace("            Sample bounds... | %d -- %d|", mask_sample_start, mask_sample_end);
               
                // Apply mask
                for (int j = mask_sample_start; j <= mask_sample_end && j <= num_samples; j++) spectrum[j] = RAND_MAX;
                // if (VERBOSE) log_trace("    [MASK]              | %d -- %d|", mask_sample_start, mask_sample_end);


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

/**
 * @brief  Processes Spectrum data to find the lowest noise frequency bands 
 * * (returned as clr_freq_bands). Steps are as follows: 
 * * (1) Setup Freq Search parameters (Clear Freq Search Range, Clear Freq Bandwidth, etc.),
 * * (2) Scan Search Range with Bandpass Filter (BPF) by way of convolution, 
 * * (3) Find Noise of BPF and compare w/ Clear Freq Bands as they are found, and
 * * (4) If appropriate Clear Freq Band, insert it as New Clear Freq Band in an ascending noise list of clear freq bands.
 * * * Overwriting any intersecting, higher noise Clear Freq Bands as necesary.  
 * @note   By DF
 * @param  *spectrum: Spectrum Data (Power per Sample)
 * @param  meta_data: Misc info on operating Radar parameters  
 * @param  avg_delta_f: Frequency step per Sample post spectral averaging
 * @param  f_start: Clear Freq Search Boundary start
 * @param  f_end: Clear Freq Search Boundary end
 * @param  clear_bw: Bandwidth of the Clear Frequency Bands
 * @param  *clr_band: Passed by reference; Overwritten with the
 * * lowest noise freq_band.
 * @retval None
 */
void find_clear_freqs(double *spectrum, sample_meta_data meta_data, double avg_delta_f, double f_start, double f_end, int clear_bw, freq_band *clr_band) {
    
    log_debug("Entered find_clear_freqs()...");
    int clear_sample_bw = ceil(clear_bw / avg_delta_f);  // Always round up to avoid any overlapping bands
    log_info("    Clear Freq Bandwidth: %d samples", clear_sample_bw);
    
    
    // Define Range of Clear Freq Search (f_start, f_end into clr_search_sample_start, clr_search_sample_end)
    int spectrum_sample_start = (int) ((meta_data.usrp_fcenter * 1000 - meta_data.usrp_rf_rate / 2) / avg_delta_f);
    int spectrum_sample_end = (int) ((meta_data.usrp_fcenter * 1000 + meta_data.usrp_rf_rate / 2) / avg_delta_f);
    int spectrum_sample_bw = spectrum_sample_end - spectrum_sample_start;
    int clr_search_sample_start = (int) (f_start / avg_delta_f) - spectrum_sample_start;
    int clr_search_sample_end = (int) (f_end / avg_delta_f) - spectrum_sample_start;
    
    // Set Search Range to within bounds of Spectrum Range
    if (clr_search_sample_start < 0) clr_search_sample_start = 0;
    else if (clr_search_sample_start > spectrum_sample_bw) clr_search_sample_start = spectrum_sample_bw;
    if (clr_search_sample_end < 0) clr_search_sample_end = 0;
    else if (clr_search_sample_end > spectrum_sample_bw) clr_search_sample_end = spectrum_sample_bw;


    // Trim Spectrum Data to only Clear Search Range (Used for convolving)
    int clr_search_sample_bw = clr_search_sample_end - clr_search_sample_start;
    log_info("    Clear Search Bandwidth: %d samples", clr_search_sample_bw);
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
    double *convolve_result = calloc(convolve_bw, sizeof(double));
    convolve(clr_search_band, clr_search_sample_bw, bpf, clear_sample_bw, convolve_result);
    log_debug("    Convolved Scan Band and BPF...");

    // Debug: Check convolve result
    // for (int i = 0; i < clr_search_sample_bw; i++)
    // {
    //     if (i < 10) log_trace("convolve[%d]: %f", i, convolve_result[i]);
    // }
    
    // Initialize Clear Freq Band
    clr_band->f_start = clr_search_sample_start * avg_delta_f - (meta_data.usrp_rf_rate / 2) + meta_data.usrp_fcenter * 1000;
    clr_band->f_end   = clr_search_sample_end * avg_delta_f - (meta_data.usrp_rf_rate / 2) + meta_data.usrp_fcenter * 1000;
    clr_band->noise   = RAND_MAX;
    
    // Identify lowest noise band from convolve results...
    freq_band curr_band;
    for (int i = 0; i < convolve_bw; i++) {
        curr_band.f_start = (spectrum_sample_start + clr_search_sample_start + i) * avg_delta_f;
        curr_band.f_end = (spectrum_sample_start + clr_search_sample_start + i + clear_sample_bw) * avg_delta_f;
        curr_band.noise = convolve_result[i];
        // log_trace("[%d] | %d -- %f -- %d|", i, curr_band.f_start, curr_band.noise, curr_band.f_end);
        
        // Select band if it has a lower noise
        if (curr_band.noise < clr_band->noise && curr_band.noise > 0 && curr_band.noise < CLR_NOISE_THRESHOLD && curr_band.noise < RAND_MAX) {
            clr_band->f_start = curr_band.f_start;
            clr_band->f_end   = curr_band.f_end;
            clr_band->noise   = curr_band.noise;
        }
    }

    // Success: Clr Band Found
    if (clr_band->f_start > 0 && clr_band->f_end > 0 && clr_band->noise < RAND_MAX) {
        log_trace("Found Clear Band:");
        log_trace("    | %d kHz -- Noise: %f -- %d kHz |",
            clr_band->f_start/1000,
            clr_band->noise,
            clr_band->f_end/1000
        );   
    }

    // Free allocated memory
    free(convolve_result);

    if (clr_search_sample_bw < (25 * clear_sample_bw) ) {
        log_warn("WARN: Clear Search Bandwidth is severely limiting CFS to: %d possible Clr Freqs", clr_search_sample_bw / clear_sample_bw);
    }

    log_debug("Exiting find_clear_freqs()...");
}


void calc_clear_freq_on_raw_samples(
    fftw_complex *raw_samples, 
    int *active_antennas,
    sample_meta_data *meta_data, 
    freq_band *restricted_bands, 
    int restricted_num, 
    int *clear_freq_range, 
    double beam_angle, 
    int smsep, 
    int avg_ratio,
    freq_band *clr_band
) {
    // int **sample_re = NULL;
    // int **sample_im = NULL;
    
    // Extract meta data
    int num_samples = meta_data->number_of_samples;
    int *antennas = meta_data->antenna_list;

    // Ensure inputs exist
    if (!raw_samples || !meta_data || !antennas) {
        log_fatal("Error: Null input detected.");
        exit(EXIT_FAILURE);
    }

    // Allocate memory for Variables
    fftw_complex *phasing_vector = NULL;
    fftw_complex *beamformed_samples = NULL;
    phasing_vector = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * num_samples);
    beamformed_samples = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * num_samples);
    double freq_vector[num_samples];
    memset(freq_vector, 0, num_samples * sizeof(double));
    // sample_im = (int**) malloc(sizeof(int*) * meta_data->num_antennas);
    // sample_re = (int**) malloc(sizeof(int*) * meta_data->num_antennas);
    // for (int i = 0; i < meta_data->num_antennas; i++) {
    //     sample_im[i] = (int *)malloc(meta_data->number_of_samples * sizeof(int));
    //     sample_re[i] = (int *)malloc(meta_data->number_of_samples * sizeof(int));
    // }   
    if (phasing_vector == NULL || beamformed_samples == NULL || freq_vector == NULL) {
        perror("Error allocating memory.");
        exit(EXIT_FAILURE);
    }

    // if (VERBOSE) {   
    //     for (int i = 0; i < num_samples; i++) if (i < 5) {
    //         log_trace("raw_samples[0][%d]: %f + %fi", i, creal(raw_samples[0][i]), cimag(raw_samples[0][i]));
    //         log_trace("raw_samples[1][%d]: %f + %fi", i, creal(raw_samples[1][i]), cimag(raw_samples[1][i]));
    //     }
    // }

    phasing_and_beamforming(
        beam_angle, 
        clear_freq_range, 
        meta_data, 
        phasing_vector, 
        antennas, 
        num_samples, 
        raw_samples, 
        active_antennas,
        beamformed_samples
    );

    // if (VERBOSE) {for (int i = 0; i < num_samples; i++) if (i < 5) log_trace("beamformed[%d]    = %f + %fi", i, creal(beamformed_samples[i]), cimag(beamformed_samples[i]));}
    
    // Frequency Vector Calculation
    double delta_f = meta_data->usrp_rf_rate / num_samples;
    double f_start = meta_data->usrp_fcenter * 1000 - (meta_data->usrp_rf_rate / 2);
    for (int i = 0; i < num_samples; i++) {
        freq_vector[i] = i * delta_f + f_start;
    }

    /// Spectrum Calculation and Averaging; delinate Transmitters and filter out noise
    log_debug("=----Starting Spectral Average----=");
    int num_avg_samples = num_samples / avg_ratio; 
    // log_trace("num_avg_samples: %d", num_avg_samples);
    // log_trace("avg_ratio: %d", avg_ratio);

    // Determine Avg Freq Vector; used in Clear Freq Calculation
    double *avg_freq_vector = (double*) malloc(sizeof(double) * num_avg_samples);
    int delta_f_avg = delta_f * avg_ratio;
    for (int i = 0; i < num_avg_samples; i++) avg_freq_vector[i] = i * delta_f_avg + f_start;

    double *avg_spectrum = (double*) calloc(num_avg_samples, sizeof(double));
    fftw_complex *fft_spectrum = NULL;
    fft_spectrum = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * num_samples);
    if (fft_spectrum == NULL) {
        perror("Error allocating memory.");
        exit(EXIT_FAILURE);
    }    
  
    clock_t t_avg_curr, t_avg;
    t_avg_curr = clock();

    // FFT Beamformed Samples
    // TODO: Optimize fft_plan usage; Long-term fft_plan storage (create and store in samples_server.c)
    fftw_plan fft_plan = fftw_plan_dft_1d(num_samples, beamformed_samples, fft_spectrum, FFTW_FORWARD, FFTW_ESTIMATE);
    fftw_execute(fft_plan);    

    // Spectral Averaging 
    // (0, 1, 2, 3 -> avg[0]), ..., (n-4, n-3, n-2, n-1 -> avg[n/4])
    // For every averaged element, ...
    for (int k = 0; k < num_avg_samples; k++) {
        // Avg the magnitude of four spectrum samples in a row 
        for (int j = 0; j < avg_ratio; j++) {
            double re = creal(fft_spectrum[k * avg_ratio + j]) * creal(fft_spectrum[k * avg_ratio + j]);
            double im = cimag(fft_spectrum[k * avg_ratio + j]) * cimag(fft_spectrum[k * avg_ratio + j]);
            avg_spectrum[k] += sqrt(re + im);

            // if (k == 9) {
            //     log_trace("sample[%d][%d][%d]: %f + j%f", k, j, k + j, creal(fft_spectrum[k + j]), cimag(fft_spectrum[k + j]));
            // }
        }
        avg_spectrum[k] /= avg_ratio;

        if (k == 0 && VERBOSE) log_trace("avg_spectrum[%d]: %f", k, avg_spectrum[k]);
    }
    t_avg = clock() - t_avg_curr;
    if (VERBOSE) log_info("====> Spectral Avg took (s): %lf", ((double) (t_avg)) / (CLOCKS_PER_SEC));


    // Dispose of temp variables
    fftw_destroy_plan(fft_plan);
    fftw_free(fft_spectrum);

    /// END of Spectrum Calculations
    
    
    // Mask restricted frequencies
    if (restricted_bands != NULL) mask_restricted_freq(avg_spectrum, avg_freq_vector, delta_f_avg, num_avg_samples, restricted_bands, restricted_num);
    log_trace("------f_start: %f      f_end: %f",avg_freq_vector[0]/1000, avg_freq_vector[num_avg_samples - 1]/1000);
    
    // Define Clear Freq Range from Hz to sample index
    int clear_sample_start = (int) round((clear_freq_range[0] - f_start) / delta_f_avg);
    int clear_sample_end = (int) round((clear_freq_range[1] - f_start) / delta_f_avg);
    log_trace("clear_range: | %d -- %d |", clear_freq_range[0]/1000, clear_freq_range[1]/1000);


    // Trasmission separation
    float gb = (1e6 * (GB_MULT - 1) ) / (smsep * 2);
    if (gb < MIN_FREQ_SEP) gb = MIN_FREQ_SEP;   // Ensure minimum frequency separation
    float clear_bw = 1e6 / smsep + gb;          // Clear Bandwidth in Hz
    log_info("clear_bw: %f Hz = %f Hz (signal) + %f Hz (guard)", clear_bw, 1e6 / smsep, gb);

    // Display delta_f and num_samples before and after averaging 
    if (VERBOSE) log_trace("delta_f: %f num_samples: %d", delta_f, num_samples);
    log_trace("delta_f_avg: %d num_avg_samples: %d", delta_f_avg, num_avg_samples);
    
    // Find clear frequency
    clock_t t1, t2;
    t1 = clock();
    find_clear_freqs(avg_spectrum, *meta_data, delta_f_avg, clear_freq_range[0], clear_freq_range[1], clear_bw, clr_band);
    t2 = clock();
    if (VERBOSE) log_info("find_clear_freqs (s): %lf", ((double) (t2 - t1)) / (CLOCKS_PER_SEC));

    // Debug: Output results
    // log_debug("Clear Freq Band: | %d kHz -- Noise: %f -- %d kHz |", clr_band.f_start/1000, clr_band.noise, clr_band.f_end/1000);
    
    // // Debug: Print Restricted Freqs
    // for (int i = 0; i < restricted_num; i++) {
    //     log_trace("Restricted[%d]: %d -- %d", i, restricted_bands[i].f_start/1000, restricted_bands[i].f_end/1000);
    // }      
    

    // Save data
    if (access(SPECTRAL_LOG_FILE, F_OK) == 0) {        
        // Write logs if its folder accessable
        if (BIN_OR_CSV_LOG == 0) {
            FILE *tmp_file = NULL;
            write_spectrum_mag_bin(
                &tmp_file,
                avg_spectrum, 
                avg_freq_vector, 
                num_avg_samples
            );

            fclose(tmp_file);
            tmp_file = NULL;

            write_clr_freq_bin(
                &tmp_file,
                clr_band, 
                clear_freq_range
            );                                           // Used to plot Clear Freq Bands w/ spectrum_plot.clr_freq.py
            fclose(tmp_file);
        } else {
            FILE *tmp_file = NULL;
            // write_sample_mag_csv(sample_im_file, sample_im, freq_vector, meta_data);                                                     // Used to check complex Samples after Beamforming; ...
            // write_sample_mag_csv(sample_re_file, sample_re, freq_vector, meta_data);                                                     // Plot w/ sample_plot.py
            write_spectrum_mag_csv(
                &tmp_file,
                avg_spectrum, 
                avg_freq_vector, 
                num_avg_samples
            );  // Spectrum after Spectrum FFT averaging; plot w/ spectrum_plot.py

            fclose(tmp_file);
            tmp_file = NULL;

            write_clr_freq_csv(
                &tmp_file, 
                clr_band, 
                clear_freq_range
            );

            fclose(tmp_file);
        }
        log_warn("\'save_spectra\' found; Logged individual FFT Spectrum and Clear Frequency batches.");
    } else log_warn("\'save_spectra\' not found. Not logging spectra nor clr_frequency.");

    log_debug("Finished Clear Freq Search!");
    
    fftw_free(phasing_vector);
    fftw_free(beamformed_samples);
    fftw_cleanup();
    log_trace("freed allocated fftw & its ptrs");
    free(avg_freq_vector);
    free(avg_spectrum);
    log_trace("freed allocated ptrs ");
}


/**
 * @brief  Phases and beamforms the raw samples. Also masks inactive antennas.
 * @note   By DF
 * @param  beam_angle: Angle of the beam to be formed.
 * @param  *clear_freq_range: Frequency range for the clear frequencies.
 * @param  *meta_data: Metadata containing sample information.
 * @param  *phasing_vector: Vector used for phasing.
 * @param  *antennas: List of antennas used for processing.
 * @param  num_samples: Number of samples to process.
 * @param  **raw_samples: Raw samples to be processed.
 * @param  *beamformed_samples: Output array for the beamformed samples.
 * @retval None
 */
void phasing_and_beamforming(
    double beam_angle, 
    int *clear_freq_range, 
    sample_meta_data *meta_data, 
    fftw_complex *phasing_vector, 
    int *antennas, 
    int num_samples, 
    fftw_complex *raw_samples, 
    int *active_antennas,
    fftw_complex *beamformed_samples
) {
    // Calculate and Apply phasing vector
    float phase_increment = 0;
    log_trace("clear_freq_range: | %d -- %d kHz |", clear_freq_range[0]/1000, clear_freq_range[1]/1000);
    phase_increment = calc_phase_increment(beam_angle, (clear_freq_range[0] + clear_freq_range[1]) / 2, meta_data->x_spacing);
    if (VERBOSE) log_trace("phase_increment: %lf", phase_increment);

    for (int a_idx = 0; a_idx < meta_data->num_antennas; a_idx++) {
        // log_debug("antenna[%d](%d)", a_idx, meta_data->antenna_list[a_idx]);

        if ((antennas[a_idx] <= IDX_LAST_MA || antennas[a_idx] > IDX_LAST_IA) &&
        (active_antennas[a_idx] == 1)) {
            // log_debug("antenna[%d] is active", a_idx);
            phasing_vector[a_idx] = rad_to_rect(antennas[a_idx] * phase_increment);
        } 
        else phasing_vector[a_idx] = 0;

        // if (VERBOSE) {
        //     // log_trace("ant[%d]: %d", i, antennas[i]);
            // log_trace("phasing vec[%d]: %f + %fi", aidx, creal(phasing_vector[aidx]), cimag(phasing_vector[aidx]));
        // }
    }

    // Apply beamforming
    double avg_pwr[meta_data->num_antennas];
    for (int i = 0; i < num_samples; i++) {
        double real_sum = 0.0;
        double imag_sum = 0.0;

        for (int a_idx = 0; a_idx < meta_data->num_antennas; a_idx++) {
            double real_sample = creal(raw_samples[a_idx * num_samples + i]);
            double imag_sample = cimag(raw_samples[a_idx * num_samples + i]);
            double real_phase = creal(phasing_vector[a_idx]);
            double imag_phase = cimag(phasing_vector[a_idx]);
            // if (VERBOSE && i == 2499) {
            //     log_trace("sample[%d][2499]    = %f + %fi", meta_data->antenna_list[a_idx], real_sample, imag_sample);
            //     log_trace("phase[%d]           = %f + %fi", meta_data->antenna_list[a_idx], real_phase, imag_phase);
            // }

            real_sum += real_sample * real_phase - imag_sample * imag_phase;
            imag_sum += real_sample * imag_phase + imag_sample * real_phase;

            // Debug: Accumulate for average power
            if (i == 0) { avg_pwr[a_idx] = real_sample * real_sample + imag_sample * imag_sample;}
            else {avg_pwr[a_idx] += real_sample * real_sample + imag_sample * imag_sample;}

            // Debug: On last sample, divide for average power
            if (i >= num_samples - 1) {
                avg_pwr[a_idx] = avg_pwr[a_idx] / num_samples;
                log_trace("   avg_pwr[%d]    = %f", meta_data->antenna_list[a_idx], avg_pwr[a_idx]);
                if (avg_pwr[a_idx] > MAX_ANT_PWR) {
                    log_warn("   ant#%d Pwr is high", meta_data->antenna_list[a_idx]);
                }
            }
        }
        beamformed_samples[i] = real_sum + I * imag_sum;

        // if (VERBOSE && i == 2499) {
        //     log_trace("beamformed[%d]    = %f + %fi", i, creal(beamformed_samples[i]), cimag(beamformed_samples[i]));
        // }
    }
}


/**
 * @brief  Processes all beams' fft spectrum.
 * @note   By DF
 * @param  *raw_samples: Raw samples to be processed.
 * @param  clear_freq_range: Frequency range for the clear frequencies.
 * @param  smsep: Sample separation in microseconds.
 * @param  *restricted_bands: Array of restricted frequency bands.
 * @param  restrict_num: Number of restricted frequency bands.
 * @param  meta_data: Metadata containing sample information.
 * @param  *beamformed_spectra: Output array for the beamformed spectra.
 * @retval None
 */
void process_all_beamformed_spectras(
        fftw_complex *raw_samples, 
        int *active_antennas,
        int clear_freq_range[],
        int smsep,
        freq_band *restricted_bands, 
        int restrict_num,
        sample_meta_data *meta_data,
        Config config,
        fftw_complex *beamformed_spectra
    ) {
    log_debug("Entered process_all_beamformed_spectras()...");

    // Initial Data Variables
    int beam_total = config.array_info.nbeams;
    double beam_sep = config.array_info.beam_sep;
    // int **sample_re = NULL;
    // int **sample_im = NULL;
    int num_samples = meta_data->number_of_samples;
    int *antennas = meta_data->antenna_list;


    // Ensure inputs exist
    if (!raw_samples || !meta_data || !antennas || !beam_total) {
        log_error("Error: Null input detected in process_all_beam_spectra().");
        exit(EXIT_FAILURE);
    }
    if (beam_total < 2) {
        log_warn("beam_total is below 2; Expect significantly reduced accuracy!");
    }
    log_trace("     Inputs exist in process_all_beam_spectra()");

    // Allocate memory for Variables    
    fftw_complex *phasing_vector = fftw_alloc_complex(beam_total * num_samples);
    fftw_complex *beamformed_samples = fftw_alloc_complex(beam_total * num_samples);
    // fftw_complex *beamformed_samples_ptr = beamformed_samples;
    // for (int cur_beam = 0; cur_beam < beam_total; cur_beam++) {
    //     phasing_vector[cur_beam]     = (fftw_complex*) fftw_malloc(num_samples * sizeof(fftw_complex));
    //     // beamformed_samples[cur_beam] = (fftw_complex*) fftw_malloc(num_samples * sizeof(fftw_complex));
    //     // beam_fft_spectrum[cur_beam]       = (fftw_complex*) fftw_malloc(num_samples * sizeof(int));
    // }
    if (!phasing_vector || !beamformed_samples) {
        perror("Error: Failed to allocate process_all_beam_spectra() memory.");
        exit(EXIT_FAILURE);
    }

    log_debug("     Allocated memory for phasing_vector and beamformed_samples");

    // Scale parameters to Hz and ms
    // smsep = smsep / 1000;
    log_trace("      clr_range: | %d -- %d |", clear_freq_range[0], clear_freq_range[1]);
    if (clear_freq_range[0] < 100000 || clear_freq_range[1] < 100000) {
        clear_freq_range[0] = clear_freq_range[0] * 1000; 
        clear_freq_range[1] = clear_freq_range[1] * 1000;
    }
    log_trace("      clr_range: | %d -- %d |", clear_freq_range[0], clear_freq_range[1]);

    // Beam Angle Calculation
    double beam_angle[beam_total];
    memset(beam_angle, 0, sizeof(beam_angle));
    for (int cur_beam = 0; cur_beam < beam_total; cur_beam++) {
        beam_angle[cur_beam] = calc_beam_angle(beam_total, cur_beam, beam_sep);  
        log_trace("     beam_angle[%d]: %f", cur_beam, beam_angle[cur_beam]);
    }
    log_debug("     Beam angles calculated");
    

    // Phasing and Beamforming Calculation
    fftw_complex *current_beam_samples = beamformed_samples;
    for (int cur_beam = 0; cur_beam < beam_total; cur_beam++) {
        current_beam_samples = beamformed_samples + cur_beam * num_samples;
        phasing_and_beamforming(
            (double) (beam_angle[cur_beam]), 
            clear_freq_range, 
            meta_data, 
            &(phasing_vector[cur_beam * num_samples]), 
            antennas, 
            num_samples, 
            raw_samples, 
            active_antennas,
            current_beam_samples
        );
        // for (int i = 0; i < num_samples; i++) if (i < 5 || i > 2495) log_trace("beamformed[%d]    = %f + %fi", i, creal(beamformed_samples[cur_beam * num_samples + i]), cimag(beamformed_samples[cur_beam * num_samples + i]));
    }
    log_debug("     Beamforming done");


    // FFT Beamformed Samples
    current_beam_samples = beamformed_samples;
    fftw_complex *fft_spectrum = beamformed_spectra;
    fftw_plan fft_plan = fftw_plan_dft_1d(num_samples, current_beam_samples, fft_spectrum, FFTW_FORWARD, FFTW_ESTIMATE);
    for (int cur_beam = 0; cur_beam < beam_total; cur_beam++) {
        // Set the input and output for the FFT
        current_beam_samples    = beamformed_samples + cur_beam * num_samples;
        fft_spectrum            = beamformed_spectra + cur_beam * num_samples;

        fftw_execute_dft(fft_plan, current_beam_samples, fft_spectrum);
    }
    fftw_destroy_plan(fft_plan);
    log_debug("     FFT done");

    // Debug: Print FFT results
    // for (int cur_beam = 0; cur_beam < beam_total; cur_beam++) {
    //     for (int i = 0; i < num_samples; i++) if (i < 5) {
    //         log_trace("fft_spectrum[%d][%d]: %f + %fi", cur_beam, i, creal(beamformed_spectra[cur_beam * num_samples + i]), cimag(beamformed_spectra[cur_beam * num_samples + i]));
    //     }
    // }
    
    // Dispose of temp variables
    fftw_free(phasing_vector);
    fftw_free(beamformed_samples);
    fftw_cleanup();
}

/**
 * @brief  Averages the collected beamformed spectrum data for a single beam direction.
 * @note   By DF
 * @param  ***beamformed_spectra: Beamformed spectra to be averaged.
 * @param  avg_ratio: Ratio for averaging the spectra.
 * @param  num_samples: Number of samples in the spectra.
 * @param  cur_beam: Current beam index.
 * @param  beam_num: Number of beams.
 * @param  spectra_num: Number of spectra in the min of stored spectra.
 * @param  *meta_data: Metadata containing sample information.
 * @param  **avg_beam_spectra: Output array for the averaged beam spectra.
 * @param  *avg_freq_vector: Frequency vector for the averaged spectra.
 * @retval None
 */
void process_avg_beam_spectra(
    fftw_complex *beamformed_spectra, 
    int avg_ratio,
    int num_samples, 
    int cur_beam,
    int beam_num, 
    int spectra_num,
    sample_meta_data *meta_data,
    double **avg_beam_spectra,
    double *avg_freq_vector,
    FILE *fft_file
) {
    log_debug("Entered process_avg_beam_spectra()...");

    // Frequency Vector and Spectral Avg Constants
    int num_avg_samples = num_samples / avg_ratio; 
    double delta_f = meta_data->usrp_rf_rate / num_samples;
    double f_start = meta_data->usrp_fcenter * 1000 - (meta_data->usrp_rf_rate / 2);

    // Determine Avg Freq Vector; used in Clear Freq Calculation
    if (avg_freq_vector == NULL) avg_freq_vector = calloc(sizeof(double), num_avg_samples);
    int delta_f_avg = delta_f * avg_ratio;
    for (int i = 0; i < num_avg_samples; i++) avg_freq_vector[i] = i * delta_f_avg + f_start;

    if (avg_beam_spectra == NULL) {
        avg_beam_spectra[cur_beam] = (double*) fftw_malloc(sizeof(double) * num_avg_samples);
        memset(avg_beam_spectra[cur_beam], 0, sizeof(double) * num_avg_samples);
    }

    log_trace("     num_avg_sample: %d avg_freq_ratio: %d", num_avg_samples, avg_ratio);
        

    clock_t t_avg_curr, t_avg;
    t_avg_curr = clock();

    // Spectral Averaging 
    // (0, 1, 2, 3 -> avg[0]), ..., (n-4, n-3, n-2, n-1 -> avg[n/4])
    // Process n/(4 * 20) averaged elements ...
    int s_idx = 0;
    for (int k = 0; k < num_avg_samples; k++) {
        avg_beam_spectra[cur_beam][k] = 0;

        // Across each descrete spectra stored by ... 
        for (int cur_spectra = 0; cur_spectra < spectra_num; cur_spectra++) {
            // Sum the magnitude of four spectrum samples in a row 
            for (int j = 0; j < avg_ratio; j++) {
                // Index using the following array format: [cur_spectra][cur_beam][k * avg_ratio + j]
                s_idx = cur_spectra * beam_num * num_samples + cur_beam * num_samples + k * avg_ratio + j;

                double re = creal(beamformed_spectra[s_idx]) * creal(beamformed_spectra[s_idx]);
                double im = cimag(beamformed_spectra[s_idx]) * cimag(beamformed_spectra[s_idx]);
                
                avg_beam_spectra[cur_beam][k] += sqrt(re + im);

                // if (k == 9) {
                //     log_trace("    spectra[%d]: %f + j%f", s_idx, creal(beamformed_spectra[s_idx]), cimag(beamformed_spectra[s_idx]));
                // }
            }   
        }

        // Div by the total elements summed (default: avg ratio of 4 * 20 3-sec spectras)
        avg_beam_spectra[cur_beam][k] /= (avg_ratio * spectra_num);
        if (k == 5) log_trace("         avg_spectra[%d][%d]: %f", cur_beam, k, avg_beam_spectra[cur_beam][k]);
    }
    t_avg = clock() - t_avg_curr;
    log_trace("     ====> Min Spectral Avg took (s): %lf", ((double) (t_avg)) / (CLOCKS_PER_SEC));
    log_trace("     beamformed_spectra[%d][0]: %f", cur_beam, avg_beam_spectra[cur_beam][0]);

    // Save data to csv
    if (access(SPECTRAL_LOG_FILE, F_OK) == 0) {        
                
        // log_warn("Logging for TCS has some bugs being worked out and has been disabled in the meantime.");
        // Write logs if its folder accessable
        if (BIN_OR_CSV_LOG == 0) {
            write_spectrum_mag_bin(
                &fft_file, 
                avg_beam_spectra[cur_beam], 
                avg_freq_vector, 
                num_avg_samples
            );
        } else {
            // write_sample_mag_csv(sample_im_file, sample_im, freq_vector, meta_data);      // Used to check complex Samples after Beamforming; ...
            // write_sample_mag_csv(sample_re_file, sample_re, freq_vector, meta_data);      // Plot w/ sample_plot.py
            write_spectrum_mag_csv(
                &fft_file,
                avg_beam_spectra[cur_beam], 
                avg_freq_vector, 
                num_avg_samples
            );  // Spectrum after Spectrum FFT averaging; plot w/ spectrum_plot.py
        }
        log_trace("[CFS] \'save_spectra\' found; Logged individual FFT Spectrum and Clear Frequency batches.");
    } else log_trace("[CFS] \'save_spectra\' not found. Not logging spectra nor clr_frequency.");
}

/**
 * @brief  Processes all beams clear frequency data.
 * @note   By DF
 * @param  **avg_beam_spectra: Averaged Beam-specific spectra to be processed.
 * @param  clear_freq_range: Frequency range for the clear frequencies.
 * @param  smsep: Sample separation in microseconds.
 * @param  delta_f_avg: Average frequency delta.
 * @param  num_avg_samples: Number of averaged samples.
 * @param  *restricted_bands: Array of restricted frequency bands.
 * @param  restricted_num: Number of restricted frequency bands.
 * @param  *avg_freq_vector: Frequency vector for the averaged spectra.
 * @param  num_avg_samples: Number of averaged samples.
 * @param  *meta_data: Metadata containing sample information.
 * @param  beam_num: Number of beams to process.
 * @param  *clr_band: The found clear frequency band.
 * @retval None
 */
void process_beam_clr_freq(
    double **avg_beam_spectra, 
    int cur_beam,
    int clear_freq_range[],
    int smsep,
    freq_band *restricted_bands, 
    int restricted_num,
    double *avg_freq_vector,
    int num_avg_samples,
    sample_meta_data *meta_data,
    freq_band *clr_band,
    FILE *clr_file
) {
    log_debug("Entered process_beam_clr_freq()...");

    double f_start = avg_freq_vector[0];
    int delta_f_avg = avg_freq_vector[1] - avg_freq_vector[0];

    // Mask restricted frequencies
    if (restricted_bands != NULL) mask_restricted_freq(avg_beam_spectra[cur_beam], avg_freq_vector, delta_f_avg, num_avg_samples, restricted_bands, restricted_num);
    log_trace("------f_start: %f -- f_end: %f", f_start/1000, avg_freq_vector[num_avg_samples - 1]/1000);

    log_trace("     num_avg_samples: %d", num_avg_samples);
    log_trace("     delta_f_avg: %d", delta_f_avg);

    // Define Clear Freq Range from Hz to sample index
    int clear_sample_start = (int) round((clear_freq_range[0] - f_start) / delta_f_avg);
    int clear_sample_end = (int) round((clear_freq_range[1] - f_start) / delta_f_avg);
    log_trace("     clear_range: | %d -- %d |", clear_freq_range[0]/1000, clear_freq_range[1]/1000);
    // log_trace("         samples: | %d -- %d |", clear_sample_start, clear_sample_end);
    // if (VERBOSE){ for (int i = clear_sample_start; i < clear_sample_end; i++) {
    //     // if (i < 2 + clear_sample_start || i > clear_sample_end - 3) 
    //     log_trace("spectrum_pow[%d]: %f", i, avg_spectrum[i]);   
    // }}

    // Clear Freq separation
    float gb = (1e6 * (GB_MULT - 1) ) / (smsep * 2);  // Dynamic Guardband based on sample separation
    if (gb < MIN_FREQ_SEP) gb = MIN_FREQ_SEP;       // Ensure minimum frequency separation
    float clear_bw = 1e6 / smsep + gb;          // Clear Bandwidth in Hz
    log_info("clear_bw: %f Hz = %f Hz (signal) + %f Hz (guard)", clear_bw, 1e6 / smsep, gb);
    
    // Find clear frequency
    clock_t t1, t2;
    t1 = clock();
    find_clear_freqs(avg_beam_spectra[cur_beam], *meta_data, delta_f_avg, clear_freq_range[0], clear_freq_range[1], clear_bw, clr_band);
    t2 = clock();
    log_trace("     find_clear_freqs(s): %lf", cur_beam, ((double) (t2 - t1)) / (CLOCKS_PER_SEC));

    // Save data to csv
    if (access(SPECTRAL_LOG_FILE, F_OK) == 0) {        
        char* avg_clr_freq_filename[256] = {0};
        sprintf(avg_clr_freq_filename, CLR_FREQ_FILE, "%s", "tcs.%s"); 

        // Write logs if its folder accessable
        if (BIN_OR_CSV_LOG == 0) {
            write_clr_freq_bin(
                &clr_file, 
                clr_band, 
                clear_freq_range
            );
        } else {
            write_clr_freq_csv( 
                &clr_file,
                clr_band, 
                clear_freq_range
            ); // Used to plot Clear Freq Bands w/ spectrum_plot.clr_freq.py
        }
        log_trace("[CFS] \'save_spectra\' found; Logged individual FFT Spectrum and Clear Frequency batches.");
    } else log_trace("[CFS] \'save_spectra\' not found. Not logging spectra nor clr_frequency.");
}

/**
 * @brief  Searches for clear frequency bands in the given raw samples.
 * @note   By DF
 * @param  *raw_samples: Raw samples to be processed.
 * @param  clear_freq_range: Frequency range to search for clear frequencies.
 * @param  cur_beam: Beam number for processing.
 * @param  smsep: Sample separation in microseconds.
 * @param  *restricted_bands: Array of restricted frequency bands.
 * @param  restrict_num: Number of restricted frequency bands.
 * @param  meta_data: Metadata containing sample information.
 * @param  *clr_band: The found clear frequency band.
 * @retval None
 */
clear_freq clear_freq_search(
        fftw_complex *raw_samples, 
        int *active_antennas,
        int clear_freq_range[],
        int cur_beam,
        int smsep,
        int avg_ratio,
        freq_band *restricted_bands, 
        int restrict_num,
        sample_meta_data meta_data,
        Config config,
        freq_band *clr_band
    ) {

    // Initial Data Variables
    int n_beams = config.array_info.nbeams;
    double beam_sep = config.array_info.beam_sep;

    // Scale Clear Freq Range from kHz to Hz
    if (clear_freq_range[0] < 100000 || clear_freq_range[1] < 100000) {
        clear_freq_range[0] = clear_freq_range[0] * 1000; 
        clear_freq_range[1] = clear_freq_range[1] * 1000;
    }

    // Beam Angle Calculation
    double beam_angle = calc_beam_angle(n_beams, cur_beam, beam_sep);  

    // Stopwatch Start
    double t1,t2;
    t1 = clock();

    // Find Clear Frequency Bands
    calc_clear_freq_on_raw_samples(
        raw_samples, 
        active_antennas,
        &meta_data, 
        restricted_bands, 
        restrict_num, 
        clear_freq_range, 
        beam_angle, 
        smsep, 
        avg_ratio,
        clr_band
    );
    
    // Print processing time; Stopwatch End
    t2 = clock();
    log_info("clear_freq_search (s): %lf", ((double) (t2 - t1)) / (CLOCKS_PER_SEC));
};

void process_avg_ant_pwr (
    fftw_complex *raw_samples, 
    int num_samples, 
    sample_meta_data *meta_data,
    int *muted_config_ants,
    int num_muted_config_ants,
    int *ant_active_ct,
    int *active_antennas,
    long *acculated_pwrs
) {
    double avg_pwrs[meta_data->num_antennas];
    double overall_avg_pwr = 0;

    log_trace("Entered process_avg_ant_pwr()...");
    // Average antenna powers from all samples
    int s_idx = 0;
    for (int ant_idx = 0; ant_idx < meta_data->num_antennas; ant_idx++) {     
        avg_pwrs[ant_idx] = 0;

        // Average Power of antenna's sample set
        for (int cur_sample = 0; cur_sample < num_samples; cur_sample++) {

            // Index samples using the following array format: [cur_ant][cur_sample]
            s_idx = ant_idx * num_samples + cur_sample;
            
            // Calculate Power of each sample ... 
            double re = creal(raw_samples[s_idx]) * creal(raw_samples[s_idx]);
            double im = cimag(raw_samples[s_idx]) * cimag(raw_samples[s_idx]);
            avg_pwrs[ant_idx] += re + im; // sqrt(re + im);
            
            // if (cur_sample % 250 == 0) log_trace("ant_idx: %d  cur_sample: %d", ant_idx, cur_sample);
            // log_trace("    spectra[%d]: %f + j%f", s_idx, creal(raw_samples[s_idx]), cimag(raw_samples[s_idx]));
            // log_trace("         avg_ant_pwr[%d]: %f", ant_idx, avg_pwrs[ant_idx]);
        }
        avg_pwrs[ant_idx] /= num_samples;
        overall_avg_pwr += avg_pwrs[ant_idx];

        // Accumulate the average power into the sum of averaged powers
        acculated_pwrs[meta_data->antenna_list[ant_idx]] += avg_pwrs[ant_idx];
    }

    overall_avg_pwr /= meta_data->num_antennas;
    double min_pwr_threshold = 1;
    if (overall_avg_pwr > 0) {
        min_pwr_threshold = overall_avg_pwr * MIN_ANT_PWR_MULT;
    } else {
        log_error("ERROR: CFS shows antennas are all down! CFS will resort to last clear freq set.");
    }
    log_debug("overall_avg_pwr: %f min_pwr_threshold: %f", overall_avg_pwr, min_pwr_threshold);

    // Filter out active vs inactive antennas
    for (int ant_idx = 0; ant_idx < meta_data->num_antennas; ant_idx++) {
    
        // Check if muted in Array Config
        bool is_muted = false;
        for (int config_i = 0; config_i < num_muted_config_ants; config_i++) {
            if (muted_config_ants[config_i] == ant_idx) {
                log_debug("         Antenna[%2d]   muted: pwr = %f", meta_data->antenna_list[ant_idx], avg_pwrs[ant_idx]);
                is_muted = true;
                break;
            }
        }          
        if (is_muted == true) continue;

        if (USE_ACTIVE_MUTE == 1) {
            // Check if antenna meets active pwr threshold, ...
            if (avg_pwrs[ant_idx] >= min_pwr_threshold && 
                ( meta_data->antenna_list[ant_idx] <= IDX_LAST_MA || meta_data->antenna_list[ant_idx] > IDX_LAST_IA)) {
                log_debug("         Antenna[%2d]   active: pwr = %f", meta_data->antenna_list[ant_idx], avg_pwrs[ant_idx]);
                ant_active_ct[meta_data->antenna_list[ant_idx]]++;      // Increment # of times ant was active
                active_antennas[meta_data->antenna_list[ant_idx]] = 1;  // Mark antenna as active
            } 
            // If Inferrometric antennas meets active pwr threshold, ...
            else if (avg_pwrs[ant_idx] >= min_pwr_threshold && 
                (meta_data->antenna_list[ant_idx] > IDX_LAST_MA && meta_data->antenna_list[ant_idx] <= IDX_LAST_IA)) {
                log_debug("         Antenna[%2d]   inferr: pwr = %f", meta_data->antenna_list[ant_idx], avg_pwrs[ant_idx]);
                ant_active_ct[meta_data->antenna_list[ant_idx]]++;      // Increment # of times ant was active
                active_antennas[meta_data->antenna_list[ant_idx]] = 1;  // Mark antenna as active
            }
            else {
                log_trace("         Antenna[%2d] inactive: pwr = %f ", meta_data->antenna_list[ant_idx], avg_pwrs[ant_idx]);
            }
        }

        // Only mute if not present/empty antenna data to protect clear freqs
        else {
            // Check if antenna is present, ...
            if (avg_pwrs[ant_idx] > 0 && 
                ( meta_data->antenna_list[ant_idx] <= IDX_LAST_MA || meta_data->antenna_list[ant_idx] > IDX_LAST_IA)) {
                log_debug("         Antenna[%2d]   active: pwr = %f", meta_data->antenna_list[ant_idx], avg_pwrs[ant_idx]);
                ant_active_ct[meta_data->antenna_list[ant_idx]]++;      // Increment # of times ant was active
                active_antennas[meta_data->antenna_list[ant_idx]] = 1;  // Mark antenna as active
            } 
            // If Inferrometric antennas is present, ...
            else if (avg_pwrs[ant_idx] > 0 && 
                (meta_data->antenna_list[ant_idx] > IDX_LAST_MA && meta_data->antenna_list[ant_idx] <= IDX_LAST_IA)) {
                log_debug("         Antenna[%2d]   inferr: pwr = %f", meta_data->antenna_list[ant_idx], avg_pwrs[ant_idx]);
                ant_active_ct[meta_data->antenna_list[ant_idx]]++;      // Increment # of times ant was active
                active_antennas[meta_data->antenna_list[ant_idx]] = 1;  // Mark antenna as active
            }
            else {
                log_trace("         Antenna[%2d] inactive: pwr = %f ", meta_data->antenna_list[ant_idx], avg_pwrs[ant_idx]);
            }
        }
    }
};
