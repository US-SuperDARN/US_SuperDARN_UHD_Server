#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <complex.h>
#include <fftw3.h>      // FFT transform library
#include <string.h>
#include <time.h>
#include "clear_freq_search.h"
#include <ctype.h>
#include <sys/stat.h>
#include "log.h"

void file_access_error(const char *filepath) {
    log_error("[ERROR: accessing filepath: %s\n", filepath);
    perror("Error");
    exit(EXIT_FAILURE);
}


void add_ptr_no_global(void **ptr, void **temp_ptrs, int *temp_ptrs_num) {
    // Check for pre-existing ptr
    for (int i = 0; i < *temp_ptrs_num; i++) {
        if (temp_ptrs[i] == ptr) {
            log_trace("Pointer already exists in temp_ptrs[%d]: %p\n", i, temp_ptrs[i]);
            return;
        }
    }

    void *tmp = NULL;
    tmp = realloc(temp_ptrs, (*temp_ptrs_num + 1) * sizeof(void *));
    if (tmp == NULL) {
        perror("Error reallocating memory for temp_ptrs");
        exit(EXIT_FAILURE);
    }
    free(temp_ptrs);
    temp_ptrs = tmp;
    
    temp_ptrs[*temp_ptrs_num] = ptr; // Store the address of the pointer
    log_trace("  temp_ptrs[%d] = %p -> %p\n", *temp_ptrs_num, temp_ptrs[*temp_ptrs_num], *(void **)temp_ptrs[*temp_ptrs_num]);
    temp_ptrs_num++;
    
}

void update_ptr_no_global(void *old_ptr, void *new_ptr, void** temp_ptrs, int temp_ptrs_num) {
    // Check if the old pointer exists in the array
    for (int i = 0; i < temp_ptrs_num; i++) {
        if (temp_ptrs[i] == old_ptr) {
            log_trace("Updating temp_ptrs[%d] from %p to %p\n", i, temp_ptrs[i], new_ptr);
            temp_ptrs[i] = new_ptr;
            return; // Exit the function once the pointer is found and updated
        }
    }
    
    // If the old pointer is not found, add the new pointer to the array
    add_ptr_no_global((void **)&new_ptr, temp_ptrs, &temp_ptrs_num);
}


/**
 * @brief  Writes a complex Frequency Spectrum to csv file to be plotted in python.
 * @note   By DF
 * @param  *filename:       The filepath for the saved CSV file
 * @param  *spectrum:       Frequency Spectrum in complex form
 * @param  *freq_vector:    Array for Frequency (steps of frequency per sample)
 * @param  num_samples:     Number of four_spectrums collected per antenna
 * @retval None
 * @deprecated Replaced by write_spectrum_mag_csv. No longer using complex form to similify calculations
 */
void write_spectrum_csv(char *filename, fftw_complex *spectrum, double *freq_vector, int num_samples) {
    // Timestamp Variables
    time_t raw_time;
    struct tm *time_info;
    int buffer_size = 100;
    char timestamp[buffer_size];
    char name[buffer_size]; 

    // Generate timestamp
    time(&raw_time);
    time_info = gmtime(&raw_time);
    strftime(timestamp, buffer_size, "%Y.%m.%d_%H:%M:%S", time_info);
    snprintf(name, sizeof(name), filename, timestamp, "csv");

    FILE *file = fopen(name, "w");
    if (file == NULL) {
        file_access_error(name);
        return;
    }
    fprintf(file, "Frequency,Power\n");
    for (int i = 0; i < num_samples; i++) {
        double magnitude = sqrt(creal(spectrum[i]) * creal(spectrum[i]) + cimag(spectrum[i]) * cimag(spectrum[i]));
        fprintf(file, "%f,%f\n", freq_vector[i], magnitude);
    }

    fclose(file);
}

/**
 * @brief  Writes a magnitude Frequency Spectrum to csv file to be plotted in python.
 * @note   By DF
 * @param  *filename:       The filepath for the saved CSV file
 * @param  *spectrum:       Frequency Spectrum in magnitude form
 * @param  *freq_vector:    Array for Frequency (steps of frequency per sample)
 * @param  num_samples:     Number of samples in spectrum
 * @retval None
 */
void write_spectrum_mag_csv(char *filename, double *spectrum, double *freq_vector, int num_samples) {
    // Timestamp Variables
    time_t raw_time;
    struct tm *time_info;
    int buffer_size = 100;
    char timestamp[buffer_size];
    char name[buffer_size]; 

    // Generate timestamp
    time(&raw_time);
    time_info = gmtime(&raw_time);
    strftime(timestamp, buffer_size, "%Y.%m.%d_%H:%M:%S", time_info);
    snprintf(name, sizeof(name), filename, timestamp, "csv");

    FILE *file = fopen(name, "w");
    if (file == NULL) {
        file_access_error(name);
        return;
    }
    fprintf(file, "Frequency,Power\n");
    for (int i = 0; i < num_samples; i++) {
        fprintf(file, "%f,%f\n", freq_vector[i], spectrum[i]);
    }

    fclose(file);
}

void write_spectrum_mag_bin(char *filename, double *spectrum, double *freq_vector, int num_samples) {
    // Timestamp Variables
    time_t raw_time;
    struct tm *time_info;
    int buffer_size = 100;
    char timestamp[buffer_size];
    char name[buffer_size]; 

    // Generate timestamp
    time(&raw_time);
    time_info = gmtime(&raw_time);
    strftime(timestamp, buffer_size, "%Y.%m.%d_%H:%M:%S", time_info);
    snprintf(name, sizeof(name), filename, timestamp, "bin");

    FILE *file = fopen(name, "wb");
    if (file == NULL) {
        file_access_error(name);
        return;
    }

    fwrite(&num_samples, sizeof(int), 1, file);
    fwrite(freq_vector, sizeof(double), num_samples, file);
    fwrite(spectrum, sizeof(double), num_samples, file);


    log_trace("  ********************************************   Bytes of spectrum_mag: %ld, %ld, %ld\n", sizeof(num_samples), sizeof(freq_vector), sizeof(spectrum));
    fclose(file);
}


void write_clr_freq_csv(char *filename, freq_band *clr_bands) {
    // Timestamp Variables
    time_t raw_time;
    struct tm *time_info;
    int buffer_size = 100;
    char timestamp[buffer_size];
    char name[buffer_size]; 

    // Generate timestamp
    time(&raw_time);
    time_info = gmtime(&raw_time);
    strftime(timestamp, buffer_size, "%Y.%m.%d_%H:%M:%S", time_info);
    snprintf(name, sizeof(name), filename, timestamp, "csv");
    
    FILE *file = fopen(filename, "w");
    if (file == NULL) {
        file_access_error(name);
        return;
    }

    // Find Start and End of Clear Freq Range to Write later
    int clr_start = RAND_MAX;
    int clr_end = 0;
    for (int i = 0; i < CLR_BANDS_MAX; i++) {
        if (clr_bands[i].f_start < clr_start && clr_bands[i].noise < RAND_MAX) clr_start = clr_bands[i].f_start;
        if (clr_bands[i].f_end > clr_end && clr_bands[i].noise < RAND_MAX) clr_end = clr_bands[i].f_end;
    }    

    fprintf(file, "Start Frequency,End Frequency,Noise,Clear Freq Start,Clear Freq End\n");
    for (int i = 0; i < CLR_BANDS_MAX; i++) {
        // Special: Print Clear Freq Range on Line 0
        if (i == 0) fprintf(file, "%d,%d,%f,%d,%d\n", clr_bands[i].f_start, clr_bands[i].f_end, clr_bands[i].noise,clr_start,clr_end);
        else fprintf(file, "%d,%d,%f\n", clr_bands[i].f_start, clr_bands[i].f_end, clr_bands[i].noise);
    }

    fclose(file);
}

void write_clr_freq_bin(char *filename, freq_band *clr_bands) {
    // Timestamp Variables
    time_t raw_time;
    struct tm *time_info;
    int buffer_size = 100;
    char timestamp[buffer_size];
    char name[buffer_size]; 

    // Generate timestamp
    time(&raw_time);
    time_info = gmtime(&raw_time);
    strftime(timestamp, buffer_size, "%Y.%m.%d_%H:%M:%S", time_info);
    snprintf(name, sizeof(name), filename, timestamp, "bin");

    FILE *file = NULL;
    file = fopen(name, "wb");
    if (file == NULL) {
        file_access_error(name);
        return;
    }

    // Find Start and End of Clear Freq Range to Write later
    int clr_start = RAND_MAX;
    int clr_end = 0;
    for (int i = 0; i < CLR_BANDS_MAX; i++) {
        if (clr_bands[i].f_start < clr_start && clr_bands[i].noise < RAND_MAX) clr_start = clr_bands[i].f_start;
        if (clr_bands[i].f_end > clr_end && clr_bands[i].noise < RAND_MAX) clr_end = clr_bands[i].f_end;
    }    

    fwrite(&clr_start, sizeof(int), 1, file);
    fwrite(&clr_end, sizeof(int), 1, file);
    fwrite(clr_bands, sizeof(freq_band), CLR_BANDS_MAX, file);

    fclose(file);
}

/**
 * @brief  Writes the Real/Imaginary magnitude to csv file to be plotted in python.
 * @note   By DF
 * @param  *filename:           The filepath for the saved CSV file
 * @param  *raw_samples_mag:    Int array of the Real/Imaginary magnitude
 * @param  *freq_vector:        Array for Frequency (steps of frequency per sample)
 * @param  num_samples:         Number of four_spectrums collected per antenna
 * @retval None
 */
void write_sample_mag_csv(char *filename, int **raw_samples_mag, double *freq_vector, sample_meta_data *meta_data) {
    FILE *file = NULL;
    file = fopen(filename, "w");
    if (file == NULL) {
        file_access_error(filename);
        return;
    }

    fprintf(file, "Samples,Power\n");
    for (int j = 0; j < meta_data->num_antennas; j++) {
        for (int i = 0; i < meta_data->number_of_samples; i++) {
            fprintf(file, "%f,%d\n", freq_vector[i], raw_samples_mag[j][i]);
        }
    }
    fclose(file);
}

void read_spectrum_mag_bin(char *filename, double *spectrum, double *freq_vector) {
    int num_samples = 0;
    
    FILE *file = NULL;
    file = fopen(filename, "rb");
    if (file == NULL) {
        file_access_error(filename);
        return;
    }

    fread(&num_samples, sizeof(num_samples), 1, file);
    fread(spectrum, sizeof(double), num_samples, file);
    fread(freq_vector, sizeof(double), num_samples, file);

    fclose(file);
}

void read_clr_freq_bin(char *filename, freq_band *clr_bands, int *clr_start, int *clr_end) {
    FILE *file = NULL;
    file = fopen(filename, "rb");
    if (file == NULL) {
        file_access_error(filename);
        return;
    }

    fread(clr_start, sizeof(clr_start), 1, file);
    fread(clr_end, sizeof(int), 1, file);
    fread(clr_bands, sizeof(freq_band), 1, file);

    fclose(file);
}


/**
 * @brief  Reads intial Clear Freq parameters from a converted txt file (see utils/pickle_text_convert.py)
 * @note   
 * @param  *filename:           Filepath of converted txt file
 * @param  *meta_data:          Meta data for current sample batch
 * @param  **clear_freq_range:  Range for the Clear Freq
 * @param  ***raw_samples:      14x2500 complex sample array
 * @retval None
//  */
// void read_input_data(const char *filename, sample_meta_data *meta_data, double **clear_freq_range, fftw_complex ***raw_samples, int test_clr_range, int test_samples) {
//     FILE *file = fopen(filename, "r");
//     if (file == NULL) {
//         file_access_error(filename);
//         exit(EXIT_FAILURE);
//     }
// 
//     char line[256];
//     int antenna_list_size = 0;
// 
//     while (fgets(line, sizeof(line), file)) {
//         if (sscanf(line, "number_of_samples: %d", &meta_data->number_of_samples) == 1) continue;
//         if (sscanf(line, "usrp_rf_rate: %d", &meta_data->usrp_rf_rate) == 1) continue;
//         // if (sscanf(line, "usrp_fcenter: %d", &meta_data->usrp_fcenter) == 1) continue;
//         // if (sscanf(line, "x_spacing: %lf", &meta_data->x_spacing) == 1) continue;
//         // if (strncmp(line, "clear_freq_range:", 15) == 0 && test_clr_range) {
//         //     clear_freq_range = realloc(clear_freq_range, 2 * sizeof(double));
//         //     int i = 0;
//         //     char *token = strtok(line + 16, ",");
//         //     while (token != NULL) {
//         //         (*clear_freq_range)[i] = atof(token);
//         //         i++;
//         //     }
//         //     continue;
//         // }
// 
//         // Antenna List Data
//         if (strncmp(line, "antenna_list:", 13) == 0) {
//             char *token = strtok(line + 14, ",");
//             while (token != NULL) {
//                 // Remove any leading or trailing whitespace from token
//                 while (isspace(*token)) token++;
//                 char *end = token + strlen(token) - 1;
//                 while (end > token && isspace(*end)) end--;
//                 *(end + 1) = '\0';
// 
//                 meta_data->antenna_list = realloc(meta_data->antenna_list, (++antenna_list_size) * sizeof(int));
//                 meta_data->antenna_list[antenna_list_size - 1] = atoi(token);
// 
//                 token = strtok(NULL, ",");
//             }
//             meta_data->num_antennas = antenna_list_size;
//             continue;
//         }
// 
//         // Raw Sample Data
//         // if (strncmp(line, "raw_samples:", 12) == 0 && test_samples) {
//         //     printf("[Clear Freq Search] Aquiring test four_spectrums from pickle files...\n");
//         //     // Allocate mem
//         //     *raw_samples = (fftw_complex **)fftw_malloc(meta_data->num_antennas * sizeof(fftw_complex *));
//         //     for (int i = 0; i < meta_data->num_antennas; i++) {
//         //         (*raw_samples)[i] = (fftw_complex *)fftw_malloc(meta_data->number_of_samples * sizeof(fftw_complex));
//         //     }
//         //     if (*raw_samples == NULL) {
//         //         perror("Error allocating memory for raw four_spectrums");
//         //         exit(EXIT_FAILURE);
//         //     }
//  
//         //     // Store data
//         //     for (int i = 0; i < meta_data->num_antennas; i++) {
//         //         fftw_complex *ant_samples = (*raw_samples)[i];
// 
//         //         for (int j = 0; j < meta_data->number_of_samples; j++) {
//         //             double real, imag;
//         //             fgets(line, sizeof(line), file);
//         //             sscanf(line, "%lf,%lf", &real, &imag);
//  
//         //             ant_samples[j] = real + I * imag;
//         //         }
//         //     }
//         //     break;
//         // }
//     }
// 
//     fclose(file);
// }

void read_restrict(char *filepath, freq_band *restricted_freq, int *restricted_num) {
    FILE *file = fopen(filepath, "r");
    if (file == NULL) {
        file_access_error(filepath);
        exit(EXIT_FAILURE);
    }

    char line[256];
    int r1 = 0;
    int r2 = 0;
    int i = 0;

    while (fgets(line, sizeof(line), file)) {
        sscanf(line, "%d %d", &r1, &r2);

        // Skip non-frequency band lines
        if (r1 == 0 || r2 == 0) continue;
        else {
            // log_trace("Storing r1 & r2...\n");

            // Check for valid freq
            if (r1  < r2 && r1 > 0 && r2 > 0) {
                // Store freq band
                restricted_freq[i].f_start  = r1 * 1000;
                restricted_freq[i].f_end    = r2 * 1000; 
                log_trace("Restricted[%d]: %d -- %d", i, restricted_freq[i].f_start, restricted_freq[i].f_end);
                i++;
            } 
            
            else {
                log_trace("Invalid freq band: %d -- %d", r1, r2);
                continue;
            }
        }
    }
    
    *restricted_num = i;    
    log_trace("Number of restricted bands: %d", *restricted_num);
    
    fclose(file);
}

void read_radar_config(char *filepath, int *clr_freq_res) {
    FILE *file = fopen(filepath, "r");
    if (file == NULL) {
        file_access_error(filepath);
        exit(EXIT_FAILURE);
    }

    char line[256];
    char word[256] = {"\0"};
    int value = 0;
    int i = 0;
    int result = 0;

    while (fgets(line, sizeof(line), file)) {
        result = sscanf(line, "%s = %d", &word, &value);
        word[11] = '\0'; // Ensure null-termination     

        // Debug: Print word and value
        // log_trace("Read line: %s, word: %s, value: %d", line, word, value);

        // If sscanf finds CLR_FREQ_RES, store it
        if (strcmp(word, "CFSFREQ_RES\0") == 0) {
            *clr_freq_res = value;
            log_trace("CFSFREQ_RES: %d", *clr_freq_res);
            break;
        }
    }

    // Warn if low frequency resolution (can result in corrupted clr freq bands)
    if (*clr_freq_res <= 0) {
        log_error("CFSFREQ_RES is not set or is invalid (%d <= 0). Please check radar_config_constants.py file.", *clr_freq_res);
    }
    else if (*clr_freq_res < 500) {
        log_warn("CFSFREQ_RES is extremely low (%d < 500). This can result in corrupted clear frequency bands!", *clr_freq_res);
        log_warn("Please check radar_config_constants.py file.");
    } 

    fclose(file);
}

void get_timestamp( char* buffer){
	time_t rawtime;
	struct tm *timeinfo;
	time(&rawtime);
	timeinfo = gmtime (&rawtime);
	strftime (buffer,32,"%G.%m%d.%H%M%S",timeinfo);
}

void get_file_name(char* filename, char* filepath){
    char timestamp[32];
    get_timestamp(timestamp);
    sprintf(filename, filepath, timestamp);
}

FILE* get_log_file( char *filepath) {
    // Create logs directory if it doesn't exist
    struct stat st = {0};
	if (stat("log/", &st) == -1) {
		mkdir("log", 0700);
	}
    if (stat("log/cfs", &st) == -1) {
        mkdir("log/cfs", 0700);
    }

    char filename[128];
    get_file_name(filename, filepath);

    // Create log file with timestamp
	FILE* file = NULL;
	if (file == NULL){
		file = fopen(filename, "w");
	}
    return file;
}

FILE* init_log(int level, int file_level, char *filepath) {
    log_set_level(level);
	log_set_quiet(0);
	FILE *file = get_log_file(filepath);
	int result = log_add_fp(file, 0);
	if (result == 0){
		log_info("CFS Logger initialized...\n");
		return file;
	}
	perror("Failed to initialize Log File");
	exit(EXIT_FAILURE);
}
