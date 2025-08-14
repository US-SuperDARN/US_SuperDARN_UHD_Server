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
#include <inttypes.h>

void file_access_error(const char *filepath) {
    log_error("ERROR: accessing filepath: %s\n", filepath);
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

void gen_filename(char *name_template, char *ext, char *name) {
    time_t raw_time;
    struct tm *time_info;
    int buffer_size = 192;
    char timestamp[buffer_size];

    // Generate timestamp
    time(&raw_time);
    time_info = gmtime(&raw_time);
    strftime(timestamp, buffer_size, "%Y.%m.%d_%H:%M:%S", time_info);
    snprintf(name, buffer_size, name_template, timestamp, ext);
    log_trace("generated filename: %s", name);
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
void write_spectrum_mag_csv(
    FILE **file,
    double *spectrum, 
    double *freq_vector, 
    int num_samples
) {
    // Timestamp Variables
    time_t raw_time;
    struct tm *time_info;
    int buffer_size = 100;
    char timestamp[buffer_size];
    char name[buffer_size]; 

    // If file doesn't exists, ... 
    if (*file == NULL) {
        // Generate timestamp
        time(&raw_time);
        time_info = gmtime(&raw_time);
        strftime(timestamp, buffer_size, "%Y.%m.%d_%H:%M:%S", time_info);
        snprintf(name, sizeof(name), SPECTRUM_FILE, timestamp, "csv");
        
        *file = fopen(name, "w");
        if (*file == NULL) {
            file_access_error(name);
            return;
        }

        fprintf(*file, "Frequency,Power\n");
    }

    __uint64_t t = (__uint64_t) time(NULL); // Restrict bytes 
    // (&t, sizeof(__uint64_t), 1, *file);
    
    for (int i = 0; i < num_samples; i++) {
        if (i == 0) fprintf(*file, "%f,%f,%" PRId64 "\n", freq_vector[i], spectrum[i]);
        fprintf(*file, "%f,%f\n", freq_vector[i], spectrum[i]);
    }

    fflush(*file);
}

void write_spectrum_mag_bin(
    FILE **file,
    double *spectrum, 
    double *freq_vector, 
    int num_samples
) {
    // Timestamp Variables
    time_t raw_time;
    struct tm *time_info;
    int buffer_size = 100;
    char timestamp[buffer_size];
    char name[buffer_size]; 

    log_trace("file pointer: %p", (void*)file);
    log_trace("file pointer: %p", file); 

    // If file doesn't exists, ... 
    if (*file == NULL) {
        // Generate timestamp
        time(&raw_time);
        time_info = gmtime(&raw_time);
        strftime(timestamp, buffer_size, "%Y.%m.%d_%H:%M:%S", time_info);
        snprintf(name, sizeof(name), SPECTRUM_FILE, timestamp, "bin");
        
        *file = fopen(name, "wb");
        if (*file == NULL) {
            file_access_error(name);
            return;
        }

        // Only write number of samples once per file
        log_trace("num_samples: %d",num_samples);
        fwrite(&num_samples, sizeof(int), 1, *file);
    }

    __uint64_t t = (__uint64_t) time(NULL); // Restrict bytes 

    fwrite(&t, sizeof(__uint64_t), 1, *file);
    fwrite(freq_vector, sizeof(double), num_samples, *file);
    fwrite(spectrum, sizeof(double), num_samples, *file);

    log_trace("  ********************************************   Bytes of spectrum_mag: %ld, %ld, %ld\n", sizeof(num_samples), sizeof(freq_vector), sizeof(spectrum));
    fflush(*file);
}


void write_clr_freq_csv(
    FILE **file,
    freq_band *clr_bands, 
    int *clr_range
) {
    // Timestamp Variables
    time_t raw_time;
    struct tm *time_info;
    int buffer_size = 100;
    char timestamp[buffer_size];
    char name[buffer_size]; 

    // If file doesn't exists, ... 
    if (*file == NULL) {
        // Generate timestamp
        time(&raw_time);
        time_info = gmtime(&raw_time);
        strftime(timestamp, buffer_size, "%Y.%m.%d_%H:%M:%S", time_info);
        snprintf(name, sizeof(name), CLR_FREQ_FILE, timestamp, "csv");
        
        *file = fopen(name, "w");
        if (*file == NULL) {
            file_access_error(name);
            return;
        }
        fprintf(*file, "Start Frequency,End Frequency,Noise,Clear Freq Start,Clear Freq End\n");
    }

    // Write Clear Search Range on first line of each sample set only
    for (int i = 0; i < CLR_BANDS_MAX; i++) {
        // Special: Print Clear Freq Range on Line 0
        if (i == 0) fprintf(*file, "%d,%d,%f,%d,%d\n", clr_bands[i].f_start, clr_bands[i].f_end, clr_bands[i].noise,clr_range[0],clr_range[1]);
        else fprintf(*file, "%d,%d,%f\n", clr_bands[i].f_start, clr_bands[i].f_end, clr_bands[i].noise);
    }
    
    fflush(*file);
}

void write_clr_freq_bin(
    FILE **file,
    freq_band *clr_bands, 
    int* clr_range
) {
    // Timestamp Variables
    time_t raw_time;
    struct tm *time_info;
    int buffer_size = 100;
    char timestamp[buffer_size];
    char name[buffer_size]; 
    

    // If file doesn't exists, ... 
    if (*file == NULL) {
        // Generate timestamp
        time(&raw_time);
        time_info = gmtime(&raw_time);
        strftime(timestamp, buffer_size, "%Y.%m.%d_%H:%M:%S", time_info);
        snprintf(name, sizeof(name), CLR_FREQ_FILE, timestamp, "bin");
        
        *file = fopen(name, "wb");
        if (*file == NULL) {
            file_access_error(name);
            return;
        }
    }

    // Write clear search range, then clear bands
    fwrite(&(clr_range[1]), sizeof(int), 1, *file);
    fwrite(&(clr_range[0]), sizeof(int), 1, *file);
    for (int i = 0; i < CLR_BANDS_MAX; i ++) {
        // int noise = (int) clr_bands[i].noise;
        fwrite(&(clr_bands[i].f_start), sizeof(int), 1, *file);
        fwrite(&(clr_bands[i].noise), sizeof(double), 1, *file);
        fwrite(&(clr_bands[i].f_end), sizeof(int), 1, *file);

        log_trace("    clr band[%d]: | %dMHz -- Noise: %f -- %dMHz |", 
            i, 
            clr_bands[i].f_start, 
            clr_bands[i].noise, 
            clr_bands[i].f_end
        );  
    }
    
    fflush(*file);
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

void read_radar_config(char *filepath, int *avg_ratio) {
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
        // if (strcmp(word, "CFSFREQ_RES\0") == 0) {
        //     *clr_freq_res = value;
        //     log_trace("CFSFREQ_RES: %d", *clr_freq_res);
        //     break;
        // }

        if (strcmp(word, "AVG_RATIO") == 0) {
            *avg_ratio = value;
            log_trace("AVG_RATIO: %d", *avg_ratio);
            break;
        }
    }

    // Warn if low frequency resolution (can result in corrupted clr freq bands)
    if (*avg_ratio <= 0) {
        log_error("avg_ratio is invalid (%d <= 0). Please check radar_config_constants.py file.", *avg_ratio);
    }
    else if (*avg_ratio > 5) {
        log_warn("avg_ratio is high (%d > 5). This can result in corrupted clear frequency bands!", *avg_ratio);
        log_warn("Please check radar_config_constants.py file.");
    } 

    fclose(file);
}

void get_timestamp( char* buffer){
	time_t rawtime;
	struct tm *timeinfo;
	time(&rawtime);
	timeinfo = gmtime (&rawtime);
	strftime(buffer,32,"%G.%m%d.%H%M%S",timeinfo);
}

void get_file_name(char* filename, char* filepath){
    char timestamp[32];
    get_timestamp(timestamp);
    sprintf(filename, filepath, timestamp);
}

FILE* get_log_file( char *filepath) {
    // Create logs directory if it doesn't exist
    struct stat st = {0};
	if (stat("/data/log/", &st) == -1) {
		mkdir("/data/log", 0700);
	}
    if (stat("/data/log/cfs", &st) == -1) {
        mkdir("/data/log/cfs", 0700);
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
