#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <semaphore.h>
#include <math.h>
#include <complex.h>
#include <fftw3.h>      // FFT transform library
#include <signal.h>
#include <time.h>
#include "clear_frequency_server.h"
#include "clear_freq_search.h"
#include "ini_parser.h"
#include "read_config.c"
#include "log.h"
#include "misc_read_writes.h"

// Build with the following flags:
// -lrt -pthread -lfftw3 -lm 


typedef struct shm_obj{
    const char* name;
    void* shm_ptr;
    int shm_fd;
    size_t size;
} shm_obj;

typedef struct semaphore {
    const char* name;
    sem_t* sem;
} semaphore;




// Semaphores Locks prevent race conditions
// Semaphore Flags allow client and server to signal specific data transfers
struct semaphore sf_client   = {SEM_F_CLIENT,       NULL};
struct semaphore sf_server   = {SEM_F_SERVER,       NULL};
struct semaphore sf_samples  = {SEM_F_SAMPLES,      NULL};
struct semaphore sf_init     = {SEM_F_INIT,         NULL};
struct semaphore sf_clrfreq  = {SEM_F_CLRFREQ,      NULL};
struct semaphore sf_processed= {SEM_F_PROCESSED,    NULL};
struct semaphore sl_samples  = {SEM_L_SAMPLES,      NULL};
struct semaphore sl_init     = {SEM_L_INIT,         NULL};
struct semaphore sl_clrfreq  = {SEM_L_CLRFREQ,      NULL};
struct semaphore *semaphores[SEM_NUM] = {
    &sf_client,
    &sf_server,    
    &sf_samples,
    &sf_init,
    &sf_clrfreq,
    &sf_processed,
    &sl_samples,
    &sl_init,
    &sl_clrfreq,
};

shm_obj samples_obj     = {SAMPLES_SHM_NAME, NULL, -1, SAMPLES_SHM_SIZE};
shm_obj clr_range_obj   = {CLR_RANGE_SHM_NAME, NULL, -1, CLR_RANGE_SHM_SIZE};
shm_obj fcenter_obj     = {FCENTER_SHM_NAME, NULL, -1, FCENTER_SHM_SIZE};
shm_obj beam_num_obj    = {BEAM_NUM_SHM_NAME, NULL, -1, BEAM_NUM_SHM_SIZE};
shm_obj sample_sep_obj  = {SAMPLE_SEP_SHM_NAME, NULL, -1, SAMPLE_SEP_SHM_SIZE};
shm_obj restrict_obj    = {RESTRICT_SHM_NAME, NULL, -1, RESTRICT_SHM_SIZE};
shm_obj meta_obj        = {META_DATA_SHM_NAME, NULL, -1, META_DATA_SHM_SIZE};
shm_obj antenna_obj     = {ANTENNA_SHM_NAME, NULL, -1, ANTENNA_SHM_SIZE};
shm_obj clrfreq_obj     = {CLRFREQ_SHM_NAME, NULL, -1, CLR_BAND_SHM_SIZE};
shm_obj site_id_obj     = {SITE_ID_SHM_NAME, NULL, -1, SITE_ID_SHM_SIZE};
shm_obj radar_id_obj    = {RADAR_ID_SHM_NAME, NULL, -1, RADAR_ID_SHM_SIZE};
shm_obj channel_id_obj  = {CHANNEL_ID_SHM_NAME, NULL, -1, CHANNEL_ID_SHM_SIZE};
shm_obj client_num_obj  = {ACTIVE_CLIENTS_SHM_NAME, NULL, -1, ACTIVE_CLIENTS_SHM_SIZE};
shm_obj muted_ant_obj   = {MUTED_ANT_SHM_NAME, NULL, -1, MUTED_ANT_SHM_SIZE};
struct shm_obj *objects[PARAM_NUM] = {
    &samples_obj,
    &clr_range_obj,
    &fcenter_obj,
    &beam_num_obj,
    &sample_sep_obj,
    &restrict_obj,
    &meta_obj,
    &antenna_obj,
    &clrfreq_obj,
    &site_id_obj,
    &radar_id_obj,
    &channel_id_obj,
    &client_num_obj,
    &muted_ant_obj,
};


int temp_ptrs_num = 0;
void **temp_ptrs;

fftw_complex *temp_samples = NULL;
fftw_complex *spectra_storage = NULL;

double **avg_beam_spectrum = NULL;
int avg_beam_spectrum_sizes[] = {
    BEAM_NUM,
    SAMPLES_NUM / AVG_RATIO,
};
double *avg_freq_vector = NULL;
freq_band ****clr_band_storage = NULL;
int clr_storage_sizes[] = {
    1,
    STATIC_CHANNEL_NUM,
    CLR_STORAGE_NUM,
};
sample_meta_data meta_data = {0};
radar_freq_data **radar_table = NULL;
int radar_table_sizes[] = {
    STATIC_RADAR_NUM,
    STATIC_CHANNEL_NUM,
};

// FFT, Clear Freq, Logging Files
FILE *log_file = NULL;
FILE *fft_file[STATIC_RADAR_NUM][STATIC_CHANNEL_NUM] = {NULL};
FILE *clr_file[STATIC_RADAR_NUM][STATIC_CHANNEL_NUM] = {NULL};


void add_ptr(void **ptr) {
    // Check for pre-existing ptr
    for (int i = 0; i < temp_ptrs_num; i++) {
        if (temp_ptrs[i] == ptr) {
            // log_trace("Pointer already exists in temp_ptrs[%d]: %p", i, temp_ptrs[i]);
            return;
        }
    }

    // Allocate memory for the new pointer
    void *tmp = NULL;
    tmp = realloc(temp_ptrs, (temp_ptrs_num + 1) * sizeof(void *));
    if (tmp == NULL) {
        log_fatal( "Error reallocating memory for temp_ptrs");
        perror("Error reallocating memory for temp_ptrs");
        exit(EXIT_FAILURE);
    }
    temp_ptrs = tmp;
    temp_ptrs[temp_ptrs_num] = ptr; 
    // log_trace("  temp_ptrs[%d] = %p -> %p", temp_ptrs_num, temp_ptrs[temp_ptrs_num], *(void **)temp_ptrs[temp_ptrs_num]);
    temp_ptrs_num++;    
}

void update_ptr(void *old_ptr, void *new_ptr) {
    // Check if the old pointer exists in the array
    for (int i = 0; i < temp_ptrs_num; i++) {
        if (temp_ptrs[i] == old_ptr) {
            // log_trace("Updating temp_ptrs[%d] from %p to %p", i, temp_ptrs[i], new_ptr);
            free(temp_ptrs[i]); // Free the old pointer
            temp_ptrs[i] = new_ptr;
            return; // Exit the function once the pointer is found and updated
        }
    }
    
    // If the old pointer is not found, add the new pointer to the array
    add_ptr((void **)&new_ptr);
}

void free_nested_fftw_ptr(void *ptr, int nest_depth, int *sizes) {
    if (ptr == NULL || nest_depth <= 0) {
        return; // Base case: nothing to free
    }
    
    // Debug: Print the pointer and sizes
    // log_trace("free_nested_fftw_ptr: %p, nest_depth: %d,", ptr, nest_depth);
    // log_trace("sizes: ");
    // for (int i = 0; i < nest_depth; i++) {
    //     if (i == nest_depth - 1) {
    //         log_trace("%d", sizes[i]);
    //     } else log_trace("%d ", sizes[i]);
    // }

    if (nest_depth == 1) {
        // Free the final level of pointers
        fftw_free(ptr);
        ptr = NULL;
        return;
    }
    
    // Cast the pointer to a void** to access nested pointers
    void **nested_ptr = (void **)ptr;

    // Iterate through the array and recursively free each nested pointer
    for (int i = 0; i < sizes[0]; i++) {
        if (nested_ptr[i] != NULL) {
            free_nested_fftw_ptr(nested_ptr[i], nest_depth - 1, sizes + 1);
            nested_ptr[i] = NULL; // Set the pointer to NULL after freeing
        }
    }

    fftw_free(nested_ptr);
    nested_ptr = NULL;
}

void free_nested_ptr(void *ptr, int nest_depth, int *sizes) {
    if (ptr == NULL || nest_depth <= 0) {
        return; // Base case: nothing to free
    }
    
    log_trace("free_nested_ptr: %p, nest_depth: %d,", ptr, nest_depth);
    log_trace("sizes: ");
    for (int i = 0; i < nest_depth; i++) {
        if (i == nest_depth - 1) {
            log_trace("%d", sizes[i]);
        } else log_trace("%d ", sizes[i]);
    }

    if (nest_depth == 1) {
        // Free the final level of pointers
        free(ptr);
        ptr = NULL;
        return;
    }
    
    // Cast the pointer to a void** to access nested pointers
    void **nested_ptr = (void **)ptr;

    // Iterate through the array and recursively free each nested pointer
    for (int i = 0; i < sizes[0]; i++) {
        if (nested_ptr[i] != NULL) {
            free_nested_ptr(nested_ptr[i], nest_depth - 1, sizes + 1);
            nested_ptr[i] = NULL; // Set the pointer to NULL after freeing
        }
    }

    free(nested_ptr);
    nested_ptr = NULL;
}


void print_temp_ptrs() {
    log_trace("Contents of temp_ptrs:");
    for (int i = 0; i < temp_ptrs_num; i++) {
        log_trace("  temp_ptrs[%d] = %p -> %p", i, temp_ptrs[i], *(void **)temp_ptrs[i]);
    }
}

void read_sample_shm(fftw_complex *temp_samples, void *samples_shm_ptr, int antenna_num, int samples_num) {
    int *s_ptr = (int *) samples_shm_ptr;
    
    // Store sample data into complex form
    for (int i = 0; i < antenna_num; i++)
    {
        for (int j = 0; j < samples_num; j++) {
            temp_samples[i * samples_num + j] = s_ptr[i * samples_num + j * 2] + I * s_ptr[i * samples_num + j * 2 + 1];

            // Debug: Print 5 complex of each antenna batch
            // if (j < 4 || j > samples_num - 4 || j == 2499) {
            //     log_trace("shm[%d]      =   %d + i%d", i * samples_num + j, ((int*) samples_shm_ptr)[i * samples_num + j * 2], ((int*) samples_shm_ptr)[i * samples_num + j * 2 + 1]);
            //     log_trace("vs");
            //     log_trace("temp_samples[%d][%d] =  %f + i%f", i, j, creal(temp_samples[i][j]), cimag(temp_samples[i][j]));
            // }
        }


        // for (int j = 0; j < samples_num; j += 2)
        // {
        //     temp_samples[i][j] = s_ptr[i * samples_num + j] + I * s_ptr[i * samples_num + j + 1];
        // }
    }
}

/**
 * @brief  Reads in Clear Frequency Band from its shared memory pointer.
 * @note   Used for debugging
 * @param  *clr_band: Clear Frequency Band
 * @param  *ptr: Shared Memory Pointer for Clear Frequency Band
 * @retval None
 */
void read_clrfreq_shm(freq_band *clr_band, int *ptr) {
    int elements_per_band = 3;

    // Store into clr_band
    clr_band->f_start = ptr[elements_per_band];
    clr_band->f_end   = ptr[elements_per_band + 1];
    clr_band->noise   = ptr[elements_per_band + 2];
}

/**
 * @brief  Reads shm integer data into the result ptr.
 * @note   
 * @param  *result: Destination ptr where data will be stored.
 * @param  *shm_ptr: Shared Memory Pointer where data will read-in from.
 * @param  elem_num: Number of elements to read in from shm_ptr. 
 * @retval None
 */
void read_int(int *result, void *shm_ptr, int elem_num) {
    int *ref_ptr = (int *) shm_ptr;

    if (elem_num > 1) {
        for (int i = 0; i < elem_num; i++) {
            result[i] = ref_ptr[i];
            if (VERBOSE && i < 2) log_trace("    read_int: %d", result[i]);
        }
    } else {
        log_error("Use read_single_int() for single variables");
    }
}

void read_int_single(int result, int *shm_ptr) {
    result = *shm_ptr;
    if (VERBOSE) log_trace("    read_int: %d", result);
}

void read_double(double *result, void *shm_ptr, int elem_num) {
    double *ref_ptr = (double *) shm_ptr;

    if (elem_num > 1) {
        for (int i = 0; i < elem_num; i++) {
            result[i] = ref_ptr[i];
            if (VERBOSE && i < 2) log_trace("    read_double: %f", result[i]);
        }
    } else {
        log_error("Error: Use read_single_double() for single variables");
    }
}

void read_meta_data(sample_meta_data *result, void *shm_ptr, int ant_num) {
    log_trace("starting meta read..");
    double *ref_ptr = (double *) shm_ptr;
    int *antenna_ptr = (int *) result->antenna_list;
    
    // Read in meta data elements
    result->number_of_samples = (int) ref_ptr[0];
    result->x_spacing = ref_ptr[1];
    result->usrp_rf_rate = (int) ref_ptr[2];
    log_trace("Fin reading meta_elem; reading antenna_list...");
    
    // Read in antenna_list elements
    for (int i = META_ELEM; i < (ant_num + META_ELEM); i++) {
        if (VERBOSE) log_trace("    reading antenna_list: %d", antenna_ptr[i - META_ELEM]);
        if (VERBOSE) log_trace("    reading ref_ptr     : %d", (int) ref_ptr[i]);
        antenna_ptr[i - META_ELEM] = (int) ref_ptr[i];
    }
}

void read_site_id_data(char *result, void *shm_ptr, int id_num) {
    char *ref_ptr = (char *) shm_ptr;

    for (int i = 0; i < id_num; i++) {
        *(result + i) = ref_ptr[i];
        if (VERBOSE) log_trace("   read_site_id_data[%d]: %c", i, result[i]);
    }
    result[id_num] = '\0';
    log_trace("    read_site_id_data: %s", result);
}

void read_single_int(int *result, void *shm_ptr) {
    *result = *(int *) shm_ptr;
    if (VERBOSE) log_trace("   read_s_int: %d", *result);
}

void read_single_double(double *result, void *shm_ptr){
    *result = *(double *) shm_ptr;
    if (VERBOSE) log_trace("   read_s_int: %f", *result);
}

void write_int(int *result, int *shm_ptr, int n_elements, int shm_len) {
    // log_debug("n %d, len %d", n_elements, shm_len);

    for (int i = 0; i < n_elements; i++) {
        // log_debug("arr: %d", result[i]);
        shm_ptr[i] = result[i];
        // log_debug("writing: %d", shm_ptr[i]);
    }

    // Filling w/ dummy constant
    for (int j = n_elements ; j < shm_len; j++) {
        shm_ptr[j] = -1;
        // log_debug("writing: %d", shm_ptr[j]);
    }
}

/**
 * @brief  Writes Clear Freq Bands to its shared memory pointer.
 * @note   
 * @param  clr_band: Clear Frequency Band
 * @param  *ptr: Shared Memory Pointer for Clear Frequency Bands
 * @retval None
 */
void write_clrfreq_shm(freq_band clr_band, int *ptr) {
    // freq_band *in_ptr = &clr_band;

    int elements_per_band = 3;
    ptr[0]  = clr_band.f_start;
    ptr[1]  = (int) clr_band.noise;
    ptr[2]  = clr_band.f_end;

    log_debug( "Sending the following Clear Frequency: ");
    log_debug("    Clear Freq Band: | %5d kHz -- Noise: %-9.2f -- %5d kHz |", 
        clr_band.f_start/1000, clr_band.noise, clr_band.f_end/1000
    );
}

/**
 * @brief  Unlinks/deallocates Shared Memory Object mapping (ptr), file 
 * * descriptor (fd), and semaphore name (name).
 * @note   
 * @param  shm_obj: Shared Memory Object struct.
 * @retval None
 */
void clean_obj(shm_obj shm_obj) {
    // if (shm_obj.shm_ptr) 
    munmap(shm_obj.shm_ptr, shm_obj.size);
    // if (shm_obj.shm_fd >= 0) 
    close(shm_obj.shm_fd);
    sem_unlink(shm_obj.name);
}

void clean_sem(semaphore sem) {
    // if ((sem.sem)) 
    sem_close(sem.sem);
    sem_unlink(sem.name);
}

/**
 * @brief  Deallocates all service semaphores, SHM pointers, pointers, and the pointers' lengths
 * @note   
 * @retval None
 */
void cleanup() {
    log_info( "Cleaning up ...");
        
    // Cleanup semaphores and SHM objects
    log_info( "Cleaning all semaphores and SHM objects...");
    for (int i = 0; i < SEM_NUM; i++) clean_sem(*semaphores[i]);
    log_info( "Cleaned all semaphores ...");
    for (int i = 0; i < PARAM_NUM; i++) clean_obj(*objects[i]);
    log_info( "Cleaned all objects ...");
    
    // Cleanup fftw ptrs
    // free_nested_fftw_ptr(temp_samples, 2, temp_sample_sizes);
    fftw_free(temp_samples);
    log_debug( "Cleaned temp_samples ...");
    fftw_free(spectra_storage);
    log_debug( "Cleaned spectra_storage ...");
    // cleanup_storage_fft();
    // log_debug( "Cleaned fftw plan ...");
    
    log_info( "Cleaned all fftw pointers ...");
    
    // Cleanup ptrs
    free_nested_ptr(clr_band_storage, 3, clr_storage_sizes);
    log_debug( "Cleaned clr_band_storage ...");
    free_nested_ptr(avg_beam_spectrum, 2, avg_beam_spectrum_sizes);
    log_debug( "Cleaned avg_beam_spectrum ...");
    free(avg_freq_vector);
    log_debug( "Cleaned avg_freq_vector ...");
    free_nested_ptr(radar_table, 2, radar_table_sizes);
    log_debug( "Cleaned radar_table ...");

    for (int i = 0; i < temp_ptrs_num; i++) {
        if (*(void **)temp_ptrs[i] != NULL) { //temp_ptrs[i] != NULL && 
            // log_trace("Freeing temp_ptrs[%d/%d]: %p", i, temp_ptrs_num, *(void **)temp_ptrs[i]);
            free(*(void **)temp_ptrs[i]);
            *(void **)temp_ptrs[i] = NULL; 
        } else {
            log_error("Skipping invalid or NULL pointer at temp_ptrs[%d]", i);
        }
    }
    free(temp_ptrs);
    

    log_info("Cleaned all pointers ...");
    
    // cleanup_fftw_threads();
    fftw_cleanup();
    log_info( "Cleaned all fftw-related entities...");
}

/**
 * @brief  Catch exit signals and exits gracefully by calling to deallocate all service parameters.
 * @note   
 * @param  sig: Caught signal
 * @retval None
 */
void handle_sig(int sig) {
    log_warn("Caught signal %d, cleaning up and exiting...", sig);
    cleanup();
    
    // Prompt exit to terminal  
    log_warn( "Main processes and communication terminated. Goodbye.\n");
    
    fclose(log_file);
    if (access(SPECTRAL_LOG_FILE, F_OK) == 0) {
        for (int r_idx = 0; r_idx < STATIC_RADAR_NUM; r_idx++) {
            for (int c_idx = 0; c_idx < STATIC_CHANNEL_NUM; c_idx++) {
                fclose(fft_file[r_idx][c_idx]);
                fclose(clr_file[r_idx][c_idx]);
            }
        }
    }
    exit(sig);
}

void write_clr_log_csv(freq_band **clr_storage, int clr_num, char *ststr, int channel, int clr_range[STATIC_RANGE_NUM][2]) {
    // Timestamp Variables
    time_t raw_time;
    struct tm *time_info;
    int buffer_size = 128;
    char timestamp[buffer_size];
    char name[buffer_size]; 

    // Generate timestamp
    log_trace("Generating timestamp...");
    time(&raw_time);
    time_info = gmtime(&raw_time);
    strftime(timestamp, buffer_size, "%Y%m%d.%H%M.%S", time_info);
    snprintf(name, sizeof(name), "/data/log/clr_freq/%s.%s.%c.clrlog.csv", timestamp, ststr, channel+96);

    // Generate clear log file
    FILE *file = fopen(name, "w");
    if (file == NULL) {
        log_error( "Error opening file for writing");
        perror("Error opening file for writing");
        exit(EXIT_FAILURE);
    }
    fprintf(file, "Start Frequency,End Frequency,Noise,Clear Freq Start,Clear Freq End\n");
    for (int clr_batch_idx = 0; clr_batch_idx < clr_num; clr_batch_idx++) {
        freq_band *clr_band = clr_storage[clr_batch_idx];

        // Find Start and End of Clear Freq Range
        int clr_start = RAND_MAX;
        int clr_end = 0;
        for (int ridx = 0; ridx < STATIC_RANGE_NUM; ridx++) {
            if ((clr_band->f_start >= clr_range[ridx][0]) &&
                (clr_band->f_end <= clr_range[ridx][1]) &&
                (clr_band->noise < RAND_MAX)) {
                clr_start = clr_range[ridx][0];
                clr_end = clr_range[ridx][1];
                break;
            }
        }

        // Record Clear Freq
        // Debug: Output results
        // log_trace("Clear Freq Band: | %dHz -- Noise: %f -- %dHz |\n", clr_storage[clr_batch_idx][i].f_start, clr_storage[clr_batch_idx][i].noise, clr_storage[clr_batch_idx][i].f_end);
        
        fprintf(file, "%d,%d,%f,%d,%d\n", clr_band->f_start, clr_band->f_end, clr_band->noise,
                                          clr_start, clr_end);
    }

    fclose(file);
}

/**
 * @brief  Reallocates the long-term dynamic memory for storage variables, which is reliant on samples_num.
 * @note   
 * @param  samples_num: Number of Samples
 * @param  total_beams: Total number of beams to process
 * @retval None
 */
void realloc_storage(int samples_num, int total_beams, int radar_num, int avg_ratio) {
    log_info( "samples_num Reallocation in progress...");

    // Realloc spectra_storage
    fftw_free(spectra_storage);
    spectra_storage = fftw_alloc_complex(radar_num * STATIC_RANGE_NUM * STORAGE_NUM * total_beams * samples_num);
    if (spectra_storage == NULL) {
        log_fatal("Error allocating memory for spectra_storage");
        perror("Error allocating memory for spectra_storage");
        exit(EXIT_FAILURE);
    }
    log_trace( "Allocated new spectra_storage memory...");
    
    // Realloc avg_beam_spectrum
    free_nested_ptr(avg_beam_spectrum, 2, avg_beam_spectrum_sizes);
    avg_beam_spectrum_sizes[0] = total_beams;
    avg_beam_spectrum_sizes[1] = samples_num / avg_ratio;

    avg_beam_spectrum = (double **)malloc(total_beams * sizeof(double *));
    if (avg_beam_spectrum == NULL) {
        log_fatal( "Error reallocating memory for avg_beam_spectrum pointers");
        perror("Error reallocating memory for avg_beam_spectrum pointers");
        exit(EXIT_FAILURE);
    }
    for (int i = 0; i < total_beams; i++) {
        avg_beam_spectrum[i] = calloc((samples_num / avg_ratio), sizeof(double));
        // (double *)malloc((samples_num / avg_ratio) * sizeof(double));
        if (avg_beam_spectrum[i] == NULL) {
            log_fatal( "Error reallocating memory for avg_beam_spectrum elements");
            perror("Error reallocating memory for avg_beam_spectrum elements");
            exit(EXIT_FAILURE);
        }
        // memset(avg_beam_spectrum[i], 0, (samples_num / AVG_RATIO) * sizeof(double));
    }
    log_trace( "Allocated new avg_beam_spectrum memory...");

    // Realloc avg_freq_vector
    free(avg_freq_vector);
    avg_freq_vector = calloc(samples_num / avg_ratio, sizeof(double));
    
    if (avg_freq_vector == NULL) {
        log_fatal( "Error reallocating memory for avg_freq_vector");
        perror("Error reallocating memory for avg_freq_vector");
        exit(EXIT_FAILURE);
    }

    log_info( "Samples Num Reallocation completed...");
}


void realloc_samples(int samples_num)
{
    log_trace( "Freeing Sample SHM Cache...");
    munmap(samples_obj.shm_ptr, samples_obj.size);

    // Set Size of Shared Memory Object
    samples_obj.size = (meta_data.num_antennas) * samples_num * 2 * sizeof(int);
    if (ftruncate(samples_obj.shm_fd, samples_obj.size) == -1)
    {
        log_fatal(" ftruncate failed");
        perror("ftruncate failed");
        exit(EXIT_FAILURE);
    }

    // Request Block of Memory
    log_trace("Requesting Sample SHM Cache...");
    samples_obj.shm_ptr = mmap(0, samples_obj.size, PROT_WRITE | PROT_READ, MAP_SHARED, samples_obj.shm_fd, 0);
    if (samples_obj.shm_ptr == MAP_FAILED)
    {
        log_fatal("Memory Mapping failed for %s", samples_obj.name);
        perror("Memory Mapping failed");
        exit(EXIT_FAILURE);
    }
    log_trace("Sample SHM successfully cached...");

    // Free previously allocated memory for temp_samples
    fftw_free(temp_samples);
    log_trace("Freed old temp_samples memory...");

    // Reallocate temp_samples
    temp_samples = fftw_alloc_complex(meta_data.num_antennas * samples_num);
    if (temp_samples == NULL)
    {
        log_fatal("Error reallocating memory for temp_samples pointers");
        perror("Error reallocating memory for temp_samples pointers");
        exit(EXIT_FAILURE);
    }
    log_trace("Allocated new temp_samples memory...");
}


/* @brief  Checks if the TCS parameters have changed and updates them if necessary.
 * @note   
 * @param  old_tcs_param: Old TCS parameters
 * @param  new_tcs_param: New TCS parameters
 * @retval true if any parameter has changed, false otherwise
 */
bool has_tcs_param_changed(int old_tcs_param[3], int new_tcs_param[3]) {
    // Check if any of the parameters have changed
    bool has_changed = false;
    for (int i = 0; i < 3; i++) {
        if (old_tcs_param[i] != new_tcs_param[i]) {
            log_debug("TCS Param[%d]: %d vs %d", i, old_tcs_param[i], new_tcs_param[i]);
            has_changed = true;
            old_tcs_param[i] = new_tcs_param[i]; // Update the old parameter
        }
    }

    return has_changed;
}

/**
 * @brief  Clears the list of active antennas.
 * @note   
 * @param  active_antennas: Array of active antennas
 * @retval None
 */
void clear_active_antennas(int active_antennas[]) {
    // Set the active antennas based on the antenna_list
    for (int i = 0; i < STATIC_ANTENNA_NUM; i++) {
        active_antennas[i] = 0;
    }
}


int main() {
    // Setup Signal Handler (catches ctrl+c and termination? to quit safely)
    signal(SIGTERM, handle_sig);
    signal(SIGINT, handle_sig);
    signal(SIGSEGV, handle_sig);

    // Initialize Logging
    log_file = init_log(LOG_TERMINAL_LEVEL, LOG_FILE_LEVEL, LOG_FILEPATH);
    if (log_file == NULL) {
        log_fatal("Error opening log file");
        perror("Error opening log file");
        exit(EXIT_FAILURE);
    }
    log_info("Starting Clear Frequency Service's Server...");


    log_info("Pre-Cleaning all Shared Memory...\n");
    cleanup();
    
    // Open Shared Memory Object
    log_trace( "Initializing Shared Memory Object...");
    for (int i = 0; i < PARAM_NUM; i++){
        objects[i]->shm_fd = shm_open(objects[i]->name, O_CREAT | O_RDWR, 0666);
        if (objects[i]->shm_fd == -1) {
            log_fatal( "shm_open failed for %s", objects[i]->name);
            perror("shm_open failed");
            exit(EXIT_FAILURE);
        }
    }
    log_trace( "Created Shared Memory Objects...");
    
    // Set Size of Shared Memory Object
    for (int i = 0; i < PARAM_NUM; i++){
        if (ftruncate(objects[i]->shm_fd, objects[i]->size) == -1) {
            log_fatal( " ftruncate failed");
            perror("ftruncate failed");
            exit(EXIT_FAILURE);
        }
    }

    // Request Block of Memory
    log_trace( "Requesting Shared Memory Cache...");    
    for (int i = 0; i < PARAM_NUM; i++) {
        objects[i]->shm_ptr = mmap(0, objects[i]->size, PROT_WRITE | PROT_READ, MAP_SHARED, objects[i]->shm_fd, 0);
        if (objects[i]->shm_ptr == MAP_FAILED) {
            log_fatal( "Memory Mapping failed for %s", objects[i]->name);
            perror("Memory Mapping failed");
            exit(EXIT_FAILURE);
        }
    }
    log_info( "Memory successfully cached...");

    // Initialize SHM to zero
    log_trace( "Initializing Shared Memory to zero...");
    for (int i = 0; i < PARAM_NUM; i++) {
        memset(objects[i]->shm_ptr, 0, objects[i]->size);
    }
    log_trace( "Successfully initialized SHM to zero...");


    // Open Semaphores for synchronization     
    log_info( "Opening Communication Semaphores...");    
    for (int i = 0; i < SEM_NUM; i++) {
        // Set Semaphore flags to 0
        if (i < (SEM_NUM - SL_NUM)) semaphores[i]->sem = sem_open(semaphores[i]->name, O_CREAT, 0666, 0);
        // Set Semaphore locks to 1
        else semaphores[i]->sem = sem_open(semaphores[i]->name, O_CREAT, 0644, 1);
        if (semaphores[i]->sem == SEM_FAILED) {
            log_fatal( "\"%s\" sem_open failed.", semaphores[i]->name);
            perror("sem_open failed");
            exit(EXIT_FAILURE);    
        } 
    }
    log_info( "Done Initializing SHM...\n");


    // Initialize Array Configuration
    log_info( "Initializing Array Configuration...");
    Config array_config = {0}; 
    int ini_check = ini_parse(ARRAY_CONFIG_FILEPATH, config_ini_handler, &array_config);
    if (ini_check < 0) {
        log_fatal( "Error reading array configuration file");
        perror("Error reading array configuration file");
        exit(EXIT_FAILURE);
    }
    int radar_num = array_config.array_info.nradars;
    if (radar_num <= 0) {
        log_warn( "Defaulting to 1 radar.");
        log_warn( "If you are using a two radars, please set the number of radars under the [array_info] section: ");
        log_warn( "\"nradars = 2\"");
        if (radar_num == 0) log_error( "nradars is missing from the config file! Please add it to specify number of radars.");
        radar_num = 1;
    }
    int beam_total = array_config.array_info.nbeams;
    int cur_radar = 0;
    int *muted_config_ants = array_config.gain_control.mute_antenna_ids;
    int num_muted_config_ants = array_config.gain_control.num_mute_antennas;
    char ststr[STATIC_RADAR_NUM][SITE_ID_ELEM + 1];
    strcpy(ststr[0], array_config.array_info.radar_stid);
    if (radar_num == 2) {
        strcpy(ststr[1], array_config.array_info.radar_stid_2);
    }

    // Allocate temp mem for shm varibles
    temp_samples = fftw_alloc_complex(ANTENNA_NUM * SAMPLES_NUM);
    if (temp_samples == NULL) {
        log_fatal("Error allocating memory for temp_samples pointers");
        perror("Error allocating memory for temp_samples pointers");
        exit(EXIT_FAILURE);
    }

    spectra_storage = fftw_alloc_complex(radar_num * STATIC_RANGE_NUM * STORAGE_NUM * beam_total * SAMPLES_NUM);
    if (spectra_storage == NULL) {
        log_fatal("Error allocating memory for spectra_storage");
        perror("Error allocating memory for spectra_storage");
        exit(EXIT_FAILURE);
    }

    // Get Clear Frequency Resolution 
    log_debug( "Reading Avg_ratio from radar_config_constants.py ...");
    int avg_ratio = 0;
    read_radar_config(RADAR_CONST_CONFIG_FILEPATH, &avg_ratio);
    if (avg_ratio <= 0) {
        log_debug( "Avg_ratio: %d", avg_ratio);
        log_fatal( "Error reading radar configuration file");
        perror("Error reading radar configuration file");
        exit(EXIT_FAILURE);
    }
    log_debug( "Done reading Average Ratio...");

    avg_beam_spectrum_sizes[0] = beam_total;
    avg_beam_spectrum_sizes[1] = SAMPLES_NUM / avg_ratio;
    avg_beam_spectrum = (double **)malloc(beam_total * sizeof(double *));
    if (avg_beam_spectrum == NULL) {
        log_fatal( "Error reallocating memory for avg_beam_spectrum pointers");
        perror("Error reallocating memory for avg_beam_spectrum pointers");
        exit(EXIT_FAILURE);
    }
    for (int i = 0; i < beam_total; i++) {
        avg_beam_spectrum[i] = calloc((SAMPLES_NUM / avg_ratio), sizeof(double));
        // (double *)malloc((samples_num / AVG_RATIO) * sizeof(double));
        if (avg_beam_spectrum[i] == NULL) {
            log_fatal( "Error reallocating memory for avg_beam_spectrum elements");
            perror("Error reallocating memory for avg_beam_spectrum elements");
            exit(EXIT_FAILURE);
        }
        // memset(avg_beam_spectrum[i], 0, (samples_num / AVG_RATIO) * sizeof(double));
    }

    int restricted_num = RESTRICT_NUM;
    freq_band restricted_freq[restricted_num];
    for (int i = 0; i < restricted_num; i++) {
        restricted_freq[i].f_start = 0;
        restricted_freq[i].f_end = 0;
    }

    freq_band *clr_band = malloc(sizeof(freq_band));
    memset(clr_band, 0, sizeof(freq_band));

    clr_storage_sizes[0] = radar_num;
    clr_band_storage = (freq_band ****)malloc(radar_num * sizeof(freq_band ***));
    if (clr_band_storage == NULL) {
        log_fatal( "Error allocating memory for clr_band_storage radar pointers");
        perror("Error allocating memory for clr_band_storage radar pointers");
        exit(EXIT_FAILURE);
    }
    for (int i = 0; i < radar_num; i++) {
        clr_band_storage[i] = (freq_band ***)malloc(STATIC_RADAR_NUM * sizeof(freq_band **));
        if (clr_band_storage[i] == NULL) {
            log_fatal( "Error allocating memory for clr_band_storage pointers");
            perror("Error allocating memory for clr_band_storage pointers");
            exit(EXIT_FAILURE);
        }
        for (int j = 0; j < STATIC_RADAR_NUM; j++) {
            clr_band_storage[i][j] = (freq_band **)malloc(CLR_STORAGE_NUM * sizeof(freq_band *));
            if (clr_band_storage[i][j] == NULL) {
                log_fatal( "Error allocating memory for clr_band_storage elements");
                perror("Error allocating memory for clr_band_storage elements");
                exit(EXIT_FAILURE);
            }
            for (int k = 0; k < CLR_STORAGE_NUM; k++) {
                clr_band_storage[i][j][k] = (freq_band *)malloc(sizeof(freq_band));
                if (clr_band_storage[i][j][k] == NULL) {
                    log_fatal( "Error allocating memory for clr_band_storage elements");
                    perror("Error allocating memory for clr_band_storage elements");
                    exit(EXIT_FAILURE);
                }
                memset(clr_band_storage[i][j][k], 0, sizeof(freq_band));
            }
        }
    }

    radar_table = (radar_freq_data **)malloc(STATIC_RADAR_NUM * sizeof(radar_freq_data *));
    if (radar_table == NULL) {
        log_fatal( "Error allocating memory for radar_table pointers");
        perror("Error allocating memory for radar_table pointers");
        exit(EXIT_FAILURE);
    }
    for (int i = 0; i < STATIC_RADAR_NUM; i++) {
        radar_table[i] = (radar_freq_data *)malloc(STATIC_CHANNEL_NUM * sizeof(radar_freq_data));
        if (radar_table[i] == NULL) {
            log_fatal( "Error allocating memory for radar_table elements");
            perror("Error allocating memory for radar_table elements");
            exit(EXIT_FAILURE);
        }
        memset(radar_table[i], 0, STATIC_CHANNEL_NUM * sizeof(radar_freq_data));
    }

    int channel = 0;
    int cur_range = 0;
    int cur_channel = 0;
    int cur_beam = 0;
    int sample_sep = 0;
    int old_antenna_num = ANTENNA_NUM;
    int samples_num = SAMPLES_NUM;
    // int old_samples_num = -1;
    // int old_usrp_fcenter = -1;
    int old_usrp_rf_rate = -1;
    int old_tcs_param[3] = {
        samples_num,
        beam_total,
        old_usrp_rf_rate
    };
    int new_tcs_param[3] = {
        samples_num,
        beam_total,
        0
    };
    int valid_sample_cycles[STATIC_RADAR_NUM] = {0};                    // num of times valid samples were in send() cycle 
    int ccn_invalid_sample_cyles[STATIC_RADAR_NUM] = {0};               // num of times in a row invalid samples were in send() cycle
    long accu_avg_ant_pwr[STATIC_RADAR_NUM][STATIC_ANTENNA_NUM] = {0};  // integrated avg antenna power from sample sets for an accurate avg ant power
    int active_antennas[STATIC_RADAR_NUM][STATIC_ANTENNA_NUM] = {0};    // active antennas for each radar
    int ant_active_ct[STATIC_RADAR_NUM][STATIC_ANTENNA_NUM] = {0};      // num of times antenna was active
    int active_ant_num = 0;
    int muted_ant_ids[STATIC_RADAR_NUM][STATIC_ANTENNA_NUM] = {0};      // Inactive Main Array Antennas to be muted at USRP Server side
    int muted_ant_idx = 0;
    int clr_storage_i[STATIC_RADAR_NUM][STATIC_CHANNEL_NUM] = {0};
    int tcs_storage_i[STATIC_RADAR_NUM][STATIC_RANGE_NUM] = {0};
    bool is_tcs_ready[STATIC_RADAR_NUM][STATIC_RANGE_NUM] = {false};
    int clr_range[STATIC_RADAR_NUM][STATIC_RANGE_NUM][2] = {0};
    time_t clr_range_time[STATIC_RADAR_NUM][STATIC_RANGE_NUM] = {0};
    int clr_range_overwrite_idx[STATIC_RADAR_NUM] = {0};                  // Index used to track which clr_range to overwrite (increments to next clr_range idx each overwrite)
    bool using_full_usrp_range[STATIC_RADAR_NUM][STATIC_RANGE_NUM] = {false}; 
    freq_band selected_clr_band = {0};
    int def_low_range[STATIC_RADAR_NUM] = {0};
    int def_high_range[STATIC_RADAR_NUM]= {0};

    bool first_request[STATIC_RADAR_NUM][STATIC_CHANNEL_NUM];
    for (int ridx = 0; ridx < STATIC_RADAR_NUM; ridx++) {
      for (int cidx = 0; cidx < STATIC_CHANNEL_NUM; cidx++) {
        first_request[ridx][cidx] = true;
      }
    }

    // Failure flags
    bool fl_clr_range_out_bounds = false; // Flag for Clear Search Range being out of bounds
    
    // Parameters for Reading Restricted Frequencies
    char restrict_file[255] = "";
    char *rst_path = getenv("RSTPATH");
    if (rst_path == NULL) {
        log_fatal( "$RSTPATH not found. Restrict Freq file is inaccessible.\n");
        perror("Error: $RSTPATH not found");
        exit(EXIT_FAILURE);
    }

    log_info( "Getting site's Resticted Frequencies...");
    int str_f_result = 0; 
    
    // Get site specific restrict file and join with path
    if (strcmp(ststr[0],"lab") != 0) {
        log_info( "Using /site.%s/restrict.dat.%s in ststr\n", ststr[0], ststr[0]);
        str_f_result = snprintf(restrict_file, sizeof(restrict_file), "%s/tables/superdarn/site/site.%s/restrict.dat.%s", rst_path, ststr[0], ststr[0]);
        if (str_f_result < 1) {
            log_error( " site path format failed");
            return 1;
        }
    }

    // Default: Get lab testing restrict file
    else {
        log_warn("WARNING: Parameter \'ststr\' not passed from usrp_server or set to the \"lab\" setting!");
        str_f_result = snprintf(restrict_file, sizeof(restrict_file), "%s/tables/superdarn/site/site.%s/restrict.dat.%s", rst_path, DEFAULT_SITE_STSTR, DEFAULT_SITE_STSTR);
        if (str_f_result < 1) {
            log_error( " site path format failed");
            return 1;
        }
    }

    log_info("Using restrict file path: %s\n", restrict_file);
    read_restrict(restrict_file, restricted_freq, &restricted_num);

    // Continuously process clients via shared memory
    while (1) {
        log_info( "Requesting new client to respond...");
        sem_post(sf_client.sem); 
        log_info( "Awaiting client response...");
        sem_wait(sf_server.sem);   
        log_info( "Processing CF Client...");


        double t1,t2;
        t1 = clock();

        // If initialization data flagged, read and store data
        if (sem_trywait(sf_init.sem) == 0){
            log_info( "Awaiting initialization data unlock...");
            sem_wait(sl_init.sem);
            log_info( "Initialization data read...");

            // Read Antenna number
            log_debug( "Antenna Number reading...");
            read_single_int(&meta_data.num_antennas, antenna_obj.shm_ptr);
            
            // If new num_antennas, Reallocate meta SHM 
            if (meta_data.num_antennas != old_antenna_num) {
                log_info( "Reallocating Meta Shared Memory...");
                log_debug("num of antenna: %d", meta_data.num_antennas);

                log_trace( "Freeing Meta SHM Cache...");
                munmap(meta_obj.shm_ptr, meta_obj.size);
                
                // Set Size of meta_data SHM Object
                log_trace( "Setting Size of Meta SHM Cache...");
                meta_obj.size = (meta_data.num_antennas + META_ELEM) * sizeof(double);
                if (ftruncate(meta_obj.shm_fd, meta_obj.size) == -1) {
                    log_fatal( " ftruncate failed");
                    perror("ftruncate failed");
                    exit(EXIT_FAILURE);
                }

                // Request meta_data's Block of Memory
                log_trace( "Requesting Meta SHM Cache...");                    
                meta_obj.shm_ptr = mmap(0, meta_obj.size, PROT_WRITE | PROT_READ, MAP_SHARED, meta_obj.shm_fd, 0);
                if (meta_obj.shm_ptr == MAP_FAILED) {
                    log_fatal( "Memory Mapping failed for %s", meta_obj.name);
                    perror("Memory Mapping failed");
                    exit(EXIT_FAILURE);
                }                    
                log_trace( "Meta Data successfully cached...");     
                
                // Read Meta Data 
                log_trace( "Meta Data reading...");
                read_meta_data(&meta_data, meta_obj.shm_ptr, meta_data.num_antennas);
                samples_num = meta_data.number_of_samples;

                // Reallocate Samples SHM
                log_info( "Reallocating Sample related Memory due to change in Antenna Num...");
                realloc_samples(samples_num);
            
                old_antenna_num = meta_data.num_antennas;
                log_info( "Reallocation due to change in Antenna Num done...");
            }
            
            // Default: Read in Meta Data
            else {
                log_trace( "Meta Data reading...");
                read_meta_data(&meta_data, meta_obj.shm_ptr, meta_data.num_antennas);
                samples_num = meta_data.number_of_samples;
            }
            
            // If critical TCS parameters have changed, reset TCS
            new_tcs_param[0] = samples_num;
            new_tcs_param[1] = beam_total;
            new_tcs_param[2] = meta_data.usrp_rf_rate;
            if (has_tcs_param_changed(old_tcs_param, new_tcs_param) == true) {
                log_info( "TCS Parameters changed...");
                for (int r_idx = 0; r_idx < radar_num; r_idx++) {
                    for (int range_idx = 0; range_idx < STATIC_RANGE_NUM; range_idx++) {
                        is_tcs_ready [r_idx][range_idx] = false;
                        tcs_storage_i[r_idx][range_idx] = 0;
                    }
                }
                
                realloc_storage(samples_num, beam_total, radar_num, avg_ratio);

                // Reset Avg Antenna Power and Missing Antenna Trackers
                for (int r_idx = 0; r_idx < radar_num; r_idx++) {
                    valid_sample_cycles[r_idx] = 0;
                    ccn_invalid_sample_cyles[r_idx] = 0;
                    for(int ant_idx = 0; ant_idx < STATIC_ANTENNA_NUM; ant_idx++) {
                        accu_avg_ant_pwr[r_idx][ant_idx] = 0;
                        ant_active_ct[r_idx][ant_idx] = 0;
                    }
                }
                // log_info( "Reinitializing TCS FFTW plan...");
                // cleanup_storage_fft();
                // init_storage_fft(samples_num, beam_total);
            }
            
            // Debug: Display meta_data info
            for (int j = 0; j < meta_data.num_antennas; j++) {
                log_trace("    antenna_list[%d]: %d", j, meta_data.antenna_list[j]);
            }
            log_debug("     num_antennas: %d", meta_data.num_antennas);
            log_debug("     num_samples : %d", meta_data.number_of_samples);
            log_debug("     fcenter: passed during sample DT");
            log_debug("     rf_rate     : %d", meta_data.usrp_rf_rate);
            log_debug("     x_spacing   : %f", meta_data.x_spacing);

            sem_post(sl_init.sem);

            // log_info( "Initialization data read; processing...");

            // TODO: storeInRadarTable(restrict_freq, meta_data)

            // log_info( "Initialization data processed...");
        }
        
        // If samples flagged, process samples and clear frequency
        if (sem_trywait(sf_samples.sem) == 0){
            // Special: Semaphore order is faulty
            if (meta_data.num_antennas == 0) {
                log_error( "ERROR: Called samples flag without prior initialization");
                log_error( "ERROR: There is likely a semaphore leak, please close and restart all related processes.");
                cleanup();
                return 0;
            }
            
            // Wait to read-in data
            log_trace( "Awaiting sample data unlock...");
            sem_wait(sl_samples.sem);

            // Process Sample relevant data
            log_info( "Processing client sample data...");
            read_sample_shm(temp_samples, samples_obj.shm_ptr, meta_data.num_antennas, samples_num);
            log_trace( "Samples done...");

            // Read Center Frequency
            if (*(int*) (fcenter_obj.shm_ptr) != 0) {
                log_debug( "Freq Center reading...");
                read_single_int( &(meta_data.usrp_fcenter), fcenter_obj.shm_ptr);
                log_debug("    fcenter: %d", meta_data.usrp_fcenter);
            }

            // Read Radar ID
            if (*(int*) (radar_id_obj.shm_ptr) >= 0) {
                log_debug( "Radar ID reading...");
                read_single_int(&cur_radar, radar_id_obj.shm_ptr);
                log_debug("    cur_radar: %d", cur_radar);
                if (cur_radar >= radar_num) {
                    log_error( "ERROR: Radar ID out of range");
                    log_error( "ERROR: There is likely a semaphore leak or error in CFS order of operations, please close and restart all related processes.");
                    perror("ERROR: Radar ID out of range");
                    exit(EXIT_FAILURE);
                }
            }

            // Skip reading channel ID, as it is not used in this context


            // Lock out so no data interferance 
            sem_post(sl_samples.sem);


            // Process Average Antenna Power
            log_info( "[TCS] Processing Average Antenna Power...");
            clear_active_antennas(active_antennas[cur_radar]);
            process_avg_ant_pwr(
                temp_samples,
                samples_num,
                &meta_data,
                muted_config_ants,
                num_muted_config_ants,
                ant_active_ct[cur_radar],
                active_antennas[cur_radar],
                accu_avg_ant_pwr[cur_radar]
            );

            // Check that main array antennas are active
            active_ant_num = 0;
            muted_ant_idx = 0;
            for (int i = 0; i < STATIC_ANTENNA_NUM; i++) {
                // log_debug("checking ant#%d", i);

                // Ignore inferrometer array
                if (active_antennas[cur_radar][i] > 0 && (i <= IDX_LAST_MA || i > IDX_LAST_IA) ) {
                    active_ant_num++;
                    log_debug("checking ant#%d: active", i);
                }
                else if (i <= IDX_LAST_MA || i > IDX_LAST_IA) {
                    muted_ant_ids[cur_radar][muted_ant_idx] = i;
                    muted_ant_idx++;
                    log_trace("checking ant#%d: inactive", i);
                }
                else {
                    log_trace("checking ant#%d: inferro (ignored)", i);
                }
            }
            log_info("Active Main Array Antennas: %d", active_ant_num);
            // If has an active antenna, Increment # of valid sample sets 
            if (active_ant_num > 0) {
                valid_sample_cycles[cur_radar]++;
                ccn_invalid_sample_cyles[cur_radar] = 0;
            }
            else {ccn_invalid_sample_cyles[cur_radar]++;}

            // Send back Muted Antennas 
            if (USE_ACTIVE_MUTE == 1) write_int(muted_ant_ids[cur_radar], muted_ant_obj.shm_ptr, muted_ant_idx, STATIC_ANTENNA_NUM);
            else write_int(muted_config_ants, muted_ant_obj.shm_ptr, num_muted_config_ants, STATIC_ANTENNA_NUM);

            // Process and Store Spectra Data
            if (tcs_storage_i[cur_radar][cur_range] < STORAGE_NUM && active_ant_num > 0) { // Ignore empty sample sets
                log_info( "Processing Spectra Data...");

                // If clr_range is not set, default clr_range to usrp_fcenter -/+ 0.5 * usrp_rf_rate
                if (clr_range[0][0][0] == 0 && clr_range[0][0][1] == 0) {
                    log_info( "Setting default clr_range...");
                    for (int i = 0; i < STATIC_RADAR_NUM; i++) {
                        using_full_usrp_range[i][0] = true;
                        clr_range_time[i][0] = time(NULL);
                        for (int j = 0; j < STATIC_RANGE_NUM; j++) {
                            clr_range[i][j][0] = (meta_data.usrp_fcenter * 1000 - (meta_data.usrp_rf_rate / 2));
                            clr_range[i][j][1] = (meta_data.usrp_fcenter * 1000 + (meta_data.usrp_rf_rate / 2));
                        }
                        def_low_range [i] = (meta_data.usrp_fcenter * 1000 - (meta_data.usrp_rf_rate / 2));
                        def_high_range[i] = (meta_data.usrp_fcenter * 1000 + (meta_data.usrp_rf_rate / 2));
                    }
                    log_info( "Default clr_range set to %d -- %d kHz", clr_range[0][0][0]/1000, clr_range[0][0][1]/1000);
                }

                if (USE_MULTI_RANGE == 1) {
                    log_info("Processing samples in Multi Range Mode...");

                    // Process Spectra for all active Clear Ranges
                    for (int range_idx = 0; range_idx < STATIC_RANGE_NUM; range_idx++) {

                        // Special: Skip processing of default clear ranges, unless Client is scanning entire usrp range
                        if (clr_range[cur_radar][range_idx][0] == def_low_range [cur_radar] && 
                            clr_range[cur_radar][range_idx][1] == def_high_range[cur_radar] &&
                            using_full_usrp_range[cur_radar][range_idx] == false
                        ) {
                            log_trace("skipping range [%5d -- %5d]...",
                                clr_range[cur_radar][range_idx][0] / 1000, 
                                clr_range[cur_radar][range_idx][1] / 1000
                            );
                            continue;
                        }

                        // Skip processing of inactive clear ranges
                        if (time(NULL) - clr_range_time[cur_radar][range_idx] > 120) {
                            log_trace("skipping range [%5d -- %5d]...",
                                clr_range[cur_radar][range_idx][0] / 1000, 
                                clr_range[cur_radar][range_idx][1] / 1000
                            );
                            continue;
                        }

                        // Process Spectra 
                        log_info("Processing samples for #%d[%5d -- %5d]...", 
                            range_idx, 
                            clr_range[cur_radar][range_idx][0] / 1000, 
                            clr_range[cur_radar][range_idx][1] / 1000
                        );
                        process_all_beamformed_spectras(
                                temp_samples,
                                active_antennas[cur_radar],
                                clr_range[cur_radar][range_idx], 
                                sample_sep, 
                                restricted_freq, 
                                restricted_num,
                                &meta_data,
                                array_config,
                                &(spectra_storage[
                                    cur_radar * STATIC_RANGE_NUM *  STORAGE_NUM * beam_total * samples_num + 
                                    range_idx * STORAGE_NUM * beam_total * samples_num +
                                    tcs_storage_i[cur_radar][range_idx] * beam_total * samples_num
                                ])
                            );
        
                        // Display TCS state
                        log_info( "[TCS] Processed Radar[%d][%5d --  %5d]'s spectra_storage[%d/%d] successfully...", 
                            cur_radar, 
                            clr_range[cur_radar][range_idx][0] / 1000,
                            clr_range[cur_radar][range_idx][1] / 1000,
                            tcs_storage_i[cur_radar][range_idx] + 1, 
                            STORAGE_NUM
                        );
                        tcs_storage_i[cur_radar][range_idx] += 1;
        
                        // Reset TCS Storing point at (Storage_Num - 1)
                        if (tcs_storage_i[cur_radar][range_idx] >= STORAGE_NUM) {
                            log_info( "[TCS] Radar[%d][%5d -- %5d] Storage is now Ready...", 
                                cur_radar, 
                                clr_range[cur_radar][range_idx][0] / 1000, 
                                clr_range[cur_radar][range_idx][1] / 1000
                            );
                            is_tcs_ready [cur_radar][range_idx] = true;
                            tcs_storage_i[cur_radar][range_idx] = 0;
                        } 
                    }
                }

                else {
                    // Process Spectra 
                    process_all_beamformed_spectras(
                            temp_samples,
                            active_antennas[cur_radar],
                            clr_range[cur_radar][cur_range], 
                            sample_sep, 
                            restricted_freq, 
                            restricted_num,
                            &meta_data,
                            array_config,
                            &(spectra_storage[
                                cur_radar * STATIC_RANGE_NUM *  STORAGE_NUM * beam_total * samples_num + 
                                cur_range * STORAGE_NUM * beam_total * samples_num +
                                tcs_storage_i[cur_radar][cur_range] * beam_total * samples_num
                            ])
                        );

                    // Display TCS state
                    log_info( "[TCS] Processed Radar[%d][%5d --  %5d]'s spectra_storage[%d/%d] successfully...", 
                        cur_radar, 
                        clr_range[cur_radar][cur_range][0] / 1000,
                        clr_range[cur_radar][cur_range][1] / 1000,
                        tcs_storage_i[cur_radar][cur_range] + 1, 
                        STORAGE_NUM
                    );
                    tcs_storage_i[cur_radar][cur_range] += 1;

                    if (tcs_storage_i[cur_radar][cur_range] >= STORAGE_NUM) {
                        log_info( "[TCS] Radar[%d][%5d -- %5d] Storage is now Ready...", 
                            cur_radar, 
                            clr_range[cur_radar][cur_range][0] / 1000, 
                            clr_range[cur_radar][cur_range][1] / 1000
                        );
                        is_tcs_ready [cur_radar][cur_range] = true;
                        tcs_storage_i[cur_radar][cur_range] = 0;
                    } 
                }
            }
            else if (active_ant_num == 0) {
                log_info( "[TCS] No active main antennas found for Radar[%d], skipping spectra storage...", cur_radar);
            }

            log_info( "Stored Samples successfully...");

            // Flag that Processed Data is ready
            log_info( "Signaling that Processed Data is ready!");
            if (msync(muted_ant_obj.shm_ptr, MUTED_ANT_SHM_SIZE, MS_SYNC) == -1) { // Synchronize data writes with program counter
                log_error( "msync failed");
                perror("msync failed");
            }
            sem_post(sf_processed.sem);
        } 

        // If Clear Freq flagged, process clear frequency
        else if (sem_trywait(sf_clrfreq.sem) == 0) {
            // Lock Write Clear Freq Data
            log_trace( "Aquiring Semaphore Locks...");
            sem_wait(sl_clrfreq.sem);
            log_info( "Starting CFS Clear Frequency Process...");


            // Read in Clear Freq Data
            log_trace( "Reading Clear Freq Data...");

            // Read Beam_num
            if (msync(beam_num_obj.shm_ptr, BEAM_NUM_SHM_SIZE, MS_SYNC) == -1) { // Synchronize data writes with program counter
                log_error( "msync failed");
                perror("msync failed");
            }
            if (*(int*) (beam_num_obj.shm_ptr) >= 0) {
                log_debug( "Beam Number reading...");
                read_single_int(&cur_beam, beam_num_obj.shm_ptr);
                log_debug("    cur_beam: %d", cur_beam);
                if (cur_beam >= beam_total) {
                    log_error( "ERROR: Beam Number out of range");
                    log_error( "ERROR: There is likely a semaphore leak or error in CFS order of operations, please close and restart all related processes.");
                    perror("ERROR: Beam Number out of range");
                    exit(EXIT_FAILURE);
                }
            }

            // Read Sample Separation
            if ( *(int*) (sample_sep_obj.shm_ptr) != 0) {
                log_debug( "Sample Separation reading...");
                read_single_int(&sample_sep, sample_sep_obj.shm_ptr);
                log_debug("    sample_sep: %d", sample_sep);
            }

            // Read Center Frequency
            if (*(int*) (fcenter_obj.shm_ptr) != 0) {
                log_debug( "Freq Center reading...");
                read_single_int( &(meta_data.usrp_fcenter), fcenter_obj.shm_ptr);
                log_debug("    fcenter: %d", meta_data.usrp_fcenter);
            }

            // Read Radar ID
            if (*(int*) (radar_id_obj.shm_ptr) >= 0) {
                log_debug( "Radar ID reading...");
                read_single_int(&cur_radar, radar_id_obj.shm_ptr);
                log_debug("    cur_radar: %d", cur_radar);
                if (cur_radar >= radar_num) {
                    log_error( "ERROR: Radar ID out of range");
                    log_error( "ERROR: There is likely a semaphore leak or error in CFS order of operations, please close and restart all related processes.");
                    perror("ERROR: Radar ID out of range");
                    exit(EXIT_FAILURE);
                }
            }

            // Read Channel ID
            if (*(int*) (channel_id_obj.shm_ptr) >= 0) {
                log_debug( "Channel ID reading...");
                read_single_int(&channel, channel_id_obj.shm_ptr);
                log_debug("    channel_id: %d", channel);
                if (strcmp(ststr[cur_radar],"kod") == 0) {
                    cur_channel = 4 - channel;
                } else {
                    cur_channel = channel - 1;
                }
                log_debug("    cur_channel: %d", cur_channel);

                if (cur_channel >= STATIC_CHANNEL_NUM || cur_channel < 0) {
                    log_error( "ERROR: Channel ID out of range");
                    log_error( "ERROR: There is likely an error on the CFS client-side, please close and restart all related processes.");
                    perror("ERROR: Channel ID out of range");
                    exit(EXIT_FAILURE);
                }
            }

            // Read Clear Range
            if (*(int*) (clr_range_obj.shm_ptr) != 0) {
                int tmp_clr_range[2] = {0};
                int old_clr_range[2] = {0};

                log_debug( "Clear Range reading...");
                read_int(tmp_clr_range, clr_range_obj.shm_ptr, 2);

                // If clr_range is in kHz, convert to Hz
                if (tmp_clr_range[0] < 100000 ||    tmp_clr_range[1] < 100000) {
                    tmp_clr_range[0] =              tmp_clr_range[0] * 1000;
                    tmp_clr_range[1] =              tmp_clr_range[1] * 1000;
                    log_debug("    new_clr_range: %d -- %d kHz", tmp_clr_range[0]/1000, tmp_clr_range[1]/1000);
                }

                // If Clear Range exists, don't overwrite and set as cur_range
                bool range_exists = false;
                if (USE_MULTI_RANGE == 1) {
                    // If Multi Range Optimization, Check existing clear ranges
                    for (int i = 0; i < STATIC_RANGE_NUM; i++) {
                        if (tmp_clr_range[0] == clr_range[cur_radar][i][0] && tmp_clr_range[1] == clr_range[cur_radar][i][1]) {
                            range_exists = true;
                            old_clr_range[0] = clr_range[cur_radar][i][0];
                            old_clr_range[1] = clr_range[cur_radar][i][1]; 
                            cur_range = i;
                        }
                        log_debug("    tmp_clr_range: %d -- %d kHz", tmp_clr_range[0]/1000, tmp_clr_range[1]/1000);

                        // Special: Client wants full usrp range
                        if (def_low_range[cur_radar] == tmp_clr_range[0] && def_high_range[cur_radar] == tmp_clr_range[1]) {
                            log_trace("searching full usrp range");
                            using_full_usrp_range[cur_radar][i] = true;
                        } else {
                            log_trace("not searching full usrp range");
                            using_full_usrp_range[cur_radar][i] = false;
                        } 
                    }
                } else {
                    // If Single Range Optimization, only check 1st range
                    if (tmp_clr_range[0] == clr_range[cur_radar][0][0] && tmp_clr_range[1] == clr_range[cur_radar][0][1]) {
                        range_exists = true;
                        old_clr_range[0] = clr_range[cur_radar][0][0];
                        old_clr_range[1] = clr_range[cur_radar][0][1]; 
                        cur_range = 0;
                        log_debug("    matching_range: %5d -- %5d kHz", clr_range[cur_radar][0][0]/1000, clr_range[cur_radar][0][1]/1000);
                    }

                    // Special: Client wants full usrp range
                    if (def_low_range[cur_radar] == tmp_clr_range[0] && def_high_range[cur_radar] == tmp_clr_range[1]) using_full_usrp_range[cur_radar][0] = true;
                    else using_full_usrp_range[cur_radar][0] = false;
                }

                // If a radar's clr_range doesn't exist, set it
                if (range_exists != true) {
                    // Set clr_range
                    cur_range = clr_range_overwrite_idx[cur_radar];
                    clr_range[cur_radar][cur_range][0] = tmp_clr_range[0];
                    clr_range[cur_radar][cur_range][1] = tmp_clr_range[1];

                    // Display that change occured
                    log_info( "Radar#%d's Clear Range changed...", cur_radar);
                    log_info("    old_clr_range: %d -- %d", old_clr_range[0], old_clr_range[1]);
                    log_info("    clr_range: %d -- %d", clr_range[cur_radar][cur_range][0]/1000, clr_range[cur_radar][cur_range][1]/1000);
                    
                    // Reset TCS per clr range
                    is_tcs_ready[cur_radar] [cur_range] = false;
                    tcs_storage_i[cur_radar][cur_range] = 0;

                    // Prep for next clear range overwrite
                    clr_range_overwrite_idx[cur_radar]++;
                }

                clr_range_time[cur_radar][cur_range] = time(NULL);

                // Special: Restrict Clr Range to Usrp Range
                // Restrict lower bound to lower bound of Usrp Range
                if (clr_range[cur_radar][cur_range][0] < (meta_data.usrp_fcenter * 1000 - (meta_data.usrp_rf_rate / 2))) {
                    clr_range[cur_radar][cur_range][0] = (meta_data.usrp_fcenter * 1000 - (meta_data.usrp_rf_rate / 2));
                }
                // Restrict upper bound to upper bound of Usrp Range
                if (clr_range[cur_radar][cur_range][1] > (meta_data.usrp_fcenter * 1000 + (meta_data.usrp_rf_rate / 2))) {
                    clr_range[cur_radar][cur_range][1] = (meta_data.usrp_fcenter * 1000 + (meta_data.usrp_rf_rate / 2));
                }
                
                // Fail: Clr Range has no intersection with Usrp Range 
                if (clr_range[cur_radar][cur_range][1] < (meta_data.usrp_fcenter * 1000 - (meta_data.usrp_rf_rate / 2)) ||
                    clr_range[cur_radar][cur_range][0] > (meta_data.usrp_fcenter * 1000 + (meta_data.usrp_rf_rate / 2))) 
                {
                    log_error("ERROR: Clear Range is out of Usrp Range!");
                    log_error("ERROR: Please check your Clear Range and Usrp RF Rate settings.");
                    // print usrp range and clr range
                    log_error("Usrp Range: %d -- %d",
                        (meta_data.usrp_fcenter * 1000 - (meta_data.usrp_rf_rate / 2)),
                        (meta_data.usrp_fcenter * 1000 + (meta_data.usrp_rf_rate / 2))
                    );
                    log_error("Clr Range: %d -- %d", clr_range[cur_radar][cur_range][0], clr_range[cur_radar][cur_range][1]);
                    log_error("ERROR: Please check your Clear Range and Usrp RF Rate settings.");

                    // Prevent further CFS from processing search
                    fl_clr_range_out_bounds = true;
                }

            }
            
            // Unmask current channel's old reserved frequency
            log_debug("Unmasking old reserved clr band[%d]: | %5d kHz -- %5d kHz |", 
                restricted_num + cur_radar * STATIC_CHANNEL_NUM + cur_channel,
                restricted_freq[restricted_num + cur_radar * STATIC_CHANNEL_NUM + cur_channel].f_start/1000,
                restricted_freq[restricted_num + cur_radar * STATIC_CHANNEL_NUM + cur_channel].f_end/1000
            );
            restricted_freq[restricted_num + cur_radar * STATIC_CHANNEL_NUM + cur_channel].f_end = 0;
            restricted_freq[restricted_num + cur_radar * STATIC_CHANNEL_NUM + cur_channel].f_start = 0;
            restricted_freq[restricted_num + cur_radar * STATIC_CHANNEL_NUM + cur_channel].noise = 0;
            radar_table[cur_radar][cur_channel].clr_band.f_start = 0;
            radar_table[cur_radar][cur_channel].clr_band.noise = 0;
            radar_table[cur_radar][cur_channel].clr_band.f_end = 0;
            radar_table[cur_radar][cur_channel].last_time = 0;
            radar_table[cur_radar][cur_channel].last_time = 0;

            // Check for inactive channels to unmask
            for (int r_idx = 0; r_idx < STATIC_RADAR_NUM; r_idx++) {
                for (int c_idx = 0; c_idx < STATIC_CHANNEL_NUM; c_idx++) {
                    if (radar_table[r_idx][c_idx].last_time == 0) continue;

                    if (time(NULL) - radar_table[r_idx][c_idx].last_time > 10) {
                        log_debug("Unmasking old reserved clr band[%d]: | %5d kHz -- %5d kHz |",
                            restricted_num + r_idx * STATIC_CHANNEL_NUM + c_idx,
                            restricted_freq[restricted_num + r_idx * STATIC_CHANNEL_NUM + c_idx].f_start/1000,
                            restricted_freq[restricted_num + r_idx * STATIC_CHANNEL_NUM + c_idx].f_end/1000
                        );
                        restricted_freq[restricted_num + r_idx * STATIC_CHANNEL_NUM + c_idx].f_end = 0;
                        restricted_freq[restricted_num + r_idx * STATIC_CHANNEL_NUM + c_idx].f_start = 0;
                        restricted_freq[restricted_num + r_idx * STATIC_CHANNEL_NUM + c_idx].noise = 0;
                        radar_table[r_idx][c_idx].clr_band.f_start = 0;
                        radar_table[r_idx][c_idx].clr_band.noise = 0;
                        radar_table[r_idx][c_idx].clr_band.f_end = 0;
                        radar_table[r_idx][c_idx].last_time = 0;
                    }
                }
            }

            if (first_request[cur_radar][cur_channel] == true) {
                if (access(SPECTRAL_LOG_FILE, F_OK) == 0) {
                    log_trace("Initializing FFT File");
                    char* tcs_spectra_filename_template[128] = {0};
                    char* tcs_spectra_filename[128] = {0};
                    sprintf(tcs_spectra_filename_template, SPECTRUM_FILE, "%s", ststr[cur_radar], channel+96, "%s");
                    char *ext = BIN_OR_CSV_LOG ? ".csv" : "bin";
                    log_trace("extension \"%s\" enabled", ext);
                    gen_filename(&tcs_spectra_filename_template, ext, &tcs_spectra_filename);
                    fft_file[cur_radar][cur_channel] = fopen(tcs_spectra_filename, BIN_OR_CSV_LOG ? "w" : "wb");
                    if (fft_file[cur_radar][cur_channel] == NULL) {
                        file_access_error(tcs_spectra_filename);
                        return;
                    }

                    log_trace("Initializing Clear Freq File");
                    char *tcs_clr_filename_template[128] = {0};
                    char *tcs_clr_filename[128] = {0};
                    sprintf(tcs_clr_filename_template, CLR_FREQ_FILE, "%s", ststr[cur_radar], channel+96, "%s");
                    gen_filename(&tcs_clr_filename_template, ext, &tcs_clr_filename);
                    clr_file[cur_radar][cur_channel] = fopen(tcs_clr_filename, BIN_OR_CSV_LOG ? "w" : "wb");
                    if (clr_file[cur_radar][cur_channel] == NULL) {
                        file_access_error(tcs_clr_filename);
                        return;
                    }
                }

                first_request[cur_radar][cur_channel] = false;
            }

            log_info( "    avg_ratio: %d", avg_ratio);
            log_info( "    delta_f: %d", (int) (meta_data.usrp_rf_rate / samples_num) );


            // Clear Freq Processing
            // If no fail flags, proceed with Clear Frequency Search
            if (fl_clr_range_out_bounds == false) {
                // If TCS is not ready, process new clrfreq per unique beam request
                if (is_tcs_ready[cur_radar][cur_range] == false) {
                    
                    // Fail: No active antennas and no sample data from prior clr freqs
                    if (active_ant_num == 0 && (clr_band->noise == 0 || clr_band->noise == RAND_MAX)) {
                        log_error("ERROR: No main array antennas active and no computed clr freqs...CFS has no data to compensate with.");
                    } 

                    // Special: No active antennas, but usable sample data from prior CFS cycle
                    else if (active_ant_num == 0 && (clr_band->noise != 0 && clr_band->noise != RAND_MAX)) {
                        log_warn("WARN: No main array antennas active, but computed clr freqs... using old clr freqs to compensate...");

                        // "Output Clear Freq Bands" handles the selection of clr freqs... 
                    } 

                    // Default: Process Clear Search
                    else {
                        log_info( "Clr Freq @ Beam #%d ...", cur_beam);
                        log_info( "Starting Clear Freq Search...");
                        clear_freq_search(
                            temp_samples, 
                            active_antennas[cur_radar],
                            clr_range[cur_radar][cur_range],
                            cur_beam,
                            sample_sep,
                            avg_ratio,
                            restricted_freq, 
                            restricted_num + RESERV_NUM,
                            meta_data,
                            array_config,
                            clr_band,
                            fft_file[cur_radar][cur_channel],
                            clr_file[cur_radar][cur_channel],
                            ststr[cur_radar],
                            channel
                        );
                    }                
                } // end of Clear Search (CS) 
                // If TCS ready, process beam-specific clr freq
                else {
                    log_info( "[TCS] Clr Freq @ Beam #%d ...", cur_beam);
                    process_avg_beam_spectra(
                        &(spectra_storage[cur_radar * STATIC_RANGE_NUM * STORAGE_NUM * beam_total * samples_num]),
                        avg_ratio, 
                        meta_data.number_of_samples,
                        cur_beam,
                        beam_total,
                        STORAGE_NUM,
                        &meta_data,
                        avg_beam_spectrum,
                        avg_freq_vector,
                        fft_file[cur_radar][cur_channel],
                        ststr[cur_radar],
                        channel
                    );
                    log_trace( "    Avg Beam Spectrum done...");

                    process_beam_clr_freq(
                        avg_beam_spectrum,
                        cur_beam,
                        clr_range[cur_radar][cur_range],
                        sample_sep,
                        restricted_freq, 
                        restricted_num + RESERV_NUM,
                        avg_freq_vector,
                        (int) (meta_data.number_of_samples / avg_ratio),
                        &meta_data,
                        clr_band,
                        clr_file[cur_radar][cur_channel],
                        ststr[cur_radar],
                        channel
                    );
                    log_info( "[TCS] Clr Freq @ Beam #%d done...", cur_beam);
                } // end of TCS 
            } // end of Clear Freq Processing

            // Clear Freq Selection and Display 
            bool is_clr_band_found = true;
            if (clr_band->noise != 0 && clr_band->noise != RAND_MAX) {  // if clr_band filled correctly, proceed to Freq Selection
                log_debug("Clear Freq Band: | %5d kHz -- Noise: %-9.2f -- %5d kHz |", 
                    clr_band->f_start/1000,
                    clr_band->noise,
                    clr_band->f_end/1000
                );
                
                // Compare with Radar/Reservation Table
                bool cur_freq_intersects = false;
                for (int r_idx = 0; r_idx < STATIC_RADAR_NUM; r_idx++) {
                    for (int c_idx = 0; c_idx < STATIC_CHANNEL_NUM; c_idx++) {
                        freq_band compare_freq = radar_table[cur_radar][c_idx].clr_band;
    
                        // If cur freq intersects w/ reservations, flag to skip the freq 
                        if ((compare_freq.f_start <= clr_band->f_start   && clr_band->f_start < compare_freq.f_end ) ||
                             compare_freq.f_start <= clr_band->f_end      && clr_band->f_end <= compare_freq.f_end ) {
                                cur_freq_intersects = true;

                                // If this warning occurs, CFS' find_clr_freq method could be causing the issue
                                log_warn("WARN: CFS Freq Reservation failsafe has prevented an intersecting clear freq when it shouldn't have needed to...");
                                break;
                            }
                    }
                    if (cur_freq_intersects == true) {break;} 
                }

                // Skip the freq; Do NOT select it
                if (cur_freq_intersects == true) {
                    is_clr_band_found = false;
                }
            } else {
                is_clr_band_found = false;
            }

            if (is_clr_band_found == true) {
                log_debug("    Reserving frequency into RadarTable...");
                
                // Reserve the frequency band 
                radar_table[cur_radar][cur_channel].clr_band = *clr_band;
                radar_table[cur_radar][cur_channel].clear_freq_range[0] = clr_range[cur_radar][cur_range][0];
                radar_table[cur_radar][cur_channel].clear_freq_range[1] = clr_range[cur_radar][cur_range][1];
                radar_table[cur_radar][cur_channel].last_time = time(NULL);
                if (restricted_num + cur_radar * STATIC_CHANNEL_NUM + cur_channel >= RESTRICT_NUM) {
                    log_error("    ERROR: Reservation into restricted_freq failed due to overflow index!");
                    perror("ERROR: Reservation into restricted_freq failed due to overflow index!");
                    exit(EXIT_FAILURE);
                }
                restricted_freq[restricted_num + cur_radar * STATIC_CHANNEL_NUM + cur_channel] = *clr_band;

                // Select the best frequency band
                selected_clr_band = *clr_band;

                // Flag abnormal clr_band in log
                if (clr_band->f_start == 0 || clr_band->f_end == 0 || clr_band->noise == 0 ||
                    clr_band->f_start == RAND_MAX || clr_band->f_end == RAND_MAX || clr_band->noise == RAND_MAX) {
                    log_error("ERROR: Clear Freq Band is abnormal");
                    log_error("Clear Freq Band: | %5d kHz -- Noise: %-9.2f -- %5d kHz |",
                        clr_band->f_start/1000, clr_band->noise, clr_band->f_end/1000
                    );
                    log_error("ERROR: There COULD be an error in CFS order of operations, or too wide of a guardband/narrow clear search range!");
                }
            }
            // Fail: A valid clr freq could not be found 
            else {
                selected_clr_band.f_start = 0;
                selected_clr_band.f_end = 0;
                selected_clr_band.noise = 0;
                log_error("ERROR: CFS couldn't compute valid Clear Freqs. See eariler logs for details.");

                // If prev reservation for radar's channel exists, maintain freq
                if (radar_table[cur_radar][cur_channel].clr_band.noise > 0 && radar_table[cur_radar][cur_channel].clr_band.noise < RAND_MAX) {
                    selected_clr_band = radar_table[cur_radar][cur_channel].clr_band;
                    restricted_freq[restricted_num + cur_radar * STATIC_CHANNEL_NUM + cur_channel] = radar_table[cur_radar][cur_channel].clr_band;

                    log_warn("CFS resorted to old reservation: | %5d kHz -- Noise: %-9.2f -- %5d kHz |", 
                            selected_clr_band.f_start/1000, selected_clr_band.noise, selected_clr_band.f_end/1000
                        );
                }
            }
            write_clrfreq_shm(selected_clr_band, clrfreq_obj.shm_ptr);

            // Log clear freq bands
            memcpy(clr_band_storage[cur_radar][cur_channel][clr_storage_i[cur_radar][cur_channel]], &selected_clr_band, sizeof(freq_band));
            clr_storage_i[cur_radar][cur_channel]++;
            log_info( "Clr Freq Log: Radar[%d][%d] @ %d/%d", cur_radar, cur_channel, clr_storage_i[cur_radar][cur_channel], CLR_STORAGE_NUM);
            if (clr_storage_i[cur_radar][cur_channel] >= CLR_STORAGE_NUM) {
                write_clr_log_csv(clr_band_storage[cur_radar][cur_channel], clr_storage_i[cur_radar][cur_channel], ststr[cur_radar], channel, clr_range[cur_radar]);
                clr_storage_i[cur_radar][cur_channel] = 0;
            }
            

            // Display TCS Storage Information
            log_info( "[TCS] Radar[%d][%5d -- %5d] Storage [%d/%d]", 
                cur_radar, 
                clr_range[cur_radar][cur_range][0] / 1000, 
                clr_range[cur_radar][cur_range][1] / 1000, 
                tcs_storage_i[cur_radar][cur_range] + 1, STORAGE_NUM
            );
            if (is_tcs_ready[cur_radar][cur_range] == true) log_info( "[TCS] Radar[%d][%5d -- %5d] Storage is now Ready...", 
                            cur_radar, 
                            clr_range[cur_radar][cur_range][0] / 1000, 
                            clr_range[cur_radar][cur_range][1] / 1000
                        );
            else log_info( "[TCS] Radar[%d][%5d -- %5d] is NOT Ready...", 
                cur_radar,
                clr_range[cur_radar][cur_range][0] / 1000, 
                clr_range[cur_radar][cur_range][1] / 1000
            );

            // Display Average Antenna Power, reset active antennas, and warn of antenna abnormalities
            log_info( "[TCS] Average Antenna Power (total valid cycles: %d):", valid_sample_cycles[cur_radar]);
            for (int ant_idx = 0; ant_idx < STATIC_ANTENNA_NUM; ant_idx++) {
                long avg_ant_pwr = (ant_active_ct[cur_radar][ant_idx] == 0) ? 0 : accu_avg_ant_pwr[cur_radar][ant_idx] / valid_sample_cycles[cur_radar];
                char *ant_status;

                // Check if muted in Array Config
                bool is_muted = false;
                for (int config_i = 0; config_i < num_muted_config_ants; config_i++) {
                    if (muted_config_ants[config_i] == ant_idx) {
                        is_muted = true;
                        break;
                    }
                }          
                if (is_muted == true) ant_status = "   muted";
                else if (active_antennas[cur_radar][ant_idx] > 0 && (ant_idx <= IDX_LAST_MA || ant_idx > IDX_LAST_IA)) {
                    ant_status = "  active";
                } else {
                    ant_status = "inactive";
                }

                log_info( "-> ant#%2d[radar#%d][%s]: %6d (missed %4d)", 
                    ant_idx,
                    cur_radar, 
                    ant_status,
                    avg_ant_pwr,
                    valid_sample_cycles[cur_radar] - ant_active_ct[cur_radar][ant_idx]
                );
            }
            // If several poor sample sets received in row, forwarn negative effects
            if (ccn_invalid_sample_cyles[cur_radar] >= STORAGE_NUM) {
                log_error("ERROR: CFS's time integrated samples have been filled with invalid samples. CFS can no longer compensate!");
            }
            else if (ccn_invalid_sample_cyles[cur_radar] >= (STORAGE_NUM - 10)) {
                log_warn( "WARN: High number of invalid/poor main array sample sets sent! Will drastically effect clr freqs!");
                log_debug( "[TCS] Invalid sample sets: %d/%d", ccn_invalid_sample_cyles[cur_radar], STORAGE_NUM);
            }
            // // If active antennas are less than # passed to CFS, flag
            // if (active_ant_num < meta_data.num_antennas) {
            //     log_warn( "WARN: Main Active antennas (%d) < passed antennas (%d) results in less noise", 
            //         active_ant_num, 
            //         meta_data.num_antennas
            //     );
            // }

            // Display Radar Reservation Info Table
            log_debug( "Radar Reservation Info:");
            for (int r_idx = 0; r_idx < radar_num; r_idx++) {
                for (int c_idx = 0; c_idx < STATIC_CHANNEL_NUM; c_idx++) {
                    if (radar_table[r_idx][c_idx].clr_band.f_start != 0 && radar_table[r_idx][c_idx].clr_band.f_end != 0) {
                        log_debug( "    Radar[%d] Channel[%d]: | %5d kHz -- Noise: %-9.2f -- %5d kHz | in range: | %5d kHz -- %5d kHz |",
                            r_idx, c_idx, 
                            radar_table[r_idx][c_idx].clr_band.f_start      / 1000, 
                            radar_table[r_idx][c_idx].clr_band.noise, 
                            radar_table[r_idx][c_idx].clr_band.f_end        / 1000,
                            radar_table[r_idx][c_idx].clear_freq_range[0]   / 1000,
                            radar_table[r_idx][c_idx].clear_freq_range[1]   / 1000
                        );
                    }
                }
            }


            // Synchronize data writes with program counter
            if (msync(clrfreq_obj.shm_ptr, CLR_BAND_SHM_SIZE, MS_SYNC) == -1) {
                log_fatal( "msync failed");
                perror("msync failed");
            }            

            log_trace( "clrfreq_shm written...");
            sem_post(sl_clrfreq.sem);
            sem_post(sf_processed.sem);
            log_trace( "Processed Clear Freq Request successfully...");
        }

        // If no data to process, signal possible error
        else {
            log_fatal( "ERROR: No Clear Freq or Sample flag to process...");
            log_fatal( "ERROR: There is likely a semaphore leak or error in CFS order of operations, please close and restart all related processes.");
            perror( "ERROR: No Clear Freq or Sample flag to process... likely a semaphore leak or error in CFS order of operations");
            exit(EXIT_FAILURE);
        }
        
        
        t2 = clock();
        log_info( "Processed Client successfully...");
        log_info( "Processing Time for Client (s): %lf\n", ((double) (t2 - t1)) / (CLOCKS_PER_SEC));
    }
};
