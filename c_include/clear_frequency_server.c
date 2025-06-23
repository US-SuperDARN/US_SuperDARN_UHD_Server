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
#include "clear_freq_search.h"
#include "ini_parser.h"
#include "read_config.c"
#include "log.h"
#include "misc_read_writes.h"

// Build with the following flags:
// -lrt -pthread -lfftw3 -lm 


// Logging Vars
#define LOG_TERMINAL_LEVEL 2                         // 0 = TRACE, 1 = DEBUG, 2 = INFO, 3 = WARN, 4 = ERROR, 5 = FATAL  
#define LOG_FILE_LEVEL 1
#define LOG_PREFIX "[CFS] %s"               // *Unused* Prefix for log messages
#define LOG_FILEPATH "log/cfs/cfs.%s.log"

// Server Config Vars
#define AVG_RATIO 4                                 // Number of samples to average during spectral averaging (4 = 4 samples avg-ed per beam)
#define FFTW_THREADS 2                              // Number of threads to use for FFTW

// Filepaths Vars
#define ARRAY_CONFIG_FILEPATH           "array_config.ini"
#define RADAR_CONST_CONFIG_FILEPATH     "python_include/radar_config_constants.py"

// Default Length of Variables (some dynamically change during runtime)
#define SAMPLES_NUM             2500
#define ANTENNA_NUM             16
#define STATIC_ANTENNA_NUM      30 
#define STATIC_RADAR_NUM        4                   // Max Number of possible radars in an array
#define STATIC_CHANNEL_NUM      6                   // Max Number of possible channels in an array
#define RESERV_NUM              (STATIC_RADAR_NUM * STATIC_CHANNEL_NUM) // Number of reserved freq bands in the radar_table 
#define BEAM_NUM                16                  // Number of beams to process
#define SAMPLE_TIME             3                   // Time per Sample (in seconds)
#define STORAGE_TIME            60                  // Total time per Sample Storage Batch (in seconds)
#define STORAGE_NUM             (STORAGE_TIME / SAMPLE_TIME) // Total number of processed sample sets to store
#define META_ELEM               3                   // 4 = 5 - 1 (fcenter has unique obj)
#ifndef RESTRICT_NUM
#define RESTRICT_NUM            50                  // Number of restricted freq bands in the restrict.dat.inst
#endif
#ifndef CLR_BANDS_MAX
#define CLR_BANDS_MAX           6
#endif
#define CLR_STORAGE_NUM         10
#define SITE_ID_ELEM            3                   // 3 = 3-letter identifier 

#define SAMPLES_SHM_SIZE        (ANTENNA_NUM * SAMPLES_NUM * 2 * sizeof(int)) 
#define CLR_RANGE_SHM_SIZE      (2 * sizeof(int))
#define FCENTER_SHM_SIZE        (1 * sizeof(int))
#define BEAM_NUM_SHM_SIZE       (1 * sizeof(int))
#define SAMPLE_SEP_SHM_SIZE     (1 * sizeof(int))
#define RESTRICT_SHM_SIZE       (RESTRICT_NUM * 2 * sizeof(int))          // 2 = start and end freqs
#define META_DATA_SHM_SIZE      ((META_ELEM + ANTENNA_NUM) * sizeof(double))
#define ANTENNA_SHM_SIZE        (1 * sizeof(int))
#define CLR_BANDS_SHM_SIZE      (1 * sizeof(int) * 3)    
#define SITE_ID_SHM_SIZE        (SITE_ID_ELEM * sizeof(char))
#define RADAR_ID_SHM_SIZE       (1 * sizeof(int))
#define CHANNEL_ID_SHM_SIZE     (1 * sizeof(int))
#define ACTIVE_CLIENTS_SHM_SIZE (1 * sizeof(int))

// Shared Memory and Semaphore Names 
#define SAMPLES_SHM_NAME        "/samples"
#define CLR_RANGE_SHM_NAME      "/clear_freq_range"
#define FCENTER_SHM_NAME        "/fcenter"
#define BEAM_NUM_SHM_NAME       "/beam_num"
#define SAMPLE_SEP_SHM_NAME     "/sample_sep"
#define RESTRICT_SHM_NAME       "/restricted_freq"
#define META_DATA_SHM_NAME      "/meta_data"
#define ANTENNA_SHM_NAME        "/antenna_num"
#define CLRFREQ_SHM_NAME        "/clear_freq"
#define SITE_ID_SHM_NAME        "/site_id"
#define RADAR_ID_SHM_NAME       "/radar_id"
#define CHANNEL_ID_SHM_NAME     "/channel_id"
#define ACTIVE_CLIENTS_SHM_NAME "/active_clients"   // For debugging

#define SAMPLE_PARAM_NUM 4
#define RESTRICT_PARAM_NUM 2
#define PARAM_NUM 13

#define SEM_F_CLIENT    "/sf_client"                // For Sync and reserving client and server roles during data transfer
#define SEM_F_SERVER    "/sf_server"    
#define SEM_F_SAMPLES   "/sf_samples"
#define SEM_F_INIT      "/sf_init"           
#define SEM_F_CLRFREQ   "/sf_clrfreq"               // For multiple data transfers on single instance 
#define SEM_F_PROCESSED "/sf_processed"             // For processed data transfer
#define SEM_L_SAMPLES   "/sl_samples"               // For Data locking b/w write/reads
#define SEM_L_INIT      "/sl_init"                  // init = initialization
#define SEM_L_CLRFREQ   "/sl_clrfreq"
#define SL_NUM 3
#define SEM_NUM 9

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
shm_obj clrfreq_obj     = {CLRFREQ_SHM_NAME, NULL, -1, CLR_BANDS_SHM_SIZE};
shm_obj site_id_obj     = {SITE_ID_SHM_NAME, NULL, -1, SITE_ID_SHM_SIZE};
shm_obj radar_id_obj    = {RADAR_ID_SHM_NAME, NULL, -1, RADAR_ID_SHM_SIZE};
shm_obj channel_id_obj  = {CHANNEL_ID_SHM_NAME, NULL, -1, CHANNEL_ID_SHM_SIZE};
shm_obj client_num_obj  = {ACTIVE_CLIENTS_SHM_NAME, NULL, -1, ACTIVE_CLIENTS_SHM_SIZE};
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
freq_band ***clr_bands_storage = NULL;
int clr_storage_sizes[] = {
    1, 
    CLR_STORAGE_NUM,
    CLR_BANDS_MAX,
};
sample_meta_data meta_data = {0};
radar_freq_data **radar_table = NULL;
int radar_table_sizes[] = {
    STATIC_RADAR_NUM,
    STATIC_CHANNEL_NUM,
};

FILE *log_file = NULL;


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
 * @brief  Reads in Clear Frequency Bands from its shared memory pointer.
 * @note   Used for debugging
 * @param  *clr_bands: Clear Frequency Bands
 * @param  *ptr: Shared Memory Pointer for Clear Frequency Bands
 * @retval None
 */
void read_clrfreq_shm(freq_band *clr_bands, int *ptr) {
    int elements_per_band = 3;

    // Store into clr_bands
    for (int i = 0; i < CLR_BANDS_MAX; i++) {
        clr_bands[i].f_start= ptr[i * elements_per_band];
        clr_bands[i].f_end  = ptr[i * elements_per_band + 1];
        clr_bands[i].noise  = ptr[i * elements_per_band + 2];
    }
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
    log_debug("    Clear Freq Band[%s]: | %dHz -- Noise: %f -- %dHz |", 
        clr_band.is_selected ? "Selected" : "Free", clr_band.f_start, clr_band.noise, clr_band.f_end
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
    free_nested_ptr(clr_bands_storage, 3, clr_storage_sizes);
    log_debug( "Cleaned clr_bands_storage ...");
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
    exit(sig);
}

void write_clr_log_csv(freq_band **clr_storage, int clr_num, int radar_id) {
    // Timestamp Variables
    time_t raw_time;
    struct tm *time_info;
    int buffer_size = 128;
    char timestamp[buffer_size];
    char name[buffer_size]; 

    // Generate timestamp
    log_trace("Generating timestamp...");
    time(&raw_time);
    time_info = localtime(&raw_time);
    strftime(timestamp, buffer_size, "%Y.%m.%d_%H:%M:%S", time_info);
    snprintf(name, sizeof(name), "log/clr_freq/clrlog_%s.r%d.csv", timestamp, radar_id);

    // Generate clear log file
    FILE *file = fopen(name, "w");
    if (file == NULL) {
        log_error( "Error opening file for writing");
        perror("Error opening file for writing");
        exit(EXIT_FAILURE);
    }
    fprintf(file, "Start Frequency,End Frequency,Noise,Clear Freq Start,Clear Freq End\n");
    for (int clr_batch_idx = 0; clr_batch_idx < clr_num; clr_batch_idx++) {
        freq_band *clr_bands = clr_storage[clr_batch_idx];

        // Find Start and End of Clear Freq Range
        int clr_start = RAND_MAX;
        int clr_end = 0;
        for (int i = 0; i < CLR_BANDS_MAX; i++) {
            if (clr_bands[i].f_start < clr_start && clr_bands[i].noise < RAND_MAX) clr_start = clr_bands[i].f_start;
            if (clr_bands[i].f_end > clr_end && clr_bands[i].noise < RAND_MAX) clr_end = clr_bands[i].f_end;
        }    

        // Record each Clear Freq
        for (int i = 0; i < CLR_BANDS_MAX; i++) {
            // Debug: Output results
            // log_trace("Clear Freq Band[%d]: | %dHz -- Noise: %f -- %dHz |\n", i, clr_storage[clr_batch_idx][i].f_start, clr_storage[clr_batch_idx][i].noise, clr_storage[clr_batch_idx][i].f_end);
            
            // Special: Print Clear Freq Range on Line 0
            if (i == 0) fprintf(file, "%d,%d,%f,%d,%d\n", clr_bands[i].f_start, clr_bands[i].f_end, clr_bands[i].noise,clr_start,clr_end);
            else fprintf(file, "%d,%d,%f\n", clr_bands[i].f_start, clr_bands[i].f_end, clr_bands[i].noise);
        }
    }

    fclose(file);
}

void flag_debug() {

    log_warn("[FLAG DEBUGGING] All functionality except for semaphore flags is absent!");
    log_warn("[FLAG DEBUGGING] Comment out the flag_debug() function to revert to standard functionality.\n");

    // Debug: Verify that the flags are down 
    int test = sem_trywait(sf_server.sem);
    log_info("sf_server was recieved if 0: %d", test);
    int test1 = sem_trywait(sf_init.sem);
    log_info("sf_int was recieved if 0: %d", test1);
    int test2 = sem_trywait(sf_samples.sem);
    log_info("sf_samples was recieved if 0: %d", test2);
    int test3 = sem_trywait(sf_clrfreq.sem);
    log_info("sf_clrfrqe was recieved if 0: %d\n", test3);
    
    // Debug: Check if semaphore flags are signaled in correct order
    int i = 0;
    while(true) {
        log_info( "Requesting new client to respond...\n");
        sem_post(sf_client.sem); 
        log_info( "Awaiting client response...");
        sem_wait(sf_server.sem);   
        log_info( "Processing CF Client...");

        test1 = sem_trywait(sf_init.sem);
        if (test1) {
            sem_wait(sl_init.sem);
            sem_post(sl_init.sem);
        }
        // test2 = sem_trywait(sf_samples.sem);
        if (sem_trywait(sf_samples.sem) == 0) {
            log_info( "Aquiring sample semlock...");
            sem_wait(sl_samples.sem);
            sleep(1);
            sem_post(sl_samples.sem);
            log_info( "Samples & Clr Freq processed...\n");
        }
        // log_info("checking clr_freq");
        // test3 = sem_trywait(sf_clrfreq.sem);
        if (sem_trywait(sf_clrfreq.sem) == 0) {           
            // Lock Write Clear Freq Data
            log_info( "Aquiring Semaphore Locks...");
            sem_wait(sl_clrfreq.sem);
            sem_wait(sl_samples.sem);
            log_info( "Writing clear frequency data to Shared Memory...");
            
            // Read beam num
            // write clr freq
            
            log_info( "clrfreq_shm written...");
            sem_post(sl_samples.sem);
            sem_post(sl_clrfreq.sem);
            sem_post(sf_clrfreq.sem);
            log_info( "Processed Clear Freq Request successfully...\n");
        }
        
        // log_info("sf was recieved if 0: %d %d %d %d", test, test1, test2, test3);
        sleep(1);
    }

    return;
};

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
    spectra_storage = fftw_alloc_complex(radar_num * STORAGE_NUM * total_beams * samples_num);
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
 * @brief  Updates the active antennas based on the antenna_list.
 * @note   
 * @param  active_antennas: Array of active antennas
 * @param  antenna_list: List of antennas to be activated
 * @param  num_antennas: Number of antennas in the list
 * @retval None
 */
void update_active_antennas(bool active_antennas[], int antenna_list[], int num_antennas) {
    // Set the active antennas based on the antenna_list
    for (int i = 0; i < num_antennas; i++) {
        if (antenna_list[i] >= 0 && antenna_list[i] < STATIC_ANTENNA_NUM) {
            active_antennas[antenna_list[i]] = true;
        } else {
            log_error("Invalid antenna index: %d", antenna_list[i]);
        }
    }
}


/**
 * @brief  Flags reserved frequencies from the radar table.
 * @note   
 * @param  channel_id: Channel ID number
 * @param  radar_num: Number of radars
 * @param  clr_bands: Array of clear frequency bands
 * @param  clr_range: Array of clear frequency ranges
 * @retval None
 */
void flag_reserved_freqs(int radar_id, int channel_id, int radar_num, freq_band clr_bands[6], int clr_range[2])
{
    bool has_valid_clr_band = false;

    log_info("Reserving clr_bands from and into Radar Table...");
    // For each clr_band in the Radar Table, compare w/ new clr_bands to see if already reserved
    for (int r_idx = 0; r_idx < radar_num; r_idx++) {
        for (int c_idx = 0; c_idx < STATIC_CHANNEL_NUM; c_idx++) {
            radar_freq_data channel_data = radar_table[r_idx][c_idx];

            // Skip current channel's reservation
            if (c_idx == channel_id && r_idx == radar_id) {
                log_info("[TCS] Skipping current channel reservation...");
                log_info("      Clr Freq Band[radar#%d][channel#%d] | %dHz -- Noise: %f -- %dHz |", 
                    r_idx, c_idx, channel_data.clr_band.f_start, channel_data.clr_band.noise, channel_data.clr_band.f_end);
                continue;
            }
            
            // Scan through clr_bands for intersections
            for (int i = 0; i < CLR_BANDS_MAX; i++) {
                // Check if the clr_band is valid
                if (clr_bands[i].f_start < clr_bands[i].f_end && clr_bands[i].noise < RAND_MAX) {
                    has_valid_clr_band = true;
                }

                // Check any for intersection between identified clr_bands and radar_table resevervation
                if (clr_bands[i].f_start >= channel_data.clr_band.f_start &&
                    clr_bands[i].f_start <  channel_data.clr_band.f_end ||
                    clr_bands[i].f_end   >= channel_data.clr_band.f_start &&
                    clr_bands[i].f_end   <  channel_data.clr_band.f_end) 
                {
                    // If reserved, flag clr_band as already selected
                    log_trace("     Conflict found with [radar#%d][channel#%d]", r_idx, c_idx);
                    clr_bands[i].is_selected = true;
                }
            }
        }
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
    log_info( "Done initializing Array Configuration...");


    // Allocate temp mem for shm varibles
    // temp_samples = (fftw_complex **)fftw_malloc(ANTENNA_NUM * sizeof(fftw_complex *));
    temp_samples = fftw_alloc_complex(ANTENNA_NUM * SAMPLES_NUM);
    if (temp_samples == NULL) {
        log_fatal("Error allocating memory for temp_samples pointers");
        perror("Error allocating memory for temp_samples pointers");
        exit(EXIT_FAILURE);
    }

    spectra_storage = fftw_alloc_complex(radar_num * STORAGE_NUM * beam_total * SAMPLES_NUM);
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

    freq_band clr_bands[CLR_BANDS_MAX];
    for (int clr_idx = 0; clr_idx < CLR_BANDS_MAX; clr_idx++) {
        clr_bands[clr_idx].f_start = 0;
        clr_bands[clr_idx].f_end = 0;
        clr_bands[clr_idx].noise = 0;
        clr_bands[clr_idx].is_selected = false;
    }
    
    clr_storage_sizes[0] = radar_num;
    clr_bands_storage = (freq_band ***)malloc(radar_num * sizeof(freq_band **));
    if (clr_bands_storage == NULL) {
        log_fatal( "Error allocating memory for clr_bands_storage radar pointers");
        perror("Error allocating memory for clr_bands_storage radar pointers");
        exit(EXIT_FAILURE);
    }
    for (int i = 0; i < radar_num; i++) {
        clr_bands_storage[i] = (freq_band **)malloc(CLR_STORAGE_NUM * sizeof(freq_band *));
        if (clr_bands_storage[i] == NULL) {
            log_fatal( "Error allocating memory for clr_bands_storage pointers");
            perror("Error allocating memory for clr_bands_storage pointers");
            exit(EXIT_FAILURE);
        }
        for (int j = 0; j < CLR_STORAGE_NUM; j++) {
            clr_bands_storage[i][j] = (freq_band *)malloc(CLR_BANDS_MAX * sizeof(freq_band));
            if (clr_bands_storage[i][j] == NULL) {
                log_fatal( "Error allocating memory for clr_bands_storage elements");
                perror("Error allocating memory for clr_bands_storage elements");
                exit(EXIT_FAILURE);
            }
            memset(clr_bands_storage[i][j], 0, CLR_BANDS_MAX * sizeof(freq_band));
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
    int clr_storage_i[STATIC_RADAR_NUM] = {0};
    int tcs_storage_i[STATIC_RADAR_NUM] = {0};
    bool is_tcs_ready[STATIC_RADAR_NUM] = {false};
    int clr_range[STATIC_RADAR_NUM][2] = {0};
    freq_band selected_clr_band = {0};
    
    
    // Parameters for Reading Restricted Frequencies
    char restrict_file[255] = "";
    char ststr[SITE_ID_ELEM + 1] = {0}; 
    char new_site_id[SITE_ID_ELEM + 1] = {0};
    char *rst_path = getenv("RSTPATH");
    if (rst_path == NULL) {
        log_fatal( "$RSTPATH not found. Restrict Freq file is inaccessible.\n");
        perror("Error: $RSTPATH not found");
        exit(EXIT_FAILURE);
    }

    // Debug: Check Semaphore Flag order
    // flag_debug();

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

            // Read Meta Data
            // if ( *(double*) (meta_obj.shm_ptr) != 0) {

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
                        is_tcs_ready[r_idx] = false;
                        tcs_storage_i[r_idx] = 0;
                    }
                    
                    realloc_storage(samples_num, beam_total, radar_num, avg_ratio);

                    // log_info( "Reinitializing TCS FFTW plan...");
                    // cleanup_storage_fft();
                    // init_storage_fft(samples_num, beam_total);
                }
                

                for (int j = 0; j < meta_data.num_antennas; j++) {
                    log_debug("    antenna_list[%d]: %d", j, meta_data.antenna_list[j]);
                }
                log_debug("     num_antennas: %d", meta_data.num_antennas);
                log_debug("     num_samples : %d", meta_data.number_of_samples);
                log_debug("     fcenter: passed during sample DT");
                log_debug("     rf_rate     : %d", meta_data.usrp_rf_rate);
                log_debug("     x_spacing   : %f", meta_data.x_spacing);
            // }

            /// Read Restricted Frequency (by grabbing site ID then reading its restricted freq file)
            log_debug( "Site ID reading...");
            read_site_id_data(new_site_id, site_id_obj.shm_ptr, SITE_ID_ELEM);
            log_debug("    Site ID: %s", ststr);
            log_debug("    New Site ID: %s", new_site_id);
    
            // If first client or new ststr, proceed to read in ststr and Restrict File
            if (strcmp(new_site_id, ststr) != 0) {
                log_info( "Site ID assigned, getting site's Resticted Frequencies ...");
                strncpy(ststr, new_site_id, SITE_ID_ELEM);
                ststr[SITE_ID_ELEM] = '\0'; 
                int str_f_result = 0; 

                // Get site specific restrict file and join with path
                if (strcmp(new_site_id,"lab") != 0) {
                    log_info( "Using /site.%s/restrict.dat.inst in ststr\n", ststr);
                    // str_f_result = asprintf(&restrict_file,"%s/tables/superdarn/site/site.%s/restrict.dat.inst",rst_path,ststr);
                    str_f_result = snprintf(restrict_file, sizeof(restrict_file), "%s/tables/superdarn/site/site.%s/restrict.dat.inst", rst_path, ststr);
                    if (str_f_result < 1) {
                        log_error( " site path format failed");
                        return 1;
                    }
                }

                // Default: Get lab testing restrict file
                else {
                    log_warn("WARNING: Parameter \'ststr\' is missing or set to a \"lab\" setting!");
                    strcpy(restrict_file, "/home/df/Desktop/PSU-SuperDARN/SuperDARN_MSI_ROS/linux/home/radar/ros.3.6/tables/superdarn/site/site.sys/restrict.dat.inst\0");
                }

                log_info("Using restrict file path: %s\n", restrict_file);
                read_restrict(restrict_file, restricted_freq, &restricted_num);
            }

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

            sem_post(sl_samples.sem);

            // Store Spectra Data
            log_info( "Storing Spectra Data...");
            if (tcs_storage_i[cur_radar] < STORAGE_NUM) {

                // If clr_range is not set, default clr_range to usrp_fcenter -/+ 0.5 * usrp_rf_rate
                if (clr_range[0][0] == 0 && clr_range[0][1] == 0) {
                    log_info( "Setting default clr_range...");
                    for (int i = 0; i < STATIC_RADAR_NUM; i++) {
                        clr_range[i][0] = (meta_data.usrp_fcenter * 1000 - (0.5 * meta_data.usrp_rf_rate)) / 1000;
                        clr_range[i][1] = (meta_data.usrp_fcenter * 1000 + (0.5 * meta_data.usrp_rf_rate)) / 1000;
                    }
                    log_info( "Default clr_range set to %d -- %d", clr_range[0][0], clr_range[0][1]);
                }
                
                // Fill Spectra Storage
                process_all_beamformed_spectras(
                        temp_samples,
                        clr_range[cur_radar], 
                        sample_sep, 
                        restricted_freq, 
                        restricted_num,
                        &meta_data,
                        array_config,
                        &(spectra_storage[
                            cur_radar * STORAGE_NUM * beam_total * samples_num + 
                            tcs_storage_i[cur_radar] * beam_total * samples_num
                        ])
                    );

                log_info( "[TCS] Processed Radar[%d]'s spectra_storage[%d/%d] successfully...", cur_radar, tcs_storage_i[cur_radar] + 1, STORAGE_NUM);
                tcs_storage_i[cur_radar] += 1;

                if (tcs_storage_i[cur_radar] >= STORAGE_NUM) {
                    log_info( "[TCS] Radar[%d] Storage is now Ready...", cur_radar);
                    is_tcs_ready[cur_radar] = true;
                    tcs_storage_i[cur_radar] = 0;
                } 
            }

            log_info( "Stored Samples successfully...");
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
                read_single_int(&cur_channel, channel_id_obj.shm_ptr);
                log_debug("    channel_id: %d", cur_channel);

                if (cur_channel >= STATIC_CHANNEL_NUM || cur_channel < 0) {
                    log_error( "ERROR: Channel ID out of range");
                    log_error( "ERROR: There is likely an error on the CFS client-side, please close and restart all related processes.");
                    perror("ERROR: Channel ID out of range");
                    exit(EXIT_FAILURE);
                }
            }

            // Read Clear Range
            if (*(int*) (clr_range_obj.shm_ptr) != 0) {
                int old_clr_range[2] = {clr_range[cur_radar][0], clr_range[cur_radar][1]};

                log_debug( "Clear Range reading...");
                read_int(clr_range[cur_radar], clr_range_obj.shm_ptr, 2);

                // If clr_range is in kHz, convert to Hz
                if (clr_range[cur_radar][0] < 100000 || clr_range[cur_radar][1] < 100000) {
                    clr_range[cur_radar][0] = clr_range[cur_radar][0] * 1000;
                    clr_range[cur_radar][1] = clr_range[cur_radar][1] * 1000;
                    log_debug("    clr_range: %d -- %d Hz", clr_range[cur_radar][0], clr_range[cur_radar][1]);
                }

                // If a radar's clr_range changed, reset TCS for that radar
                log_debug("    old_clr_range: %d -- %d", old_clr_range[0], old_clr_range[1]);
                if (clr_range[cur_radar][0] != old_clr_range[0] || clr_range[cur_radar][1] != old_clr_range[1]) {
                    log_info( "Radar#%d's Clear Range changed...", cur_radar);
                    log_info("    old_clr_range: %d -- %d", old_clr_range[0], old_clr_range[1]);
                    log_info("    clr_range: %d -- %d", clr_range[cur_radar][0], clr_range[cur_radar][1]);


                    is_tcs_ready[cur_radar] = false;
                    tcs_storage_i[cur_radar] = 0;
                }
            }
            
            log_info( "    avg_ratio: %d", avg_ratio);
            log_info( "    delta_f: %d", (int) (meta_data.usrp_rf_rate / samples_num) );

            // Clear Freq Processing
            // If TCS is not ready, process new clrfreq per unique beam request
            if (is_tcs_ready[cur_radar] == false) {
                // Process basic Clear Search
                log_info( "Clr Freq @ Beam #%d ...", cur_beam);
                log_info( "Starting Clear Freq Search...");
                clear_freq_search(
                    temp_samples, 
                    clr_range[cur_radar],
                    cur_beam,
                    sample_sep,
                    avg_ratio,
                    restricted_freq, 
                    restricted_num + RESERV_NUM,
                    meta_data,
                    array_config,
                    clr_bands                
                );
            }
            // If TCS ready, process beam-specific clr freq
            else {
                log_info( "[TCS] Clr Freq @ Beam #%d ...", cur_beam);
                process_avg_beam_spectra(
                    &(spectra_storage[cur_radar * STORAGE_NUM * beam_total * samples_num]),
                    avg_ratio, 
                    meta_data.number_of_samples,
                    cur_beam,
                    beam_total,
                    STORAGE_NUM,
                    &meta_data,
                    avg_beam_spectrum,
                    avg_freq_vector
                );
                log_trace( "    Avg Beam Spectrum done...");

                process_beam_clr_freq(
                    avg_beam_spectrum,
                    cur_beam,
                    clr_range[cur_radar],
                    sample_sep,
                    restricted_freq, 
                    restricted_num + RESERV_NUM,
                    avg_freq_vector,
                    (int) (meta_data.number_of_samples / avg_ratio),
                    &meta_data,
                    clr_bands
                );
                log_info( "[TCS] Clr Freq @ Beam #%d done...", cur_beam);
            }
            
            // // Flag intersecting freq bands from Radar Table
            // flag_reserved_freqs(cur_radar, cur_channel, radar_num, clr_bands, clr_range[cur_radar]);

            // Output Clear Freq Bands
            bool is_clr_band_found = false;
            for (int i = 0; i < CLR_BANDS_MAX; i++) {
                log_debug("Clear Freq Band[%d]: | %dHz -- Noise: %f -- %dHz |", i, clr_bands[i].f_start, clr_bands[i].noise, clr_bands[i].f_end);
                
                // Reserve the best avalible frequency band
                if (clr_bands[i].is_selected == false && is_clr_band_found == false) {
                    log_debug("    Reserving frequency into RadarTable...", i);
                    is_clr_band_found = true;
                    clr_bands[i].is_selected = true;
                    
                    // Reserve the frequency band 
                    radar_table[cur_radar][cur_channel].clr_band = clr_bands[i];
                    radar_table[cur_radar][cur_channel].clear_freq_range[0] = clr_range[0];
                    radar_table[cur_radar][cur_channel].clear_freq_range[1] = clr_range[1];
                    if (restricted_num + cur_radar * STATIC_CHANNEL_NUM + cur_channel >= RESTRICT_NUM) {
                        log_error("    ERROR: Reservation into restricted_freq failed due to overflow index!");
                        perror("ERROR: Reservation into restricted_freq failed due to overflow index!");
                        exit(EXIT_FAILURE);
                    }
                    restricted_freq[restricted_num + cur_radar * STATIC_CHANNEL_NUM + cur_channel] = clr_bands[i];

                    // Select the best frequency band
                    selected_clr_band = clr_bands[i];
                }

                // Flag abnormal clr_bands in log
                if (clr_bands[i].f_start == 0 || clr_bands[i].f_end == 0 || clr_bands[i].noise == 0 ||
                    clr_bands[i].f_start == RAND_MAX || clr_bands[i].f_end == RAND_MAX || clr_bands[i].noise == RAND_MAX) {
                    log_error("ERROR: Clear Freq Band[%d] is abnornal", i);
                    log_error("Clear Freq Band[%d]: | %dHz -- Noise: %f -- %dHz |", i, clr_bands[i].f_start, clr_bands[i].noise, clr_bands[i].f_end);
                    log_error("ERROR: There is likely a semaphore leak or error in CFS order of operations, please close and restart all related processes.");
                }
            }
            write_clrfreq_shm(selected_clr_band, clrfreq_obj.shm_ptr);

            // Log clear freq bands
            memcpy(clr_bands_storage[cur_radar][clr_storage_i[cur_radar]], clr_bands, CLR_BANDS_MAX * sizeof(freq_band));
            clr_storage_i[cur_radar]++;
            log_info( "Clr Freq Log: Radar[%d] @ %d/%d", cur_radar, clr_storage_i[cur_radar], CLR_STORAGE_NUM);
            if (clr_storage_i[cur_radar] >= CLR_STORAGE_NUM) {
                write_clr_log_csv(clr_bands_storage[cur_radar], clr_storage_i[cur_radar], cur_radar);
                clr_storage_i[cur_radar] = 0;
            }
            
            // Display TCS Storage Information
            log_info( "[TCS] Radar[%d] Storage [%d/%d]", cur_radar, tcs_storage_i[cur_radar] + 1, STORAGE_NUM);
            if (is_tcs_ready[cur_radar] == true) {
                log_info( "     [TCS] Radar[%d] ready...", cur_radar);
            }

            // Display Radar Table Information
            log_debug( "Radar Table Information:");
            for (int r_idx = 0; r_idx < radar_num; r_idx++) {
                for (int c_idx = 0; c_idx < STATIC_CHANNEL_NUM; c_idx++) {
                    if (radar_table[r_idx][c_idx].clr_band.f_start != 0 && radar_table[r_idx][c_idx].clr_band.f_end != 0) {
                        log_debug( "    Radar[%d] Channel[%d]: | %dHz -- Noise: %f -- %dHz |",
                            r_idx, c_idx, 
                            radar_table[r_idx][c_idx].clr_band.f_start, 
                            radar_table[r_idx][c_idx].clr_band.noise, 
                            radar_table[r_idx][c_idx].clr_band.f_end);
                    }
                }
            }

            // Synchronize data writes with program counter
            if (msync(clrfreq_obj.shm_ptr, CLR_BANDS_SHM_SIZE, MS_SYNC) == -1) {
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
}
