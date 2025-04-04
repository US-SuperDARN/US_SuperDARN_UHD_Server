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



// Build with the following flags:
// -lrt -pthread -lfftw3 -lm 



// Default Length of Variables (some dynamically change during runtime)
#define SAMPLES_NUM     10000
#define ANTENNA_NUM     16
#define SAMPLE_TIME     3                   // Time per Sample (in seconds)
#define STORAGE_TIME    60                  // Total time per Sample Storage Batch (in seconds)
#define STORAGE_NUM     (STORAGE_TIME / SAMPLE_TIME) // Total number of processed sample sets to store
#define META_ELEM       3                   // 4 = 5 - 1 (fcenter has unique obj)
#define RESTRICT_NUM    50                  // Number of restricted freq bands in the restrict.dat.inst
#ifndef CLR_BANDS_MAX
#define CLR_BANDS_MAX   6
#endif
#define CLR_STORAGE_NUM 10
#define CLR_STORE_FILEPATH "log/clr_band_storage/"
#define SITE_ID_ELEM    3                   // 3 = 3-letter identifier 

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
#define ACTIVE_CLIENTS_SHM_NAME "/active_clients"   // For debugging

#define SAMPLE_PARAM_NUM 4
#define RESTRICT_PARAM_NUM 2
#define PARAM_NUM 11

#define SEM_F_CLIENT    "/sf_client"               // For Sync and reserving client and server roles during data transfer
#define SEM_F_SERVER    "/sf_server"    
#define SEM_F_SAMPLES   "/sf_samples"
#define SEM_F_INIT      "/sf_init"           
#define SEM_F_CLRFREQ   "/sf_clrfreq"              // For multiple data transfers on single instance 
#define SEM_L_SAMPLES   "/sl_samples"              // For Data locking b/w write/reads
#define SEM_L_INIT      "/sl_init"                 // init = initialization
#define SEM_L_CLRFREQ   "/sl_clrfreq"
#define SL_NUM 3
#define SEM_NUM 8

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




// TODO: Read of specific data types 
// TODO: Calc all beam directions at recieve of samples
// TODO: Num of Radar {has table of all beam dirc {has Clr freq in beam direction}}} while sharing restricting assigned freq 
// TODO: Rewrite of usrp sample send/clr freq request timing logic 

// Semaphores Locks prevent race conditions
// Semaphore Flags allow client and server to signal specific data transfers
struct semaphore sf_client   = {SEM_F_CLIENT,  NULL};
struct semaphore sf_server   = {SEM_F_SERVER,  NULL};
struct semaphore sf_samples  = {SEM_F_SAMPLES, NULL};
struct semaphore sf_init     = {SEM_F_INIT,    NULL};
struct semaphore sf_clrfreq  = {SEM_F_CLRFREQ, NULL};
struct semaphore sl_samples  = {SEM_L_SAMPLES, NULL};
struct semaphore sl_init     = {SEM_L_INIT,    NULL};
struct semaphore sl_clrfreq  = {SEM_L_CLRFREQ, NULL};
struct semaphore *semaphores[SEM_NUM] = {
    &sf_client,
    &sf_server,    
    &sf_samples,
    &sf_init,
    &sf_clrfreq,
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
    &client_num_obj,
};


int temp_ptrs_num = 0;
void **temp_ptrs;

fftw_complex **temp_samples = NULL;
int temp_sample_sizes[] = {
    ANTENNA_NUM,
    SAMPLES_NUM,
};
fftw_complex ***samples_storage = NULL;
int samples_storage_sizes[] = {
    STORAGE_NUM,
    ANTENNA_NUM,
    SAMPLES_NUM,
};
int samples_storage_i = 0;
freq_band **clr_bands_storage = NULL;
int clr_storage_i = 0;
int clr_storage_sizes[] = {
    CLR_STORAGE_NUM,
    CLR_BANDS_MAX,
};
sample_meta_data meta_data = {0};


void add_ptr(void **ptr) {
    // Check for pre-existing ptr
    for (int i = 0; i < temp_ptrs_num; i++) {
        if (temp_ptrs[i] == ptr) {
            // printf("Pointer already exists in temp_ptrs[%d]: %p\n", i, temp_ptrs[i]);
            return;
        }
    }

    // Allocate memory for the new pointer
    void *tmp = NULL;
    tmp = realloc(temp_ptrs, (temp_ptrs_num + 1) * sizeof(void *));
    if (tmp == NULL) {
        perror("Error reallocating memory for temp_ptrs");
        exit(EXIT_FAILURE);
    }
    temp_ptrs = tmp;
    temp_ptrs[temp_ptrs_num] = ptr; 
    // printf("  temp_ptrs[%d] = %p -> %p\n", temp_ptrs_num, temp_ptrs[temp_ptrs_num], *(void **)temp_ptrs[temp_ptrs_num]);
    temp_ptrs_num++;    
}

void update_ptr(void *old_ptr, void *new_ptr) {
    // Check if the old pointer exists in the array
    for (int i = 0; i < temp_ptrs_num; i++) {
        if (temp_ptrs[i] == old_ptr) {
            // printf("Updating temp_ptrs[%d] from %p to %p\n", i, temp_ptrs[i], new_ptr);
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
    // printf("free_nested_fftw_ptr: %p, nest_depth: %d,", ptr, nest_depth);
    // printf("sizes: ");
    // for (int i = 0; i < nest_depth; i++) {
    //     if (i == nest_depth - 1) {
    //         printf("%d\n", sizes[i]);
    //     } else printf("%d ", sizes[i]);
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
    
    printf("free_nested_ptr: %p, nest_depth: %d,", ptr, nest_depth);
    printf("sizes: ");
    for (int i = 0; i < nest_depth; i++) {
        if (i == nest_depth - 1) {
            printf("%d\n", sizes[i]);
        } else printf("%d ", sizes[i]);
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
    printf("Contents of temp_ptrs:\n");
    for (int i = 0; i < temp_ptrs_num; i++) {
        printf("  temp_ptrs[%d] = %p -> %p\n", i, temp_ptrs[i], *(void **)temp_ptrs[i]);
    }
}

void read_sample_shm(fftw_complex **temp_samples, void *samples_shm_ptr, int antenna_num, int samples_num) {
    int *s_ptr = (int *) samples_shm_ptr;
    
    // Store sample data into complex form
    for (int i = 0; i < antenna_num; i++)
    {
        for (int j = 0; j < samples_num; j++) {
            temp_samples[i][j] = s_ptr[i * samples_num + j * 2] + I * s_ptr[i * samples_num + j * 2 + 1];

            // Debug: Print 5 complex of each antenna batch
            // if (j < 4 || j > samples_num - 4 || j == 2499) {
            //     printf("shm[%d]      =   %d + i%d\n", i * samples_num + j, ((int*) samples_shm_ptr)[i * samples_num + j * 2], ((int*) samples_shm_ptr)[i * samples_num + j * 2 + 1]);
            //     printf("vs\n");
            //     printf("temp_samples[%d][%d] =  %f + i%f\n\n", i, j, creal(temp_samples[i][j]), cimag(temp_samples[i][j]));
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
            if (VERBOSE && i < 2) printf("    read_int: %d\n", result[i]);
        }
    } else {
        printf("Error: Use read_single_int() for single variables\n");
    }
}

void read_int_single(int result, int *shm_ptr) {
    result = *shm_ptr;
    if (VERBOSE) printf("    read_int: %d\n", result);
}

void read_double(double *result, void *shm_ptr, int elem_num) {
    double *ref_ptr = (double *) shm_ptr;

    if (elem_num > 1) {
        for (int i = 0; i < elem_num; i++) {
            result[i] = ref_ptr[i];
            if (VERBOSE && i < 2) printf("    read_double: %f\n", result[i]);
        }
    } else {
        printf("Error: Use read_single_double() for single variables\n");
    }
}

void read_meta_data(sample_meta_data *result, void *shm_ptr, int ant_num) {
    printf("starting meta read..\n");
    double *ref_ptr = (double *) shm_ptr;
    int *antenna_ptr = (int *) result->antenna_list;

    
    // Loop thru all meta elements and copy into result
    for (int i = 0; i < (META_ELEM); i++) {
        printf("reading[%d]: %f\n", i, ref_ptr[i]);
        
        if      (i == (0)) result->number_of_samples = (int) ref_ptr[i];
        else if (i == (1)) result->x_spacing = ref_ptr[i];
        else if (i == (2)) result->usrp_rf_rate = (int) ref_ptr[i];
        
        // if (VERBOSE && i < (ant_num + 2)) printf("    read_meta: %f\n", ref_ptr[i]);
    }

    printf("Fin reading meta_elem; reading antenna_list...\n");
    
    // Read in antenna_list elements
    for (int i = META_ELEM; i < (ant_num + META_ELEM); i++) {
        if (VERBOSE) printf("    reading antenna_list: %d\n", antenna_ptr[i - META_ELEM]);
        if (VERBOSE) printf("    reading ref_ptr     : %d\n", (int) ref_ptr[i]);
        antenna_ptr[i - META_ELEM] = (int) ref_ptr[i];
    }
}

void read_site_id_data(char *result, void *shm_ptr, int id_num) {
    char *ref_ptr = (char *) shm_ptr;

    for (int i = 0; i < id_num; i++) {
        *(result + i) = ref_ptr[i];
        if (VERBOSE) printf("   read_site_id_data[%d]: %c\n", i, result[i]);
    }
    result[id_num] = '\0';
    printf("    read_site_id_data: %s\n", result);
}

void read_single_int(int *result, void *shm_ptr) {
    *result = *(int *) shm_ptr;
    if (VERBOSE) printf("   read_s_int: %d\n", *result);
}

void read_single_double(double *result, void *shm_ptr){
    *result = *(double *) shm_ptr;
    if (VERBOSE) printf("   read_s_int: %f\n", *result);
}

/**
 * @brief  Writes Clear Freq Bands to its shared memory pointer.
 * @note   
 * @param  *clr_bands: Clear Frequency Bands
 * @param  *ptr: Shared Memory Pointer for Clear Frequency Bands
 * @retval None
 */
void write_clrfreq_shm(freq_band *clr_bands, int *ptr) {
    int elements_per_band = 3;
    for (int i = 0; i < CLR_BANDS_MAX; i++) {
        if (clr_bands[i].is_selected == false) {
            clr_bands[i].is_selected = true;
            ptr[i * elements_per_band]      = clr_bands[i].f_start;
            ptr[i * elements_per_band + 1]  = clr_bands[i].noise;
            ptr[i * elements_per_band + 2]  = clr_bands[i].f_end;

            printf("[Frequency Server] Sending the following Clear Frequency: \n");
            printf("    Clear Freq Band[%d][%s]: | %dHz -- Noise: %f -- %dHz |\n", 
                i, clr_bands[i].is_selected ? "Selected" : "Free", clr_bands[i].f_start, clr_bands[i].noise, clr_bands[i].f_end
            );
            break;
        }
    }
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
    // print_temp_ptrs();
    printf("[Frequency Server] Cleaning all semaphores and SHM objects...\n");

    for (int i = 0; i < SEM_NUM; i++) clean_sem(*semaphores[i]);
    printf("[Frequency Server] Cleaned all semaphores ...\n");

    for (int i = 0; i < PARAM_NUM; i++) clean_obj(*objects[i]);
    printf("[Frequency Server] Cleaned all objects ...\n");

    // Free fftw ptrs
    free_nested_fftw_ptr(temp_samples, 2, temp_sample_sizes);
    printf("[Frequency Server] Cleaned 1/2 fftw_ptrs ...\n");
    free_nested_fftw_ptr(samples_storage, 3, samples_storage_sizes);
    printf("[Frequency Server] Cleaned 2/2 fftw_ptrs ...\n");

    printf("[Frequency Server] Cleaned all fftw pointers ...\n");
    
    // Free ptrs
    free_nested_ptr(clr_bands_storage, 2, clr_storage_sizes);
    printf("[Frequency Server] Cleaned clr_bands_storage ...\n");

    // int temp_sizes[] = {1 ,temp_ptrs_num};
    // free_nested_ptr(temp_ptrs, 2, temp_sizes);
    for (int i = 0; i < temp_ptrs_num; i++) {
        if (*(void **)temp_ptrs[i] != NULL) { //temp_ptrs[i] != NULL && 
            // printf("Freeing temp_ptrs[%d/%d]: %p\n", i, temp_ptrs_num, *(void **)temp_ptrs[i]);
            free(*(void **)temp_ptrs[i]);
            *(void **)temp_ptrs[i] = NULL; 
        } else {
            printf("Skipping invalid or NULL pointer at temp_ptrs[%d]\n", i);
        }
    }
    free(temp_ptrs);


    printf("[Frequency Server] Cleaned all pointers ...\n");
}

/**
 * @brief  Catch exit signals and exits gracefully by calling to deallocate all service parameters.
 * @note   
 * @param  sig: Caught signal
 * @retval None
 */
void handle_sig(int sig) {
    printf("\n[Frequency Server] Caught signal %d, cleaning up and exiting...\n", sig);
    cleanup();

    // Prompt exit to terminal  
    printf("[Frequency Server] Main processes and communication terminated.\n"
           "Goodbye.\n");
           
    exit(sig);
}

void write_clr_log_csv(freq_band **clr_storage, int clr_num) {
    // Timestamp Variables
    time_t raw_time;
    struct tm *time_info;
    int buffer_size = 100;
    char timestamp[buffer_size];
    char name[buffer_size]; 

    // Generate timestamp
    time(&raw_time);
    time_info = localtime(&raw_time);
    strftime(timestamp, buffer_size, "%Y.%m.%d_%H:%M:%S", time_info);
    snprintf(name, sizeof(name), "log/clr_freq/clrlog_%s.csv", timestamp);

    // Generate clear log file
    FILE *file = fopen(name, "w");
    if (file == NULL) {
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
            // printf("Clear Freq Band[%d]: | %dHz -- Noise: %f -- %dHz |\n", i, clr_storage[clr_batch_idx][i].f_start, clr_storage[clr_batch_idx][i].noise, clr_storage[clr_batch_idx][i].f_end);
            
            // Special: Print Clear Freq Range on Line 0
            if (i == 0) fprintf(file, "%d,%d,%f,%d,%d\n", clr_bands[i].f_start, clr_bands[i].f_end, clr_bands[i].noise,clr_start,clr_end);
            else fprintf(file, "%d,%d,%f\n", clr_bands[i].f_start, clr_bands[i].f_end, clr_bands[i].noise);
        }
    }

    fclose(file);
}

void flag_debug() {

    printf("[FLAG DEBUGGING] All functionality except for semaphore flags is absent!\n");
    printf("[FLAG DEBUGGING] Comment out the flag_debug() function to revert to standard functionality.\n\n");

    // Debug: Verify that the flags are down 
    int test = sem_trywait(sf_server.sem);
    printf("\nsf_server was recieved if 0: %d\n", test);
    int test1 = sem_trywait(sf_init.sem);
    printf("sf_int was recieved if 0: %d\n", test1);
    int test2 = sem_trywait(sf_samples.sem);
    printf("sf_samples was recieved if 0: %d\n\n", test2);

    printf("[Frequency Server] Requesting new client to respond...\n\n");
    sem_post(sf_client.sem); 
    
    // Debug: Check if semaphore flags are signaled in correct order
    int i = 0;
    while(true) {
        test = sem_wait(sf_server.sem);
        test1 = sem_trywait(sf_init.sem);
        if (test1) {
            sem_wait(sl_init.sem);
            sem_post(sl_init.sem);
        }
        test2 = sem_trywait(sf_samples.sem);
        if (test2) {
            sem_wait(sl_samples.sem);
            sem_post(sl_samples.sem);
        }
        
        // Let the client finish so to not manually close it
        if (i == 0 && test == 0) {
            printf("setting clr_freq flags for the client to finish\n");
            sem_post(sl_clrfreq.sem);
            sem_post(sf_clrfreq.sem);
            i++;
        }
        
        printf("sf_int was recieved if 0: %d %d %d\n", test, test1, test2);
        sleep(1);
    }

    return;
};


int main() {
    // Setup Signal Handler (catches ctrl+c and termination? to quit safely)
    signal(SIGTERM, handle_sig);
    signal(SIGINT, handle_sig);
    signal(SIGSEGV, handle_sig);


    printf("[Frequency Server] Pre-Cleaning...\n\n");
    cleanup();
    

    // Open Shared Memory Object
    printf("[Frequency Server] Initializing Shared Memory Object...\n");
    for (int i = 0; i < PARAM_NUM; i++){
        objects[i]->shm_fd = shm_open(objects[i]->name, O_CREAT | O_RDWR, 0666);
        if (objects[i]->shm_fd == -1) {
            printf("[Frequency Server] shm_open failed for %s\n", objects[i]->name);
            exit(EXIT_FAILURE);
        }
    }
    printf("[Frequency Server] Created Shared Memory Objects...\n");
    
    // Set Size of Shared Memory Object
    for (int i = 0; i < PARAM_NUM; i++){
        if (ftruncate(objects[i]->shm_fd, objects[i]->size) == -1) {
            perror("[Frequency Server] ftruncate failed\n");
            exit(EXIT_FAILURE);
        }
    }
    // Request Block of Memory
    printf("[Frequency Server] Requesting Shared Memory Cache...\n");    
    for (int i = 0; i < PARAM_NUM; i++) {
        objects[i]->shm_ptr = mmap(0, objects[i]->size, PROT_WRITE | PROT_READ, MAP_SHARED, objects[i]->shm_fd, 0);
        if (objects[i]->shm_ptr == MAP_FAILED) {
            printf("[Frequency Server] Memory Mapping failed for %s\n", objects[i]->name);
            exit(EXIT_FAILURE);
        }
    }
    printf("[Frequency Server] Memory successfully cached...\n");

    // Initialize SHM to zero
    printf("[Frequency Server] Initializing Shared Memory to zero...\n");
    for (int i = 0; i < PARAM_NUM; i++) {
        memset(objects[i]->shm_ptr, 0, objects[i]->size);
    }
    printf("[Frequency Server] Successfully initialized SHM to zero...\n");


    // Open Semaphores for synchronization     
    printf("[Frequency Server] Opening Communication Semaphores...\n");    
    for (int i = 0; i < SEM_NUM; i++) {
        if (i < (SEM_NUM - SL_NUM)) semaphores[i]->sem = sem_open(semaphores[i]->name, O_CREAT, 0666, 0);
        else semaphores[i]->sem = sem_open(semaphores[i]->name, O_CREAT, 0644, 1);
        if (semaphores[i]->sem == SEM_FAILED) {
            printf("[Frequency Server] \"%s\" sem_open failed.\n", semaphores[i]->name);
            exit(EXIT_FAILURE);    
        } 
    }
    printf("[Frequency Server] Done Initializing...\n\n");
    


    // Allocate temp mem for shm varibles
    temp_samples = (fftw_complex **)fftw_malloc(ANTENNA_NUM * sizeof(fftw_complex *));
    if (temp_samples == NULL) {
        perror("Error allocating memory for temp_samples pointers");
        exit(EXIT_FAILURE);
    }
    for (int i = 0; i < ANTENNA_NUM; i++) {
        temp_samples[i] = (fftw_complex *)fftw_malloc(SAMPLES_NUM * sizeof(fftw_complex));
        if (temp_samples[i] == NULL) {
            perror("Error allocating memory for temp_samples elements");
            exit(EXIT_FAILURE);
        }
    }

    samples_storage = (fftw_complex ***)fftw_malloc(STORAGE_NUM * sizeof(fftw_complex **));
    if (samples_storage == NULL) {
        perror("Error allocating memory for samples_storage pointers");
        exit(EXIT_FAILURE);
    }
    for (int i = 0; i < STORAGE_NUM; i++) {
        samples_storage[i] = (fftw_complex **)fftw_malloc(ANTENNA_NUM * sizeof(fftw_complex *));
        if (samples_storage[i] == NULL) {
            perror("Error allocating memory for samples_storage's antenna pointers");
            exit(EXIT_FAILURE);
        }
        for (int j = 0; j < ANTENNA_NUM; j++) {
            samples_storage[i][j] = (fftw_complex *)fftw_malloc(SAMPLES_NUM * sizeof(fftw_complex));
            if (samples_storage[i][j] == NULL) {
                perror("Error allocating memory for samples_storage elements");
                exit(EXIT_FAILURE);
            }
        }
    }

    int restricted_num = RESTRICT_NUM;      // Number of Restricted Freqs at runtime varies depending on site
    freq_band restricted_freq[restricted_num];
    for (int i = 0; i < restricted_num; i++) {
        restricted_freq[i].f_start = 0;
        restricted_freq[i].f_end = 0;
    }
    // restricted_freq = (freq_band *)malloc(restricted_num * sizeof(freq_band));
    // if (restricted_freq == NULL) {
    //     perror("Error allocating memory for restricted_freq elements");
    //     exit(EXIT_FAILURE);
    // }
    // add_ptr((void **)&restricted_freq);

    freq_band *clr_bands = NULL;
    clr_bands = (freq_band *)malloc(CLR_BANDS_MAX * sizeof(freq_band));
    if (clr_bands == NULL) {
        perror("Error allocating memory for clr_bands elements");
        exit(EXIT_FAILURE);
    }
    add_ptr((void **)&clr_bands);

    clr_bands_storage = (freq_band **)malloc(CLR_STORAGE_NUM * sizeof(freq_band *));
    if (clr_bands_storage == NULL) {
        perror("Error allocating memory for clr_bands_storage pointers");
        exit(EXIT_FAILURE);
    }
    for (int i = 0; i < CLR_STORAGE_NUM; i++) {
        clr_bands_storage[i] = (freq_band *)malloc(CLR_BANDS_MAX * sizeof(freq_band));
        if (clr_bands_storage[i] == NULL) {
            perror("Error allocating memory for clr_bands_storage elements");
            exit(EXIT_FAILURE);
        }
    }

    int* clr_range = malloc(2 * sizeof(int));
    add_ptr((void **)&clr_range);
    int beam_num = 0;
    int sample_sep = 0;
    int old_antenna_num = ANTENNA_NUM;
    int samples_num = SAMPLES_NUM;
            
    // Parameters for Reading Restricted Frequencies
    char restrict_file[255] = "";
    char ststr[SITE_ID_ELEM + 1] = {0}; //(char*) malloc((SITE_ID_ELEM + 1) * sizeof(char));
    char new_site_id[SITE_ID_ELEM + 1] = {0}; // = (char*) malloc((SITE_ID_ELEM + 1) * sizeof(char));
    char *rst_path = getenv("RSTPATH");
    if (rst_path == NULL) {
        perror("[Frequency Server] CRITICAL ERROR: $RSTPATH not found. Restrict Freq file is inaccessible.\n\n\n");
        exit(EXIT_FAILURE);
    }

    // Continuously process clients via shared memory
    while (1) {
        printf("[Frequency Server] Requesting new client to respond...\n\n");
        sem_post(sf_client.sem); 
        
        printf("[Frequency Server] Awaiting client response...\n");
        sem_wait(sf_server.sem);   

        double t1,t2;
        t1 = clock();

        // If initialization data flagged, read and store data
        if (sem_trywait(sf_init.sem) == 0){
            printf("[Frequency Server] Awaiting initialization data unlock...\n");
            sem_wait(sl_init.sem);
            printf("[Frequency Server] Initialization data read...\n");

            // Read Sample Separation
            if ( *(int*) (sample_sep_obj.shm_ptr) != 0) {
                printf("[Frequency Server] Sample Separation reading...\n");
                read_single_int(&sample_sep, sample_sep_obj.shm_ptr);
                printf("    sample_sep: %d\n", sample_sep);
            }

            // Read Meta Data
            // if ( *(double*) (meta_obj.shm_ptr) != 0) {

                // Read Antenna number
                printf("[Frequency Server] Antenna Number reading...\n");
                read_single_int(&meta_data.num_antennas, antenna_obj.shm_ptr);

                // If new num_antennas, Reallocate meta SHM 
                if (meta_data.num_antennas != old_antenna_num) {
                    printf("num of antenna: %d\n", meta_data.num_antennas);
                    
                    // Set Size of Shared Memory Object
                    meta_obj.size = (meta_data.num_antennas + META_ELEM) * sizeof(double);
                    if (ftruncate(meta_obj.shm_fd, meta_obj.size) == -1) {
                        perror("[Frequency Server] ftruncate failed\n");
                        exit(EXIT_FAILURE);
                    }

                    // Request Block of Memory
                    printf("[Frequency Server] Requesting Shared Memory Cache...\n");                    
                    meta_obj.shm_ptr = mmap(0, meta_obj.size, PROT_WRITE | PROT_READ, MAP_SHARED, meta_obj.shm_fd, 0);
                    if (meta_obj.shm_ptr == MAP_FAILED) {
                        printf("[Frequency Server] Memory Mapping failed for %s\n", meta_obj.name);
                        exit(EXIT_FAILURE);
                    }                    
                    printf("[Frequency Server] Meta Data successfully cached...\n");     

                    /// Sample Reallocation
                    temp_sample_sizes[0] = meta_data.num_antennas;
                    temp_sample_sizes[1] = samples_num;
                    // TODO: Record the num_antennas and calc mode to filter for 1min processing
                    // samples_storage_sizes[1] = meta_data.num_antennas;
                    // samples_storage_sizes[2] = samples_num;

                    // Set Size of Shared Memory Object
                    samples_obj.size = (meta_data.num_antennas) * samples_num * 2 * sizeof(int);
                    if (ftruncate(samples_obj.shm_fd, samples_obj.size) == -1) {
                        perror("[Frequency Server] ftruncate failed\n");
                        exit(EXIT_FAILURE);
                    }

                    // Request Block of Memory
                    printf("[Frequency Server] Requesting Shared Memory Cache...\n");                    
                    samples_obj.shm_ptr = mmap(0, samples_obj.size, PROT_WRITE | PROT_READ, MAP_SHARED, samples_obj.shm_fd, 0);
                    if (samples_obj.shm_ptr == MAP_FAILED) {
                        printf("[Frequency Server] Memory Mapping failed for %s\n", samples_obj.name);
                        exit(EXIT_FAILURE);
                    }                    
                    printf("[Frequency Server] Samples Data successfully cached...\n");    


                    // Temp_samples Reallocation
                    // Free previously allocated memory for temp_samples
                    if (temp_samples != NULL) {
                        for (int i = 0; i < old_antenna_num; i++) {
                            if (temp_samples[i] != NULL) {
                                fftw_free(temp_samples[i]);
                                temp_samples[i] = NULL;
                            }
                        }
                        fftw_free(temp_samples);
                        temp_samples = NULL;
                    }
                    printf("[Frequency Server] Freed old temp_samples memory...\n");
                    
                    // Allocate new memory for temp_samples
                    temp_samples = (fftw_complex **)fftw_malloc(meta_data.num_antennas * sizeof(fftw_complex *));
                    if (temp_samples == NULL) {
                        perror("Error reallocating memory for temp_samples pointers");
                        exit(EXIT_FAILURE);
                    }
                    for (int i = 0; i < meta_data.num_antennas; i++) {
                        temp_samples[i] = (fftw_complex *)fftw_malloc(samples_num * sizeof(fftw_complex));
                        if (temp_samples[i] == NULL) {
                            perror("Error allocating memory for temp_samples elements");
                            exit(EXIT_FAILURE);
                        }
                    } 
                    printf("[Frequency Server] Allocated new temp_samples memory...\n");

                    old_antenna_num = meta_data.num_antennas;
                }
                
                // Read Meta Data 
                printf("[Frequency Server] Meta Data reading...\n");
                read_meta_data(&meta_data, meta_obj.shm_ptr, meta_data.num_antennas);
                samples_num = meta_data.number_of_samples;

                for (int j = 0; j < meta_data.num_antennas; j++) {
                    printf("    antenna_list[%d]: %d\n", j, meta_data.antenna_list[j]);
                }
                printf("     num_antennas: %d\n", meta_data.num_antennas);
                printf("     num_samples : %d\n", meta_data.number_of_samples);
                printf("     fcenter: passed during sample DT\n");
                printf("     rf_rate     : %d\n", meta_data.usrp_rf_rate);
                printf("     x_spacing   : %f\n", meta_data.x_spacing);
            // }

            /// Read Restricted Frequency (by grabbing site ID then reading its restricted freq file)
            printf("[Frequency Server] Site ID reading...\n");
            read_site_id_data(new_site_id, site_id_obj.shm_ptr, SITE_ID_ELEM);
            printf("    Site ID: %s\n", ststr);
            printf("    New Site ID: %s\n", new_site_id);
    
            // If first client or new ststr, proceed to read in ststr and Restrict File
            if (strcmp(new_site_id, ststr) != 0) {
                printf("[Frequency Server] Site ID assigned, getting site's Resticted Frequencies ...\n");
                strncpy(ststr, new_site_id, SITE_ID_ELEM);
                ststr[SITE_ID_ELEM] = '\0'; 
                int str_f_result = 0; 

                // Get site specific restrict file and join with path
                if (strcmp(new_site_id,"lab") != 0) {
                    printf("[Frequency Server] Using /site.%s/restrict.dat.inst in ststr\n\n", ststr);
                    // str_f_result = asprintf(&restrict_file,"%s/tables/superdarn/site/site.%s/restrict.dat.inst",rst_path,ststr);
                    str_f_result = snprintf(restrict_file, sizeof(restrict_file), "%s/tables/superdarn/site/site.%s/restrict.dat.inst", rst_path, ststr);
                    if (str_f_result < 1) {
                        perror("[Frequency Server] site path format failed\n");
                        return 1;
                    }
                } 

                // Default: Get lab testing restrict file
                else {
                    printf("\n[Frequency Server] WARNING: Parameter \'ststr\' is missing or set to a \"lab\" setting!\n");
                    strcpy(restrict_file, "/home/radar/repos/SuperDARN_MSI_ROS/linux/home/radar/ros.3.6/tables/superdarn/site/site.sys/restrict.dat.inst\0");
                }

                printf("\nFrequency Server] Using restrict file path: %s\n\n", restrict_file);
                read_restrict(restrict_file, restricted_freq, &restricted_num, temp_ptrs, temp_ptrs_num);
            }

            sem_post(sl_init.sem);

            // printf("[Frequency Server] Initialization data read; processing...\n");

            // TODO: storeInRadarTable(restrict_freq, meta_data)

            // printf("[Frequency Server] Initialization data processed...\n");
        }
        
        printf("[Frequency Server] Processing Clear Frequency...\n");
        
        // Special: Semaphore order is faulty
        if (meta_data.num_antennas == 0) {
            printf("[Frequency Server] ERROR: Called for Clear Freq without prior Initialization\n");
            printf("[Frequency Server] ERROR: There is likely a semaphore leak, please close and restart all related processes.\n");
            cleanup();
            return 0;
        }
        
        // If samples flagged, process clear frequency
        else if (sem_trywait(sf_samples.sem) == 0 && meta_data.num_antennas != 0){
            // Wait to read-in data
            printf("[Frequency Server] Awaiting sample data unlock...\n");
            sem_wait(sl_samples.sem);

            // Process Sample relevant data
            printf("[Frequency Server] Processing client sample data...\n");
            read_sample_shm(temp_samples, samples_obj.shm_ptr, meta_data.num_antennas, samples_num);
            printf("[Frequency Server] Samples done...\n");
            
            if (*(int*) (clr_range_obj.shm_ptr) != 0) {
                printf("[Frequency Server] Clear Range reading...\n");
                read_int(clr_range, clr_range_obj.shm_ptr, 2);
                // printf("    clr_range: %d -- %d\n", clr_range[0], clr_range[1]);
            }

            if (*(int*) (fcenter_obj.shm_ptr) != 0) {
                printf("[Frequency Server] Freq Center reading...\n");
                read_single_int( &(meta_data.usrp_fcenter), fcenter_obj.shm_ptr);
                // printf("    fcenter: %d\n", meta_data.usrp_fcenter);
            }

            if (*(int*) (beam_num_obj.shm_ptr) != 0) {
                printf("[Frequency Server] Beam Number reading...\n");
                read_single_int(&beam_num, beam_num_obj.shm_ptr);
                // printf("    beam_num: %d\n", beam_num);
            }

            sem_post(sl_samples.sem);

            // Store Sample Data
            // if (samples_storage_i < STORAGE_NUM) {
            //     samples_storage[samples_storage_i] = temp_samples;
            //     samples_storage_i++;
            // }
            // else {
            //     // Process Samples Storage per time Packets ...
            //     for (int i = 0; i < STORAGE_NUM; i++) {
            //         // Beamform and FFT in all directions

            //         // Store in temp bin
            //     }

            //     // Spectral Avg (all packets into 1 and X # of samples by Avg Aatio) and Find Clear Freqs


            //     // 
                

            //     samples_storage_i = 0;
            // }

            // Process Clear Freq
            printf("[Frequency Server] Starting Clear Freq Search...\n");
            clear_freq_search(
                temp_samples, 
                clr_range,
                beam_num,
                sample_sep,
                restricted_freq, 
                restricted_num,
                meta_data,
                clr_bands                
            );
            // TODO: update_clr_table(clr_bands);
            
            
            // Write Clear Freq Data
            printf("[Frequency Server] Writing clear frequency data to Shared Memory...\n");
            sem_wait(sl_clrfreq.sem);
            write_clrfreq_shm(clr_bands, clrfreq_obj.shm_ptr);
            if (msync(clrfreq_obj.shm_ptr, CLR_BANDS_SHM_SIZE, MS_SYNC) == -1) {    // Synchronize data writes with program counter
                perror("msync failed");
            }
            printf("[Frequency Server] clrfreq_shm written...\n");
            sem_post(sl_clrfreq.sem);
            sem_post(sf_clrfreq.sem);
            printf("[Frequency Server] Processed Clear Freq Request successfully...\n");


            // Debug: Store prior clear freq band sets
            memcpy(clr_bands_storage[clr_storage_i], clr_bands, CLR_BANDS_MAX * sizeof(freq_band));
            clr_storage_i++;
            printf("[Frequency Server] Clr Freq Log Batch: %d/%d\n", clr_storage_i, CLR_STORAGE_NUM);
            if (clr_storage_i >= CLR_STORAGE_NUM) {
                write_clr_log_csv(clr_bands_storage, clr_storage_i);
                clr_storage_i = 0;
            }
        }
        
        printf("[Frequency Server] Processed Client successfully...\n");

        t2 = clock();
        printf("[Frequency Server] Processing Time for Client (s): %lf\n", ((double) (t2 - t1)) / (CLOCKS_PER_SEC));
    }
}
