#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>


// Logging Vars
#define LOG_TERMINAL_LEVEL  3                           // 0 = TRACE, 1 = DEBUG, 2 = INFO, 3 = WARN, 4 = ERROR, 5 = FATAL
#define LOG_FILE_LEVEL      0
#define LOG_FILEPATH        "/data/log/cfs/cfs.%s.log"

// Server Config Vars
#define AVG_RATIO               4                       // Number of samples to average during spectral averaging (4 = 4 samples avg-ed per beam)
#define FFTW_THREADS            2                       // Number of threads to use for FFTW
#define USE_ACTIVE_MUTE         1                       // 0 to only use muted antennas in Array Config, 1 to use Array Config + let CFS detect low-power antennas to mute
#define USE_MULTI_RANGE         1                       // 1 to use CFS' multiple clear ranges optimization, 0 to optimize for single clear range
#define SAVE_CLR_LOG            0                       // 1 to store selected frequencies in CSV log files, 0 to skip

// Filepaths Vars
#define ARRAY_CONFIG_FILEPATH   "../array_config.ini"
#define DRIVER_CONFIG_FILEPATH  "../driver_config.ini"
#define DEFAULT_SITE_STSTR      "lab"                   // Default site config to use if not passed from usrp_server

// Default Length of Variables (some dynamically change during runtime)
#define SAMPLES_NUM             2500
#define STATIC_ANTENNA_NUM      20
#define STATIC_RADAR_NUM        2                       // Max Number of possible radars in an array
#define STATIC_CHANNEL_NUM      2                       // Max Number of possible channels in an array
#define STATIC_RANGE_NUM        8                       // Max number of possible clear ranges per radar
#define RESERV_NUM              (STATIC_RADAR_NUM * STATIC_CHANNEL_NUM) // Number of reserved freq bands in the radar_table
#define BEAM_NUM                16                      // Number of beams to process
#define SAMPLE_TIME             3                       // Time per Sample (in seconds)
#define STORAGE_TIME            60                      // Total time per Sample Storage Batch (in seconds)
#define STORAGE_NUM             (STORAGE_TIME / SAMPLE_TIME) // Total number of processed sample sets to store
#define META_ELEM               1                       // 1 = 2 - 1 (fcenter has unique obj)
#ifndef RESTRICT_NUM
#define RESTRICT_NUM            50                      // Number of restricted freq bands in the restrict.dat.inst
#endif
#define CLR_STORAGE_NUM         1000
#define SITE_ID_ELEM            3                       // 3 = 3-letter identifier

#define SAMPLES_SHM_SIZE        (STATIC_ANTENNA_NUM * SAMPLES_NUM * 2 * sizeof(int))
#define CLR_RANGE_SHM_SIZE      (2 * sizeof(int))
#define FCENTER_SHM_SIZE        (1 * sizeof(int))
#define BEAM_NUM_SHM_SIZE       (1 * sizeof(int))
#define SAMPLE_SEP_SHM_SIZE     (1 * sizeof(int))
#define IFBB_FREQ_SHM_SIZE      (1 * sizeof(double))
#define META_DATA_SHM_SIZE      ((META_ELEM + STATIC_ANTENNA_NUM) * sizeof(double))
#define ANTENNA_SHM_SIZE        (1 * sizeof(int))
#define CLR_BAND_SHM_SIZE       (1 * sizeof(int) * 3)
#define RADAR_ID_SHM_SIZE       (1 * sizeof(int))
#define CHANNEL_ID_SHM_SIZE     (1 * sizeof(int))
#define ACTIVE_CLIENTS_SHM_SIZE (1 * sizeof(int))
#define MUTED_ANT_SHM_SIZE      (STATIC_ANTENNA_NUM * sizeof(int))          // List of Muted Antennas

// Shared Memory and Semaphore Names
#define SAMPLES_SHM_NAME        "/samples"
#define CLR_RANGE_SHM_NAME      "/clear_freq_range"
#define FCENTER_SHM_NAME        "/fcenter"
#define BEAM_NUM_SHM_NAME       "/beam_num"
#define SAMPLE_SEP_SHM_NAME     "/sample_sep"
#define IFBB_FREQ_SHM_NAME      "/ifbb_freq"
#define META_DATA_SHM_NAME      "/meta_data"
#define ANTENNA_SHM_NAME        "/antenna_num"
#define CLRFREQ_SHM_NAME        "/clear_freq"
#define RADAR_ID_SHM_NAME       "/radar_id"
#define CHANNEL_ID_SHM_NAME     "/channel_id"
#define ACTIVE_CLIENTS_SHM_NAME "/active_clients"   // For debugging
#define MUTED_ANT_SHM_NAME      "/muted_ant"

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

#pragma once

typedef struct {
    char radar_stid[4];
    char radar_stid_2[4];
    double x_spacing;
    int nradars;
    int nbeams;
    double beam_sep;
} ArrayInfo;

typedef struct {
    double max_tpulse;
    double min_chip;
    double max_dutycycle;
    double max_integration;
    double minimum_tfreq;
    double maximum_tfreq;
    double min_tr_to_pulse;
} HardwareLimits;

typedef struct {
    int mute_antenna_ids[STATIC_ANTENNA_NUM];
    int num_mute_antennas;
} GainControl;

typedef struct {
    double min_clrfreq_delay;
    double clrfreq_res;
    int    avg_ratio;
    double auto_max_age;
    double auto_pause_time;
} ClrSettings;

typedef struct {
    int fsamprx;
} CudaSettings;

typedef struct {
    ArrayInfo array_info;
    HardwareLimits hardware_limits;
    GainControl gain_control;
    ClrSettings clr_settings;
    CudaSettings cuda_settings;
} Config;

