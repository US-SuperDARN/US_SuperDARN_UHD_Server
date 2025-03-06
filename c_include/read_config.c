#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ini_parser.h"

typedef struct {
    int radar_stid;
    double x_spacing;
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
    ArrayInfo array_info;
    HardwareLimits hardware_limits;
} Config;

static int config_ini_handler(void* user, const char* section, const char* name, const char* value) {
    Config* pconfig = (Config*)user;
    
    if (strcmp(section, "array_info") == 0) {
        if (strcmp(name, "radar_stid") == 0) {
            pconfig->array_info.radar_stid = atoi(value);
        } else if (strcmp(name, "x_spacing") == 0) {
            pconfig->array_info.x_spacing = atof(value);
            // printf("value: %f\nvalue (str): %s", atof(value), value);
        } else if (strcmp(name, "nbeams") == 0) {
            pconfig->array_info.nbeams = atoi(value);
            // printf("value: %d\nvalue (str): %s", atoi(value), value);
        } else if (strcmp(name, "beam_sep") == 0) {
            pconfig->array_info.beam_sep = atof(value);
            // printf("value: %f\nvalue (str): %s", atof(value), value);
        }
    } else if (strcmp(section, "hardware_limits") == 0) {
        if (strcmp(name, "max_tpulse") == 0) {
            pconfig->hardware_limits.max_tpulse = atof(value);
        } else if (strcmp(name, "min_chip") == 0) {
            pconfig->hardware_limits.min_chip = atof(value);
        } else if (strcmp(name, "max_dutycycle") == 0) {
            pconfig->hardware_limits.max_dutycycle = atof(value);
        } else if (strcmp(name, "max_integration") == 0) {
            pconfig->hardware_limits.max_integration = atof(value);
        } else if (strcmp(name, "minimum_tfreq") == 0) {
            pconfig->hardware_limits.minimum_tfreq = atof(value);
        } else if (strcmp(name, "maximum_tfreq") == 0) {
            pconfig->hardware_limits.maximum_tfreq = atof(value);
        } else if (strcmp(name, "min_tr_to_pulse") == 0) {
            pconfig->hardware_limits.min_tr_to_pulse = atof(value);
        }
    } 

    return 1;
}

/*
int main() {
    Config config;
    const char *config_path = "../utils/clear_freq_input/array_config.ini";

    if (ini_parse(config_path, config_ini_handler, &config) < 0) {
        printf("Can't load 'config.ini'\n");
        return 1;
    }
    

    // Assign the read values to the variables
    double restricted_frequencies[] = { };
    double clear_freq_range[] = { };
    double beam_angle = 0;
    double smsep = 1 / (2 * 250 * 100000); // ~4 ms


    printf("Configuration loaded from 'config.ini':\n");
    printf("[array_info]\n");
    printf("radar_stid = %d\n", config.array_info.radar_stid);
    printf("x_spacing = %lf\n", config.array_info.x_spacing);
    printf("nbeams = %d\n", config.array_info.nbeams);
    printf("beam_sep = %lf\n", config.array_info.beam_sep);

    printf("\n[hardware_limits]\n");
    printf("max_tpulse = %lf\n", config.hardware_limits.max_tpulse);
    printf("min_chip = %lf\n", config.hardware_limits.min_chip);
    printf("max_dutycycle = %lf\n", config.hardware_limits.max_dutycycle);
    printf("max_integration = %lf\n", config.hardware_limits.max_integration);
    printf("minimum_tfreq = %lf\n", config.hardware_limits.minimum_tfreq);
    printf("maximum_tfreq = %lf\n", config.hardware_limits.maximum_tfreq);
    printf("min_tr_to_pulse = %lf\n", config.hardware_limits.min_tr_to_pulse);

    
    

    return 0;
}
*/