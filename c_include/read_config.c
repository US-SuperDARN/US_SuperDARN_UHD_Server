#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ini_parser.h"
#include "clear_freq_search.h"
// #include "clear_frequency_server.h"
#include "log.h"


static int config_ini_handler(void* user, const char* section, const char* name, const char* value) {
    Config* pconfig = (Config*)user;

    if (strcmp(section, "array_info") == 0) {
        if (strcmp(name, "stid") == 0) {
            strncpy(pconfig->array_info.radar_stid, value, 3);
            pconfig->array_info.radar_stid[3] = '\0';
        } else if (strcmp(name, "x_spacing") == 0) {
            pconfig->array_info.x_spacing = atof(value);
            // printf("value: %f\nvalue (str): %s", atof(value), value);
        } else if (strcmp(name, "nbeams") == 0) {
            pconfig->array_info.nbeams = atoi(value);
            // printf("value: %d\nvalue (str): %s", atoi(value), value);
        } else if (strcmp(name, "beam_sep") == 0) {
            pconfig->array_info.beam_sep = atof(value);
            // printf("value: %f\nvalue (str): %s", atof(value), value);
        } else if (strcmp(name, "nradars") == 0) {
            pconfig->array_info.nradars = atoi(value);
            // printf("value: %d\nvalue (str): %s", atoi(value), value);
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
    } else if (strcmp(section, "gain_control") == 0) {
        if (strcmp(name, "mute_antenna_idx") == 0) {
            char *value_copy = strdup(value);
            int idx = 0;
            char *token = strtok(value_copy, ",");
            while (token != NULL && idx < STATIC_ANTENNA_NUM) {
                int i_token = atoi(token);
                if (i_token == 0 && idx != 0) break;

                pconfig->gain_control.mute_antenna_ids[idx] = i_token;
                value_copy = strdup(value); 
                token = strtok(NULL, ",");
                idx++;
            }
            pconfig->gain_control.num_mute_antennas = idx;
            free(value_copy);
        }
    }

    return 1;
}

/**
 * @brief  Loads in the array configuration from array_config.ini. 
 * @note   By DF
 * @param  *config_path:       Filepath of the array_config.ini file
 * @retval None
 */
Config read_array_config(const char *config_path){
    Config config = {0};

    if (ini_parse(config_path, config_ini_handler, &config) < 0) {
        log_error("Can't load config_path: \n%s", config_path);
        config.array_info.beam_sep = 0;
    }

    return config;
    // *x_spacing = config.array_info.x_spacing;
    // *n_beams = config.array_info.nbeams;
    // *beam_sep = config.array_info.beam_sep;
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
