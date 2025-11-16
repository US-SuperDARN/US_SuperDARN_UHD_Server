#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ini_parser.h"
#include "clear_freq_search.h"
#include "log.h"


static int config_ini_handler(void* user, const char* section, const char* name, const char* value) {
    Config* pconfig = (Config*)user;

    if (strcmp(section, "array_info") == 0) {
        if (strcmp(name, "stid") == 0) {
            strncpy(pconfig->array_info.radar_stid, value, 3);
            pconfig->array_info.radar_stid[3] = '\0';
        } else if (strcmp(name, "stid_2") == 0) {
            strncpy(pconfig->array_info.radar_stid_2, value, 3);
            pconfig->array_info.radar_stid_2[3] = '\0';
        } else if (strcmp(name, "x_spacing") == 0) {
            pconfig->array_info.x_spacing = atof(value);
        } else if (strcmp(name, "nbeams") == 0) {
            pconfig->array_info.nbeams = atoi(value);
        } else if (strcmp(name, "beam_sep") == 0) {
            pconfig->array_info.beam_sep = atof(value);
        } else if (strcmp(name, "nradars") == 0) {
            pconfig->array_info.nradars = atoi(value);
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
    } else if (strcmp(section, "clr_settings") == 0) {
        if (strcmp(name, "min_clrfreq_delay") == 0) {
            pconfig->clr_settings.min_clrfreq_delay = atof(value);
        } else if (strcmp(name, "clrfreq_res") == 0) {
            pconfig->clr_settings.clrfreq_res = atof(value);
        } else if (strcmp(name, "avg_ratio") == 0) {
            pconfig->clr_settings.avg_ratio = atoi(value);
        } else if (strcmp(name, "auto_max_age") == 0) {
            pconfig->clr_settings.auto_max_age = atof(value);
        } else if (strcmp(name, "auto_pause_time") == 0) {
            pconfig->clr_settings.auto_pause_time = atof(value);
        }
    } else if (strcmp(section, "cuda_settings") == 0) {
        if (strcmp(name, "FSampRX") == 0) {
            pconfig->cuda_settings.fsamprx = atoi(value);
        }
    }

    return 1;
}

