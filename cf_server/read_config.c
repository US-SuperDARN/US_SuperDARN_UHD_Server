#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ini_parser.h"
#include "clear_freq_search.h"


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
        if (strcmp(name, "avg_ratio") == 0) {
            pconfig->clr_settings.avg_ratio = atoi(value);
        }
    } else if (strcmp(section, "dsp_filters") == 0) {
        if (strcmp(name, "rate_list") == 0) {
            char *value_copy = strdup(value);
            int idx = 0;
            char *token = strtok(value_copy, ",");
            while (token != NULL && idx < 4) {
                pconfig->dsp_filters.rate_list[idx] = atoi(token);
                token = strtok(NULL, ",");
                idx++;
            }
            free(value_copy);
        } else if (strcmp(name, "rsep_list") == 0) {
            char *value_copy = strdup(value);
            int idx = 0;
            char *token = strtok(value_copy, ",");
            while (token != NULL && idx < 4) {
                pconfig->dsp_filters.rsep_list[idx] = atoi(token);
                token = strtok(NULL, ",");
                idx++;
            }
            free(value_copy);
        } else if (strcmp(name, "rfif_atten") == 0) {
            char *value_copy = strdup(value);
            int idx = 0;
            char *token = strtok(value_copy, ",");
            while (token != NULL && idx < 4) {
                pconfig->dsp_filters.rfif_atten[idx] = atof(token);
                fprintf(stderr, "rfif_atten[%d] = %f\n", idx, atof(token));
                token = strtok(NULL, ",");
                idx++;
            }
            free(value_copy);
        } else if (strcmp(name, "rfif_width") == 0) {
            char *value_copy = strdup(value);
            int idx = 0;
            char *token = strtok(value_copy, ",");
            while (token != NULL && idx < 4) {
                pconfig->dsp_filters.rfif_width[idx] = atof(token);
                fprintf(stderr, "rfif_width[%d] = %f\n", idx, atof(token));
                token = strtok(NULL, ",");
                idx++;
            }
            free(value_copy);
        } else if (strcmp(name, "ifbb_atten") == 0) {
            char *value_copy = strdup(value);
            int idx = 0;
            char *token = strtok(value_copy, ",");
            while (token != NULL && idx < 4) {
                pconfig->dsp_filters.ifbb_atten[idx] = atof(token);
                fprintf(stderr, "ifbb_atten[%d] = %f\n", idx, atof(token));
                token = strtok(NULL, ",");
                idx++;
            }
            free(value_copy);
        } else if (strcmp(name, "ifbb_rFreq") == 0) {
            char *value_copy = strdup(value);
            int idx = 0;
            char *token = strtok(value_copy, ",");
            while (token != NULL && idx < 4) {
                pconfig->dsp_filters.ifbb_rfreq[idx] = atof(token);
                fprintf(stderr, "ifbb_rfreq[%d] = %f\n", idx, atof(token));
                token = strtok(NULL, ",");
                idx++;
            }
            free(value_copy);
        }
    }

    return 1;
}

