
#ifndef FILTER_TRANSFER_FUNCTION_C_H
#define FILTER_TRANSFER_FUNCTION_C_H

#include <stddef.h>

struct freq_band;

#ifdef __cplusplus
extern "C" {
#endif

int filter_transfer_function_c(double rf_samplingRate, double if_samplingRate,
                                  double rfif_downsample, double ifbb_downsample,
                                  double bw, double rfif_atten, double rfif_width,
                                  double ifbb_atten, double ifbb_rfreq,
                                  size_t spec_len, double* out_response);

#ifdef __cplusplus
}
#endif

//
// Notes for the C caller:
//   - out_response must point to a caller-allocated buffer of spec_len doubles.
//   - Return value: 0 on success, negative on error (bad arguments, allocation
//     failure, or FFTW plan failure). No C++ exception ever crosses into C.
//   - rfif_atten/rfif_width/ifbb_atten/ifbb_rfreq are the mode-specific filter
//     spec values; the C caller selects the right mode index before calling.

/* SpectrumData must match the definition used by the C++ side exactly. */
struct SpectrumData {
    unsigned long long t;
    int   beam_num;
    int   num_samples;
    int   freq_start;
    int   dfreq;
    float *f_spectrum;
};
typedef struct SpectrumData SpectrumData;

#ifdef __cplusplus
extern "C" {
#endif

/* Returns the frequency (Hz) of lowest filtered power, or -1 on error.
 * xfr_function   : array of length xfr_function_len (filter response)
 * spectrum_data  : pointer to a populated SpectrumData
 * min_pwr        : out-parameter, set to the minimum power found
 */
int find_power_min_c(int i_start, int i_end,
                    const double *xfr_function, int xfr_function_len,
                    struct freq_band *restricted_bands, int restricted_num,
                    double* f_spectrum, int spec_len,
                    int freq_start, int dfreq,
                    double *min_pwr);

#ifdef __cplusplus
}
#endif


#ifdef __cplusplus
extern "C" {
#endif

/* Returns the frequency (Hz) of lowest filtered power, or -1 on error.
 * xfr_function   : array of length xfr_function_len (filter response)
 * spectrum_data  : pointer to a populated SpectrumData
 * min_pwr        : out-parameter, set to the minimum power found
 */
double find_power_freq_c(int i_freq, int spec_len,
                    const double *xfr_function, double* f_spectrum);

#ifdef __cplusplus
}
#endif

#endif /* FIND_POWER_MIN_C_H */
