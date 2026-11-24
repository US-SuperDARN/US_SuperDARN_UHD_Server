#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <numbers> // C++20 for std::numbers::pi
#include <complex>
#include <memory>
#include "clear_frequency_server.h"
#include "clear_freq_search.h"
#include <algorithm>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <fftw3.h>
#include <chrono>
#include "filter_based_cfs.h"


// Computes the Zeroth-order Modified Bessel function of the first kind I0(x)
// Using power series expansion (converges quickly for typical beta values)
double modifiedBesselI0(double x) {
    double sum = 1.0;
    double u = 1.0;
    double halfX = x / 2.0;
    
    for (int i = 1; i <= 25; ++i) {
        u *= (halfX / i);
        sum += u * u;
        if (u * u < sum * 1e-15) break; // Break early if precision limit is reached
    }
    return sum;
}

// Calculates the order of the Kaiser window filter based on attenuation and transition width
int KaiserOrder(double atten, double width, int &num_taps, double &beta) {
    // Implementation for calculating Kaiser filter order       
    if (atten < 21) {
        beta = 0.0;
    } else if (atten <= 50) {
        beta = 0.5842 * std::pow(atten - 21, 0.4) + 0.07886 * (atten - 21);
    } else {
        beta = 0.1102 * (atten - 8.7);
    }
    num_taps = static_cast<int>(std::ceil((atten - 7.95) / (2.285 * std::numbers::pi * width))) + 1;
    return 0;
}

// Generates FIR Low-Pass Filter coefficients using a Kaiser Window
std::vector<double> designKaiserLPF(int num_taps, double width, double beta) {
    std::vector<double> coefficients(num_taps);
    
    // Normalized cutoff frequency in radians/sample (0 to pi)
    double wc = std::numbers::pi * width;
    
    // M represents the order of the filter (num_taps - 1)
    double M = num_taps - 1; 
    double i0_beta = modifiedBesselI0(beta);

    for (int n = 0; n < num_taps; ++n) {
        // 1. Calculate Ideal Sinc Response (centered around M/2)
        double i_centered = n - M / 2.0;
        double ideal_sinc;
        
        if (std::abs(i_centered) < 1e-9) {
            ideal_sinc = wc / std::numbers::pi; // L'Hopital's rule limit at the center tap
        } else {
            ideal_sinc = std::sin(wc * i_centered) / (std::numbers::pi * i_centered);
        }

        // 2. Calculate Kaiser Window Weight
        double val = (2.0 * n - (num_taps - 1)) / (num_taps - 1);
        double window_weight = modifiedBesselI0(beta * std::sqrt(1.0 - val * val)) / i0_beta;

        // 3. Apply Windowing to the ideal impulse response
        coefficients[n] = ideal_sinc * window_weight;
    }

    return coefficients;
}

// convolve the filter coefficients with a window of lentht equal to the downsampling factor to give nulls at the alias frequencies
std::vector<double> smooth_window(size_t smooth_length, std::vector<double> &input_signal) {
    size_t in_len = input_signal.size();
    size_t n_out = in_len + smooth_length - 1;
    std::vector<double> smoothed_signal(n_out, 0.0);
    
    double running_sum = 0.0;

    for (size_t i = 0; i < n_out; ++i) {
        if ( i < in_len) {
            running_sum += input_signal[i];
        }
        if (i >= smooth_length) {
            running_sum -= input_signal[i - smooth_length];
        }
        smoothed_signal[i] = running_sum;
    }

    return smoothed_signal;
}


struct FFTWDeleter {
    void operator()(fftw_complex* ptr) const {
        fftw_free(ptr);
    }
};

// Assumes these project definitions already exist:
//   struct FFTWDeleter { void operator()(fftw_complex*) const; };
//   struct Config;
//   void KaiserOrder(double attenuation, double width, int& taps, double& beta);
//   std::vector<double> designKaiserLPF(int taps, double corner, double beta);
//   std::vector<double> smooth_window(std::size_t window, const std::vector<double>& h);
using FFTWComplexPtr = std::unique_ptr<fftw_complex[], FFTWDeleter>;

static std::size_t checked_downsample(double value, const char* name)
{
    if (!std::isfinite(value) || value < 1.0 || std::floor(value) != value ||
        value > static_cast<double>(std::numeric_limits<std::size_t>::max())) {
        throw std::invalid_argument(std::string(name) + " must be a positive integer");
    }
    return static_cast<std::size_t>(value);
}

static int round_taps_up(int taps, std::size_t multiple)
{
    if (taps < 1 || multiple < 1) throw std::invalid_argument("invalid tap count or multiple");
    const std::size_t rounded =
        ((static_cast<std::size_t>(taps) + multiple - 1) / multiple) * multiple;
    if (rounded > static_cast<std::size_t>(std::numeric_limits<int>::max()))
        throw std::overflow_error("rounded tap count exceeds int range");
    return static_cast<int>(rounded);
}

static void load_real_taps(fftw_complex* dst, std::size_t fft_len,
                           const std::vector<double>& taps)
{
    if (taps.size() > fft_len) throw std::invalid_argument("filter has more taps than FFT length");
    for (std::size_t i = 0; i < fft_len; ++i) {
        dst[i][0] = (i < taps.size()) ? taps[i] : 0.0;
        dst[i][1] = 0.0; // FIR coefficients are real; do not duplicate them into the imaginary part.
    }
}

// Returns |H_RFIF(f) * H_IFBB(f)| for each full-spectrum bin.
// Existing project helpers/types (Config, KaiserOrder, designKaiserLPF,
// smooth_window, FFTWDeleter) must be visible at this point.
std::vector<double> filter_transfer_function(
    double rf_samplingRate, double if_samplingRate,
    double rfif_downsample, double ifbb_downsample, double bw,
    double rfif_atten, double rfif_width, double ifbb_atten, double ifbb_rfreq,
    std::size_t spec_len)
{
    if (spec_len == 0 || spec_len > static_cast<std::size_t>(std::numeric_limits<int>::max()))
        throw std::invalid_argument("spec_len must fit a positive FFTW int length");

    const std::size_t M1 = checked_downsample(rfif_downsample, "rfif_downsample");
    const std::size_t M2 = checked_downsample(ifbb_downsample, "ifbb_downsample");
    if (spec_len % M1 != 0) throw std::invalid_argument("spec_len must be divisible by rfif_downsample");

    int taps = 0;
    double beta = 0.0;
    KaiserOrder(rfif_atten, rfif_width, taps, beta);
    taps = round_taps_up(taps, M1);
    const double if_corner = bw / rf_samplingRate;
    std::vector<double> h1 = designKaiserLPF(taps, if_corner, beta);
    h1 = smooth_window(M1, h1);

    FFTWComplexPtr in1(fftw_alloc_complex(spec_len));
    FFTWComplexPtr out1(fftw_alloc_complex(spec_len));
    if (!in1 || !out1) throw std::bad_alloc();
    load_real_taps(in1.get(), spec_len, h1);

    fftw_plan plan1 = fftw_plan_dft_1d(static_cast<int>(spec_len), in1.get(), out1.get(),
                                       FFTW_FORWARD, FFTW_ESTIMATE);
    if (!plan1) throw std::runtime_error("failed to create RFIF FFTW plan");
    fftw_execute(plan1);
    fftw_destroy_plan(plan1);
    in1.reset(); // No longer needed; retain only the first-stage response.

    double ifbb_width = 2.0 * bw * ifbb_rfreq / if_samplingRate;
    KaiserOrder(ifbb_atten, ifbb_width, taps, beta);
    taps = round_taps_up(taps, M2);
    const double bb_corner = 1.1 * bw / if_samplingRate;
    const std::vector<double> h2 = designKaiserLPF(taps, bb_corner, beta);

    const std::size_t spec_len_bb = spec_len / M1;
    if (spec_len_bb > static_cast<std::size_t>(std::numeric_limits<int>::max()))
        throw std::invalid_argument("second-stage FFT length exceeds FFTW int range");
    FFTWComplexPtr in2(fftw_alloc_complex(spec_len_bb));
    FFTWComplexPtr out2(fftw_alloc_complex(spec_len_bb));
    if (!in2 || !out2) throw std::bad_alloc();
    load_real_taps(in2.get(), spec_len_bb, h2);

    fftw_plan plan2 = fftw_plan_dft_1d(static_cast<int>(spec_len_bb), in2.get(), out2.get(),
                                       FFTW_FORWARD, FFTW_ESTIMATE);
    if (!plan2) throw std::runtime_error("failed to create IFBB FFTW plan");
    fftw_execute(plan2);
    fftw_destroy_plan(plan2);

    // Precompute the repeated stage-2 magnitude response, then combine it with
    // the stage-1 response. This avoids constructing complex temporaries per bin.
    std::vector<double> power2(spec_len_bb);
    for (std::size_t j = 0; j < spec_len_bb; ++j) {
        const double re = out2[j][0];
        const double im = out2[j][1];
        power2[j] = re * re + im * im;  // |H2|^2
        }
        
    std::vector<double> response(spec_len);
    for (std::size_t i = 0; i < spec_len; ++i) {
        const double re = out1[i][0];
        const double im = out1[i][1];
        const double power1 = re * re + im * im;  // |H1|^2
        response[i] = power1 * power2[i % spec_len_bb];
    }
    return response;
}

int find_power_freq(int i_freq, int spec_len,
                    const std::vector<double>& XFR_Function,
                    float* f_spectrum)
{
    double sum = 0;
    const int tail = spec_len - i_freq;
    // Non-wrapped portion
    for (int k = 0; k < tail; ++k){
        sum += XFR_Function[k] * f_spectrum[i_freq + k];
    }
    // Wrapped portion
    for (int k = tail; k < spec_len; ++k){
        sum += XFR_Function[k] * f_spectrum[k - tail];
    }
    return(sum/spec_len);
}

int find_power_min(int i_start, int i_end,
                    const std::vector<double>& XFR_Function,
                    freq_band *restricted_bands, int restricted_num,
                    float* f_spectrum, int spec_len,
                    int freq_start, int dfreq,
                    float* min_pwr)
{
    if (min_pwr == nullptr)
        return -1;

    *min_pwr = std::numeric_limits<float>::max();

    if (spec_len <= 0 ||
        XFR_Function.size() < static_cast<std::size_t>(spec_len))
        return -1;

    constexpr int edge_buffer = 5;
    const int first = std::max(0, i_start + edge_buffer);
    const int last  = std::min(spec_len, i_end - edge_buffer);
    if (first >= last)
        return -1;

    double best_power = std::numeric_limits<double>::infinity();
    int best_frequency = -1;
    int cur_freq;


    for (int i = first; i < last; ++i) {
        double sum = 0.0;
        const int tail = spec_len - i;
        // check for restricted frequency
        cur_freq = freq_start + i * dfreq;
        int is_restricted = 0;
        for (int ib = 0; ib < restricted_num + RESERV_NUM; ib++) {
            if( (cur_freq >= restricted_bands[ib].f_start) && (cur_freq <= restricted_bands[ib].f_end) ){
                is_restricted = 1;
                break;
            }
        }
        if( is_restricted )continue;

        // Non-wrapped portion
        for (int k = 0; k < tail; ++k){
            //std::cerr << i<< " " << k << " " << XFR_Function[k] << " " << f_spectrum[i+k] << "\n";
            sum += XFR_Function[k] * f_spectrum[i + k];
        }
        // Wrapped portion
        for (int k = tail; k < spec_len; ++k){
            //std::cerr << i<< " " << k << " " << XFR_Function[k] << " " << f_spectrum[k-tail] << "\n";
            sum += XFR_Function[k] * f_spectrum[k - tail];
        }
        if (sum <= best_power) {
            best_power = sum;
            best_frequency = freq_start + i *  dfreq;
        }
        int print=1;
        if(print)std::cout << freq_start + i * dfreq << " " << sum << "\n";
    }

    if (best_frequency >= 0)
        *min_pwr = static_cast<float>(best_power/spec_len);

    return best_frequency;
}

// --- C-callable wrapper --------------------------------------------------
// xfr_function    : pointer to spec_len doubles (the transfer-function weights)
// xfr_function_len: number of elements in xfr_function (checked against spec_len)
// spectrum_data   : plain struct, already C-compatible (raw float* member)
// min_pwr         : output, lowest weighted power found
// Returns the frequency with lowest power on success, or -1 on invalid input
// (same convention as the C++ function; freq_min can legitimately be 0, so
// check the return value, not just truthiness, when treating -1 as an error).
int find_power_min_c(
    int i_start, int i_end,
    const double* xfr_function, int xfr_function_len,
    freq_band *restricted_bands, int restricted_num,
    float* f_spectrum, int spec_len, 
    int freq_start, int dfreq,
    float* min_pwr)
{
    if (xfr_function == nullptr)
        return -1;

    const std::vector<double> xfr_vec(xfr_function, xfr_function + xfr_function_len);

    return find_power_min(i_start, i_end, xfr_vec, restricted_bands, restricted_num, f_spectrum, (int)spec_len, freq_start, dfreq, min_pwr);
}

int find_power_freq_c(
    int i_freq, int spec_len,
    const double* xfr_function,
    float* f_spectrum)
{
    if (xfr_function == nullptr)
        return -1;

    const std::vector<double> xfr_vec(xfr_function, xfr_function + spec_len);

    return find_power_freq(i_freq, spec_len, xfr_vec, f_spectrum);
}


int filter_transfer_function_c(
    double rf_samplingRate, double if_samplingRate,
    double rfif_downsample, double ifbb_downsample, double bw,
    double rfif_atten, double rfif_width, double ifbb_atten, double ifbb_rfreq,
    std::size_t spec_len, double* out_response)
{
    if (out_response == nullptr)
        return -1;

    try {
        std::vector<double> response = filter_transfer_function(
            rf_samplingRate, if_samplingRate, rfif_downsample, ifbb_downsample,
            bw, rfif_atten, rfif_width, ifbb_atten, ifbb_rfreq, spec_len);
        std::copy(response.begin(), response.end(), out_response);
        return 0;
    } catch (const std::exception&) {
        return -2;
    } catch (...) {
        return -3;
    }
}

