/* CUDA function for upsampling, upmixing, phase delay compensation and channel combination  

grid:    nSamples_BB  x nAntennas x nPulses
block:   upsampleRate x nChannels x  1

memory formats:
   inData:   (float) nAntennas x nChannels x nPulses x 2*nSamplesBB
   outData:  (int16) nAntennas x 2*nSamplesRF 
                                                    (nSamplesRF = nInputSamples*updampling*nPulses)

TODO:
 - if we need speedup:
   - calc sin&cos(phi) only once and save it
   - calc pulse only once if there is no difference 
 - if I have time
   - what is the error of using linear interpolation instead of lowpass after upsampling???
*/

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <complex.h>

#define MAXCHANNELS 8
#define MAXANTS 32

#define I_OFFSET 0
#define Q_OFFSET 1

#define CHECK_FOR_CLIPPING 1

__device__ __constant__ double txfreq_rads[MAXCHANNELS];
__device__ __constant__ float  txphasedelay_rads[MAXANTS * MAXCHANNELS];


// main function
__global__ void interpolate_and_multiply(
    float*   inData,
    int16_t* outData
){
    /* Declare shared memory array for samples.
    Vectors are written into this array, one for each
    frequency/beam channel.  Then the thread block linearly combines
    the frequency/beam channels into a single vector to be
    transmitted on the antenna */

    // irf/qrf_samples indexed as baseband 
    __shared__ float irf_samples[1024];
    __shared__ float qrf_samples[1024];


    // define clear variable names
    int32_t upSampleRate = blockDim.x;
    int32_t nChannels    = blockDim.y;
    int32_t nSamples_bb  = gridDim.x;
    int32_t nAntennas    = gridDim.y;
    int32_t nPulses      = gridDim.z;

    int32_t iUpsample    = threadIdx.x;
    int32_t iChannel     = threadIdx.y;
    int32_t iSample_bb   = blockIdx.x;
    int32_t iAntenna     = blockIdx.y;
    int32_t iPulse       = blockIdx.z;

    // calculate index in input and output array
    int32_t idxInput  = 2*iSample_bb + iPulse*2*nSamples_bb + iChannel*nPulses*2*nSamples_bb + iAntenna*nChannels*nPulses*2*nSamples_bb;
    int32_t idxOutput = 2*(iSample_bb*upSampleRate+iUpsample) + iPulse*2*nSamples_bb*upSampleRate + iAntenna*nPulses*2*nSamples_bb*upSampleRate;

    float inc_i, inc_q;

    // linear interpolation between two baseband samples
    if ( iSample_bb == (nSamples_bb-1) && iPulse == (nPulses-1) && iChannel == ( nChannels-1) && iAntenna == (nAntennas-1) ) {
        //printf("tx_cuda: Last sample => fade out! \n");
        inc_i = (0 - inData[idxInput + I_OFFSET]) / upSampleRate * iUpsample;
        inc_q = (0 - inData[idxInput + Q_OFFSET]) / upSampleRate * iUpsample;  
  } else {
        inc_i = (inData[idxInput + I_OFFSET+2] - inData[idxInput + I_OFFSET]) / upSampleRate * iUpsample;
        inc_q = (inData[idxInput + Q_OFFSET+2] - inData[idxInput + Q_OFFSET]) / upSampleRate * iUpsample;  
  }
    
    // Calculate phase for each sample by adding phase of local oscillator and cabel phase offset 
    float phase = fmod( (double)( iSample_bb*upSampleRate + iUpsample )*txfreq_rads[iChannel], 2*M_PI) + txphasedelay_rads[iAntenna+iChannel*nAntennas];     


    // Calculate the output sample vectors, one for each freq/beam channel 
    unsigned int localInx =  iChannel*upSampleRate + iUpsample;
    irf_samples[localInx] =
        (inData[idxInput + I_OFFSET] + inc_i ) * cos(phase) -
        (inData[idxInput + Q_OFFSET] + inc_q ) * sin(phase);
    qrf_samples[localInx] =
        (inData[idxInput + I_OFFSET] + inc_i ) * sin(phase) +
        (inData[idxInput + Q_OFFSET] + inc_q ) * cos(phase);

    /* Now linearly combine all freq/beam channels into a single vector */
    // TODO: Will this synchronization work for multiple channels?
    __syncthreads();

    // threads for first channels sum up all remaining channels 
    if(iChannel == 0){
        for (unsigned int iChan2sum=1; iChan2sum<nChannels; iChan2sum++){
            irf_samples[iUpsample] += irf_samples[iUpsample + iChan2sum * upSampleRate];
            qrf_samples[iUpsample] += qrf_samples[iUpsample + iChan2sum * upSampleRate];
        }
        
        if (CHECK_FOR_CLIPPING){
            if (irf_samples[iUpsample] > 1 || irf_samples[iUpsample] < -1) {
               printf("tx_cuda.cu ERROR: abs amplitude (real value) > 1! gain control is not correct\n");
             }
            if (qrf_samples[iUpsample] > 1 || qrf_samples[iUpsample] < -1) {
               printf("tx_cuda.cu ERROR: abs amplitude (imag value) > 1! gain control is not correct\n");
             }
        } 

        // write data
        outData[idxOutput+I_OFFSET] = (int16_t) ( 32768 * irf_samples[iUpsample]);
        outData[idxOutput+Q_OFFSET] = (int16_t) ( 32768 * qrf_samples[iUpsample]);
    }

}

