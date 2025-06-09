#include <math.h>
#include <stdint.h>
#include "assert.h"
#include <stdio.h>

/*
INDEXING for multiply_mix_add() (similar for multiply_and_add(), just IF => BB )

threadIdx.x = iFilterSample*2
threadIdx.y = iChannel
blockDim.x  = nFilterSamples /2
blockDim.y  = nChannels
blockIdx.x  = iSampleIF
blockIdx.y  = iAntenna
gridDim.x   = nSamplesIF
gridDim.y   = nAntennas

Input / Output format:
 - [rx_samples_FB] = nAntennas x nChannels x 2*nSamples_FB(I/Q-Interleaved)
   for all frequency bands FB: RF, IF, and BB (RF is only one channel)
 - Filter has be be time reversed (doesn't matter for symmetric filters)

TODO:
 - if filter is real we could save calc time by removing all calculations with imag(filter) and phase correction, maybe write additional function multiply_add_real
 - NTH: avoid duplicate code for indexing, multiplication and summation

*/

#define MAXCHANNELS 8
__device__ __constant__ double  phaseIncrement_NCO_rad[MAXCHANNELS];
__device__ __constant__ int16_t decimationRates[2];                 // [0]: rf2if, [1]: if2bb


// downsampling and filter
__global__ void multiply_and_add(float *samples, float *odata, float *filter)
{
    __shared__ float itemp[1024];//Array size is max number of threads in a block
    __shared__ float qtemp[1024];

    // clear variable names
    uint32_t iFilterSampleTimes2  = threadIdx.x;
    uint32_t iChannel             = threadIdx.y;
    uint32_t nFilterSamplesDivBy2 = blockDim.x;
    uint32_t nChannels            = blockDim.y;
    uint32_t iSampleBB            = blockIdx.x;
    uint32_t iAntenna             = blockIdx.y;
    uint32_t nSamplesBB           = gridDim.x;
//    uint32_t nAntennas            = gridDim.y; 

    uint32_t iThread_lin = iFilterSampleTimes2 + iChannel*nFilterSamplesDivBy2;   // linear thread index in block

    uint32_t decimationRate_if2bb = decimationRates[1];

    // offset to aline center of filter and center of pulse
    int32_t offset = nFilterSamplesDivBy2 - decimationRate_if2bb / 2;

    uint32_t idxSample_filter = 4 * (iFilterSampleTimes2  + iChannel * nFilterSamplesDivBy2);  // 2 samples/thread (unrolled) * 2 components/ sample (I /Q) = 4
    uint32_t nSamples_if = decimationRate_if2bb * (nSamplesBB -1) + nFilterSamplesDivBy2 * 2;  // nSamples_in=decimationRate*(nSamples_out-1)+nSamples_filter
    uint32_t iSample_if = iSampleBB * decimationRate_if2bb + iFilterSampleTimes2 * 2 ;         // number of (complex) sample in rf signal

    if (iSample_if >= offset) {
        uint32_t idxSample_if = (iSample_if - offset) * 2 + iChannel * nSamples_if *2 + iAntenna * nChannels * nSamples_if *2; // index in memory (account for  I/Q, iChannel, iAntenna)
        float i0 =  samples[idxSample_if  ];
        float q0 =  samples[idxSample_if+1];
        float i1 =  samples[idxSample_if+2];
        float q1 =  samples[idxSample_if+3];

        // get filter values from global memory
        float p0re = filter[idxSample_filter  ];
        float p0im = filter[idxSample_filter+1];
        float p1re = filter[idxSample_filter+2];
        float p1im = filter[idxSample_filter+3];

       
        // multiply filter
        itemp[iThread_lin] = p0re * i0 - p0im * q0 + p1re * i1 - p1im * q1;
        qtemp[iThread_lin] = p0re * q0 + p0im * i0 + p1re * q1 + p1im * i1;
    }
    else { // zero padding the first part of the filter for output sample 0
        itemp[iThread_lin] = 0;
        qtemp[iThread_lin] = 0;
    }
     __syncthreads();


     // parallel reduce samples (could unroll loop for speedup?)
     // Do this as long as the reduction is a power of 2
     int32_t s, rem;
     s = blockDim.x;// / blockDim.y;
     rem = s % 2;
     while(s > 0 && rem == 0){
        s /= 2;
        if (threadIdx.x < s) {
            itemp[iThread_lin] += itemp[iThread_lin + s];
            qtemp[iThread_lin] += qtemp[iThread_lin + s];
        }
        rem = s % 2;

        __syncthreads();
     }

    // Do this as long as the reduction is a power of 5
     rem = s % 5;
     while(s > 0 && rem == 0){
        s /= 5;
        if (threadIdx.x < s) {
            itemp[iThread_lin] = itemp[iThread_lin] + itemp[iThread_lin + s] + itemp[iThread_lin+2*s] + itemp[iThread_lin+3*s] + itemp[iThread_lin+4*s];
            qtemp[iThread_lin] = qtemp[iThread_lin] + qtemp[iThread_lin + s] + qtemp[iThread_lin+2*s] + qtemp[iThread_lin+3*s] + qtemp[iThread_lin+4*s];
        }
        rem = s % 5;

        __syncthreads();
     }

     //// Now do a serial reduction for the remaining
     if(threadIdx.x == 0){
        for(int32_t i=1; i<s; i++){
           itemp[iThread_lin] += itemp[iThread_lin + i];
           qtemp[iThread_lin] += qtemp[iThread_lin + i];
        }
     }
     __syncthreads();

     // write back results
     if (threadIdx.x == 0) {

        uint32_t output_idx = iSampleBB *2 + iChannel * nSamplesBB *2 + iAntenna * nChannels * nSamplesBB *2; 
        odata[output_idx  ] = (float)  itemp[iThread_lin];
        odata[output_idx+1] = (float)  qtemp[iThread_lin];
     }  
}       



// multiply with filter (that includes NCO), downsampling, apply filter phase correction
__global__ void multiply_mix_add(int16_t *samples, float *odata, float *filter)
{
    __shared__ float itemp[1024];
    __shared__ float qtemp[1024];
    

    // clear variable names
    uint32_t iFilterSampleTimes2  = threadIdx.x;
    uint32_t iChannel             = threadIdx.y;
    uint32_t nFilterSamplesDivBy2 = blockDim.x;
    uint32_t nChannels            = blockDim.y;
    uint32_t iSampleIF            = blockIdx.x;
    uint32_t iAntenna             = blockIdx.y;
    uint32_t nSamplesIF           = gridDim.x;
   // uint32_t nAntennas            = gridDim.y; 

    uint32_t iThread_lin = threadIdx.y*blockDim.x+threadIdx.x;
    
    uint32_t decimationRate_rf2if = decimationRates[0];  

    // offset to aline center of filter and center of pulse
    int32_t offset = nFilterSamplesDivBy2 - decimationRate_rf2if / 2;

    uint32_t idxSample_filter = 4 * (iFilterSampleTimes2  + iChannel * nFilterSamplesDivBy2);   // 2 samples/thread (unrolled) * 2 components/ sample (I /Q) = 4
    uint32_t nSamples_rf = decimationRate_rf2if * (nSamplesIF -1) + nFilterSamplesDivBy2 * 2;   // nSamples_in=decimationRate*(nSamples_out-1)+nSamples_filter
    uint32_t iSample_rf = iSampleIF * decimationRate_rf2if + iFilterSampleTimes2 * 2;           // number of (complex) sample in rf signal

    if (iSample_rf >= offset) {
        uint32_t idxSample_rf = (iSample_rf - offset) * 2 + iAntenna *  nSamples_rf *2;         // index in memory (account for  I/Q, iAntenna)

        double phi0 = fmod((double)phaseIncrement_NCO_rad[iChannel] * (double)iSample_rf, (double)(2*M_PI));
	float mc0=cos(phi0);
	float ms0=sin(phi0);

        double phi1 = fmod((double)phaseIncrement_NCO_rad[iChannel] * (double)(iSample_rf+1), (double)(2*M_PI));
	float mc1=cos(phi1);
	float ms1=sin(phi1);

        itemp[iThread_lin] =
            filter[idxSample_filter  ] * samples[idxSample_rf  ] * mc0 -
            filter[idxSample_filter+1] * samples[idxSample_rf+1] * ms0 +
            filter[idxSample_filter+2] * samples[idxSample_rf+2] * mc1 -
            filter[idxSample_filter+3] * samples[idxSample_rf+3] * ms1;
        qtemp[iThread_lin] =
            filter[idxSample_filter  ] * samples[idxSample_rf+1] * mc0 +
            filter[idxSample_filter+1] * samples[idxSample_rf  ] * ms0 +
            filter[idxSample_filter+2] * samples[idxSample_rf+3] * mc1 +
            filter[idxSample_filter+3] * samples[idxSample_rf+2] * ms1;
    } else {
        itemp[iThread_lin] = 0;
        qtemp[iThread_lin] = 0;
    }
      

     
     __syncthreads();

     /* Example: dmrate==100,
     100 -> 50 -> 25 -> 5 -> 1*/
     // parallel reduce samples (could unroll loop for speedup?)
     // Do this as long as the reduction is a power of 2

     int32_t s, rem;
     s = blockDim.x;
     rem = blockDim.x % 2;
     while(s > 0 && rem == 0){
        s /= 2;
        if (threadIdx.x < s) {
            itemp[iThread_lin] += itemp[iThread_lin + s];
            qtemp[iThread_lin] += qtemp[iThread_lin + s];
        }
        rem = s % 2;

        __syncthreads();
     }
     // Do this as long as the reduction is a power of 5
     rem = s % 5;
     while(s > 0 && rem == 0){
        s /= 5;
        if (threadIdx.x < s) {
            itemp[iThread_lin] = itemp[iThread_lin] + itemp[iThread_lin + s] + itemp[iThread_lin+2*s] + itemp[iThread_lin+3*s] + itemp[iThread_lin+4*s];
            qtemp[iThread_lin] = qtemp[iThread_lin] + qtemp[iThread_lin + s] + qtemp[iThread_lin+2*s] + qtemp[iThread_lin+3*s] + qtemp[iThread_lin+4*s];
        }
        rem = s % 5;

        __syncthreads();
     }

     // Now do a serial reduction for the remaining
     if(threadIdx.x == 0){
        for(int32_t i=1; i<s; i++){
           itemp[iThread_lin] += itemp[iThread_lin + i];
           qtemp[iThread_lin] += qtemp[iThread_lin + i];
        }
     }
     __syncthreads();
     
     if (threadIdx.x == 0) {
        // the NCO inclueded in the filter causes a phase error. this is the correction
	//        double phiOffset = fmod(phaseIncrement_NCO_rad[iChannel] * iSampleIF*decimationRate_rf2if, 2*M_PI);
        //	  double ltemp = (double) itemp[iThread_lin]; // TODO: avoid numerical errors?? why only itemp? (mgu)
        
	//        itemp[iThread_lin] = itemp[iThread_lin] * cos(phiOffset) - qtemp[iThread_lin] * sin(phiOffset);
        //	  qtemp[iThread_lin] = ltemp * sin(phiOffset) + qtemp[iThread_lin] * cos(phiOffset);
		  
        // write outout
        uint32_t output_idx = iSampleIF *2 + iChannel * nSamplesIF *2 + iAntenna * nChannels * nSamplesIF *2; 
        odata[output_idx  ] = (float) itemp[iThread_lin];
        odata[output_idx+1] = (float) qtemp[iThread_lin];
     }

}

