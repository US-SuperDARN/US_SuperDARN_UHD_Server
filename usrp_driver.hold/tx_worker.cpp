#include <complex>
#include <vector>

#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/exception.hpp>
#include <boost/thread.hpp>
#include <thread>
#include <math.h>
#include <time.h>

#include "tx_worker.h"

#define TEST_TXWORKER 0
#define SAVE_TX_SAMPLES_DEBUG 0
#define DEBUG 1

#ifdef DEBUG
#define DEBUG_PRINT(...) do{ fprintf( stdout, __VA_ARGS__ ); }  while( false )
#else
#define DEBUG_PRINT(...) do{ } while ( false )
#endif


extern int verbose;

/***********************************************************************
 * tx_worker function
 **********************************************************************/
void usrp_tx_worker(
    uhd::tx_streamer::sptr tx_stream,
    std::vector<std::vector<std::complex<int16_t>>> *pulse_samples_pointer,
    size_t padded_num_samples_per_pulse,
//    size_t pulses_per_sequence,
    uhd::time_spec_t burst_start_time,
    std::vector<size_t> pulse_sample_idx_offsets 
){
    // pulse samples are zero padded with 
    // [ spb * 0 ] [ pulse samples ] [ spb * 0 ]
    std::vector<std::vector<std::complex<int16_t>>> pulse_samples = (*pulse_samples_pointer);
    struct timeval t0, t1;
    gettimeofday(&t0,NULL);

    fprintf(stderr,"TX_WORKER starting up\n");
    DEBUG_PRINT("TX_WORKER starting up\n");

    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst   = false;
    md.has_time_spec  = true;
    md.time_spec = burst_start_time;
    float timeout = 5;

    size_t number_of_pulses = pulse_sample_idx_offsets.size();
    size_t spb = tx_stream->get_max_num_samps();
    int32_t samples_per_pulse = padded_num_samples_per_pulse - 2*spb; 
    fprintf(stderr,"TX_WORKER nSamples_per_pulse=%i + 2*%zu (zero padding)\n", samples_per_pulse, spb);
    DEBUG_PRINT("TX_WORKER nSamples_per_pulse=%i + 2*%zu (zero padding)\n", samples_per_pulse, spb);
    int iSide;
    int nSides = pulse_samples.size();
    std::vector<std::complex<int16_t>*> buffer(nSides); 
    DEBUG_PRINT("TX_WORKER nSides=%i\n", nSides);

    // assume at least spb length zero padding before first pulse
    size_t tx_burst_length_samples = pulse_sample_idx_offsets[number_of_pulses-1] + samples_per_pulse -1;
    size_t nacc_samples = 0;
    size_t ntx_samples;
    size_t sample_idx = 0;
    uint32_t pulse_idx = 0;
    int32_t nsamples_to_send, samples_to_pulse;
 

 //   for (iSide =0; iSide<nSides; iSide++) {
 //       buffer[iSide] = &pulse_samples[iSide][sample_idx]; 
 //   }
 //   nacc_samples += tx_stream->send(buffer, spb, md, timeout);

 //   md.start_of_burst = false;
 //   md.has_time_spec = false;


    while(nacc_samples < tx_burst_length_samples) {
        // calculate the number of samples to send in the packet
        nsamples_to_send = std::min(tx_burst_length_samples - nacc_samples, spb);
        
        // calculate the number of samples until the next transmit pulse
        samples_to_pulse = pulse_sample_idx_offsets[pulse_idx] - nacc_samples;
        
        // if the transmit pulse will arrive within the current sample packet, calculate correct sample index into sample vector
        if(nsamples_to_send >= samples_to_pulse) {
            //  DEBUG_PRINT("pulse_idx=%i, nacc_samples=%i, nsamples_to_send=%d, samples_to_pulse=%d \n", pulse_idx, nacc_samples, nsamples_to_send, samples_to_pulse);
            if(samples_to_pulse * -1 < samples_per_pulse) {
                sample_idx = spb - samples_to_pulse + (pulse_idx) * padded_num_samples_per_pulse;
            } else {
                // if we've passed the tail of the pulse, then restart and look for the next one..
                // DEBUG_PRINT("pulse idx: %d complete\n", pulse_idx);
         //       if (number_of_pulses -1 > pulse_idx){
                    pulse_idx++;
                    continue;
         //       } else {
         //         sample_idx = 0;

         //       }
            }
        }

        else {
            // default to transmitting zeros (first spb samples in pulse_samples)
            sample_idx = 0;
        }

        //DEBUG_PRINT("sending buffer with sample_idx: %d\n", sample_idx);
        for (iSide =0; iSide<nSides; iSide++) {
            buffer[iSide] = &pulse_samples[iSide][sample_idx]; 
        }


      ntx_samples = tx_stream->send(buffer, nsamples_to_send, md, timeout);

        md.start_of_burst = false;
        md.has_time_spec = false;
    //    if(DEBUG) std::cout << boost::format(" nacc: %1%, to pulse: %2% ") % nacc_samples % samples_to_pulse;

    //    if(DEBUG && sample_idx) std::cout << boost::format(" Sent packet:  idx: %i") % sample_idx << std::endl;
        nacc_samples += ntx_samples;
    }
    DEBUG_PRINT("TX_WORKER tx_burst_length_samples=%i\n", tx_burst_length_samples );

    md.end_of_burst = true;
    tx_stream->send(&pulse_samples[0], 0, md, timeout);

    DEBUG_PRINT("Waiting for async burst ACK... ");
    uhd::async_metadata_t async_md;
    bool got_async_burst_ack = false;

    while (not got_async_burst_ack and tx_stream->recv_async_msg(async_md, timeout)){
        got_async_burst_ack = (async_md.event_code == uhd::async_metadata_t::EVENT_CODE_BURST_ACK);
    }
    
    DEBUG_PRINT((got_async_burst_ack ? "success\n" : "fail\n"));

    DEBUG_PRINT("TX_WORKER finished pulses\n");

    gettimeofday(&t1,NULL);
    double elapsed=(t1.tv_sec-t0.tv_sec)*1E6;
    elapsed+=(t1.tv_usec-t0.tv_usec);
    std::cout << "finished transmitting pulse sequence, elapsed tx worker time (us): " << elapsed << "\n";
    DEBUG_PRINT("TX_WORKER finished\n");
 
}
