/***********************************************************************
 * recv_and_hold() function
 * A function to be used to receive samples from the USRP and
 * hold them in the network socket buffer. *rx_data_buffers points to
 * the memory locations of each antenna's samples.   Meant to operate in its
 * own thread context so that it does not block the execution in main()
 * Also handles the former duties of the timing card...
 **********************************************************************/
#include <complex>
#include <vector>
#include <queue>
#include <iostream>
#include <iomanip>
#include <thread>
#include <math.h>

#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/exception.hpp>
#include <boost/thread.hpp>

#include "rx_worker.h"
#include "usrp_utils.h"
#include "dio.h"

#define RX_STREAM_EXEC_TIME .005

#define DEBUG 1
#ifdef DEBUG
#define DEBUG_PRINT(...) do{ fprintf( stderr, __VA_ARGS__ ); } while( false )
#else
#define DEBUG_PRINT(...) do{ } while ( false )
#endif

#define RX_OFFSET 0

void usrp_rx_worker(
    uhd::usrp::multi_usrp::sptr usrp,
    uhd::rx_streamer::sptr rx_stream,
    std::vector<std::vector<std::complex<int16_t>>> *rx_data_buffer,
    size_t num_requested_samps,
    uhd::time_spec_t start_time,
    int32_t *return_status
){

    float debugt = usrp->get_time_now().get_real_secs();
    DEBUG_PRINT("entering RX_WORKER %2.4f\n",debugt);
 //   fprintf( stderr, "RX WORKER nSamples requested: %i\n", num_requested_samps );
 //   fprintf( stderr, "RX WORKER nSides : %i\n", nSides );

    int nSides = (*rx_data_buffer).size();
    const size_t max_samples_per_packet = rx_stream->get_max_num_samps();

    uhd::time_spec_t rx_usrp_pre_stream_time = usrp->get_time_now();
    if(offset_time_spec(start_time, RX_OFFSET) - rx_usrp_pre_stream_time.get_real_secs() < RX_STREAM_EXEC_TIME) {
        fprintf(stderr, "Error in rx_worker: not enough time before start of stream, skipping this integration period..");
        *return_status= RX_WORKER_STREAM_TIME_ERROR;
        return;
    }



    // max number of samples per stream command is limited by UHD
    // to 0x0fffffff samples at the full sampling rate of the ADC (200 MSPS)
    // we request samples after downconversion, so the maximum number of samples that we can request is 0x0fffffff / (200e6 / output_rate)
    // to make things a bit easier, just hardcode a conservative max number of samples
    // also make math easier by chunking at a multiple of the max number of samples per packet..

    // then, issue multiple stream commands to gather the full amount of samples

 
    //setup streaming
    uhd::rx_metadata_t md;
    md.error_code = uhd::rx_metadata_t::ERROR_CODE_NONE;

    size_t samples_remaining_to_stream = num_requested_samps;

    // calculate the maximum number of samples per stream command that is a multiple of the maximum samples per packet
    // assumes 200 MHz adc clock and 10 MSPS sampling rate
    size_t max_samples_per_stream = 0x0fffffff / (200e6 / 10e6);
    max_samples_per_stream = max_samples_per_stream - (max_samples_per_stream % max_samples_per_packet);
    double timeout = 5.0;

    // DEBUG to check timing
    double time_to_start;
    rx_usrp_pre_stream_time = usrp->get_time_now();
    time_to_start = start_time.get_real_secs() - rx_usrp_pre_stream_time.get_real_secs();
    fprintf(stderr,"#timing: time left for rx_worker  %f ms\n", time_to_start*1000);
    DEBUG_PRINT("rx_worker: samples_remaining_to stream: %d  max_samples_per_stream: %d\n",samples_remaining_to_stream,max_samples_per_stream);


    int counter=0;
    if(samples_remaining_to_stream > max_samples_per_stream) {
        // each stream command should request the around the maximum number of samples possible that is a multiple of the recv stream packet size
        // so, issue NUM_SAMPS_AND_MORE commands until there are fewer than max_samples_per_packet remaining 
        uhd::stream_cmd_t stream_cmd = uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_MORE;
        stream_cmd.num_samps = max_samples_per_stream;
        stream_cmd.time_spec = offset_time_spec(start_time, RX_OFFSET);
       
        // issue the first stream command to be timed at the start of the integration period
	debugt = usrp->get_time_now().get_real_secs();
	DEBUG_PRINT("RX_WORKER: before stream command %2.4f\n",debugt);
        stream_cmd.stream_now = false;
        usrp->issue_stream_cmd(stream_cmd); 
	debugt = usrp->get_time_now().get_real_secs();
	DEBUG_PRINT("RX_WORKER: issued stream command %2.4f\n",debugt,++counter);

        samples_remaining_to_stream -= max_samples_per_stream;

        // stream commands after the first should execute immediately
        // keep sending NUM_SAMPS_AND_MORE until there are fewer than max_samples_per_stream samples 
        stream_cmd.stream_now = true;
        while(samples_remaining_to_stream > max_samples_per_stream) {
            usrp->issue_stream_cmd(stream_cmd); 
            samples_remaining_to_stream -= max_samples_per_stream;
	    debugt = usrp->get_time_now().get_real_secs();
	    DEBUG_PRINT("RX_WORKER: issued stream command %2.4f\n",debugt,++counter);
        }
        
        // finally, issue a NUM_SAMPS_AND_DONE command for the last command
        stream_cmd = uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE;
        stream_cmd.stream_now = true;
        stream_cmd.num_samps = samples_remaining_to_stream;
        usrp->issue_stream_cmd(stream_cmd); 
	debugt = usrp->get_time_now().get_real_secs();
	DEBUG_PRINT("RX_WORKER: issued last stream command %2.4f\n",debugt,++counter);
    }
    
    else {
        // if we want fewer than max_samples_per_stream samples, just request them all at once
        uhd::stream_cmd_t stream_cmd = uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE;
        stream_cmd.stream_now = false;
        stream_cmd.time_spec = offset_time_spec(start_time, RX_OFFSET);
        stream_cmd.num_samps = samples_remaining_to_stream;
        usrp->issue_stream_cmd(stream_cmd); 
    }


    size_t num_acc_samps = 0;
    std::vector<std::complex<int16_t>*> buff_ptrs(nSides);
 /*   for (int iSide=0;iSide<nSides;iSide++) {
      DEBUG_PRINT("side %d \n  ", iSide);
    
      DEBUG_PRINT("  v1: \n  ");
      for (int iSample=0; iSample<10; iSample++)
          DEBUG_PRINT("%i, ", (*rx_data_buffer)[iSide][iSample]);
    }
 */
    debugt = usrp->get_time_now().get_real_secs();
    DEBUG_PRINT("starting rx_worker while loop %2.4f\n",debugt);
    while(num_acc_samps < num_requested_samps) {

        size_t samp_request = std::min(max_samples_per_packet, num_requested_samps - num_acc_samps);
        for (int iSide = 0; iSide < nSides; iSide++) {
            buff_ptrs[iSide] = &((*rx_data_buffer)[iSide][num_acc_samps]);
            if (num_acc_samps == 0)
               DEBUG_PRINT("rx_worker addr: %p iSide: %d \n    ", buff_ptrs[iSide], iSide);
        }

        size_t num_rx_samps = rx_stream->recv(buff_ptrs , samp_request, md, timeout);
       // DEBUG print
//        if (num_rx_samps == 1996) {
      //     DEBUG_PRINT("|");
//             DEBUG_PRINT("%d  ",num_acc_samps);
 //       } else {
//           DEBUG_PRINT("(rxed %d) ", num_rx_samps);
//        }
    
        timeout = 0.1;

        //handle the error codes
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) break;
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_LATE_COMMAND) break;
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
           // same error code used for overflow and out of sequence
           if (md.out_of_sequence) {
               std::cerr << "ERROR_CODE_OVERFLOW: out of sequece." << std::endl;
           } else {
               std::cerr << "ERROR_CODE_OVERFLOW: overflow (not out of sequece)" << std::endl;
           }
   /*        std::cerr << "stop stream because of OVERFLOW" << std::endl;
           stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
           stream_cmd.time_spec = uhd::time_spec_t();
           rx_stream->issue_stream_cmd(stream_cmd);
   */
           break;
        }

        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
            throw std::runtime_error(str(boost::format(
                "Receiver error %s"
            ) % md.strerror()));
        }
                
        //if (DEBUG) {
        //    std::cout << boost::format("Received packet: %u samples, %u full secs, %f frac secs") % num_rx_samps % md.time_spec.get_full_secs() % md.time_spec.get_frac_secs() << std::endl;
        //}
        num_acc_samps += num_rx_samps;
    }
    debugt = usrp->get_time_now().get_real_secs();
    DEBUG_PRINT("RX_WORKER fetched samples! %2.4f\n",debugt);
//    if(DEBUG) std::cout << boost::format("RX_WORKER : %u full secs, %f frac secs") % md.time_spec.get_full_secs() % md.time_spec.get_frac_secs() << std::endl;

    if (num_acc_samps != num_requested_samps){
        *return_status=-100;
        uhd::time_spec_t rx_error_time = usrp->get_time_now();
        std::cerr << "Error in receiving samples..(" << rx_error_time.get_real_secs() << ")\t";

        std::cerr << "Error code: " << md.error_code << "\t";
        std::cerr << "Samples rx'ed: " << num_acc_samps << 
            " (expected " << num_requested_samps << ")" << std::endl;
    }

    if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_LATE_COMMAND) {
        uhd::time_spec_t rx_error_time = usrp->get_time_now();
        std::cerr << "rx_worker: LATE_COMMAND encountered at " << rx_error_time.get_real_secs() << std::endl;
        *return_status=md.error_code * -1;
    }
    if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
        uhd::time_spec_t rx_error_time = usrp->get_time_now();
        std::cerr << "rx_worker: Timeout encountered at " << rx_error_time.get_real_secs() << std::endl;
        *return_status=md.error_code * -1;
    }
    if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW){
        uhd::time_spec_t rx_error_time = usrp->get_time_now();
        std::cerr << "rx_worker: Overflow encountered at " << rx_error_time.get_real_secs() << std::endl;

   /*     size_t num_rx_samps = rx_stream->recv(buff_ptrs , max_samples_per_packet, md, timeout);
        std::cerr << "Overflow cleanup: received  " << num_rx_samps << "  , end _of _burst: " << md.end_of_burst <<  std::endl;

        while (num_rx_samps != 0 && !md.end_of_burst ) {
            num_rx_samps = rx_stream->recv(buff_ptrs , max_samples_per_packet, md, timeout);
            std::cerr << "Overflow cleanup: received  " << num_rx_samps << "  , end _of _burst: " << md.end_of_burst <<  std::endl;
        }

        std::cerr << "Overflow cleanup: finished."   <<  std::endl;
   */
        *return_status=md.error_code * -1;
    }
    if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
        uhd::time_spec_t rx_error_time = usrp->get_time_now();
        std::cerr << "start time: " << start_time.get_real_secs() << std::endl;
        std::cerr << "rx_worker: Unexpected error code " << md.error_code << " encountered at " << rx_error_time.get_real_secs() << std::endl;
        *return_status=md.error_code * -1;
    }
    if (md.out_of_sequence){
        uhd::time_spec_t rx_error_time = usrp->get_time_now();
        std::cerr << "start time: " << start_time.get_real_secs() << std::endl;
        std::cerr << "rx_worker: Packets out of order " << " encountered at " << rx_error_time.get_real_secs() << std::endl;
        *return_status=*return_status - 1000;
    }
/* this is now done in main progam
    if (mute_received_samples) {
        std::cerr << "Muting received samples because error occurred. " << std::endl;
        for (int iSide = 0; iSide < nSides; iSide++) { // TODO maybe smarter to mute sample while transfer to SHM (and just use special return_status)
           for (int iSample=0; iSample<num_requested_samps; iSample++) {
              (*rx_data_buffer)[iSide][iSample] = 0;
           }
        }
    }
*/

    DEBUG_PRINT("RX_WORKER finished\n");
    return;
}

