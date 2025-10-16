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
#include <uhd/utils/thread.hpp>
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
    double rxrate,
    int32_t *return_status
) {

    float priority=1;
    bool realtime=true;
    uhd::set_thread_priority_safe(priority,realtime);

    float debugt = usrp->get_time_now().get_real_secs();
    DEBUG_PRINT("%s: entering RX_WORKER %2.4f\n", get_log_time(), debugt);

    int nSides = (*rx_data_buffer).size();
    const size_t max_samples_per_packet = rx_stream->get_max_num_samps();

    double time_to_start;
    uhd::time_spec_t rx_usrp_pre_stream_time = usrp->get_time_now();
    time_to_start = offset_time_spec(start_time, RX_OFFSET).get_real_secs() - rx_usrp_pre_stream_time.get_real_secs();
    if (time_to_start < RX_STREAM_EXEC_TIME) {
        fprintf(stderr, "Error in rx_worker: not enough time before start of stream, skipping this integration period (%f ms)\n", time_to_start*1000);
        *return_status = RX_WORKER_STREAM_TIME_ERROR;
        return;
    }

    //setup streaming
    uhd::rx_metadata_t md;
    md.error_code = uhd::rx_metadata_t::ERROR_CODE_NONE;

    // calculate the number of samples to stream that is a multiple of the maximum samples per packet
    size_t samples_to_stream = num_requested_samps - (num_requested_samps % max_samples_per_packet) + max_samples_per_packet;
    double timeout = 5.0;

    // DEBUG to check timing
    rx_usrp_pre_stream_time = usrp->get_time_now();
    time_to_start = start_time.get_real_secs() - rx_usrp_pre_stream_time.get_real_secs();
    fprintf(stderr,"#timing: time left for rx_worker  %f ms\n", time_to_start*1000);
    DEBUG_PRINT("%s: rx_worker: samples_to stream: %ld\n", get_log_time(), samples_to_stream);

    uhd::stream_cmd_t stream_cmd = uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE;
    stream_cmd.stream_now = false;
    stream_cmd.time_spec = offset_time_spec(start_time, RX_OFFSET);
    stream_cmd.num_samps = num_requested_samps;
    usrp->issue_stream_cmd(stream_cmd);

    size_t num_acc_samps = 0;
    std::vector<std::complex<int16_t>*> buff_ptrs(nSides);

    debugt = usrp->get_time_now().get_real_secs();
    DEBUG_PRINT("%s: starting rx_worker while loop %2.4f\n", get_log_time(), debugt);
    while (num_acc_samps < num_requested_samps) {

        size_t samp_request = std::min(max_samples_per_packet, num_requested_samps - num_acc_samps);
        for (int iSide = 0; iSide < nSides; iSide++) {
            buff_ptrs[iSide] = &((*rx_data_buffer)[iSide][num_acc_samps]);
            if (num_acc_samps == 0) {
                DEBUG_PRINT("%s: rx_worker addr: %p iSide: %d\n", get_log_time(), buff_ptrs[iSide], iSide);
            }
        }

        size_t num_rx_samps = rx_stream->recv(buff_ptrs, samp_request, md, timeout);

        useconds_t sleeptime = (useconds_t)(0.9*1e6*num_rx_samps/rxrate);
        if (sleeptime > 0) usleep(sleeptime);

        timeout = 0.1;

        // handle the error codes
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) break;
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_LATE_COMMAND) break;
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
            // same error code used for overflow and out of sequence
            if (md.out_of_sequence) {
                std::cerr << "ERROR_CODE_OVERFLOW: out of sequence." << std::endl;
            } else {
                std::cerr << "ERROR_CODE_OVERFLOW: overflow (not out of sequence)" << std::endl;
            }
            break;
        }

        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
            throw std::runtime_error(str(boost::format("Receiver error %s") % md.strerror()));
        }

        num_acc_samps += num_rx_samps;
    }

    debugt = usrp->get_time_now().get_real_secs();
    DEBUG_PRINT("%s:     RX_WORKER fetched samples! %2.4f\n", get_log_time(), debugt);

    if (num_acc_samps != num_requested_samps) {
        *return_status=-100;
        uhd::time_spec_t rx_error_time = usrp->get_time_now();
        std::cerr << "Error in receiving samples..(" << rx_error_time.get_real_secs() << ")\t";

        std::cerr << "Error code: " << md.error_code << "\t";
        std::cerr << "Samples rx'ed: " << num_acc_samps <<
            " (expected " << num_requested_samps << ")" << std::endl;
    }

    if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_LATE_COMMAND) {
        uhd::time_spec_t rx_error_time = usrp->get_time_now();
        std::cerr << "rx_worker: LATE_COMMAND encountered at " <<
            rx_error_time.get_real_secs() << std::endl;
        *return_status=md.error_code * -1;
    }

    if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
        uhd::time_spec_t rx_error_time = usrp->get_time_now();
        std::cerr << "rx_worker: Timeout encountered at " <<
            rx_error_time.get_real_secs() << std::endl;
        *return_status=md.error_code * -1;
    }

    if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
        uhd::time_spec_t rx_error_time = usrp->get_time_now();
        std::cerr << "rx_worker: Overflow encountered at " <<
            rx_error_time.get_real_secs() << std::endl;
        *return_status=md.error_code * -1;
    }

    if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
        uhd::time_spec_t rx_error_time = usrp->get_time_now();
        std::cerr << "start time: " << start_time.get_real_secs() << std::endl;
        std::cerr << "rx_worker: Unexpected error code " << md.error_code <<
            " encountered at " << rx_error_time.get_real_secs() << std::endl;
        *return_status=md.error_code * -1;
    }

    if (md.out_of_sequence) {
        uhd::time_spec_t rx_error_time = usrp->get_time_now();
        std::cerr << "start time: " << start_time.get_real_secs() << std::endl;
        std::cerr << "rx_worker: Packets out of order " << " encountered at " <<
            rx_error_time.get_real_secs() << std::endl;
        *return_status=*return_status - 1000;
    }

    DEBUG_PRINT("%s: RX_WORKER finished\n", get_log_time());
    return;
}

