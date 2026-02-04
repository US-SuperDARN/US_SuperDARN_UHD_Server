#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/thread.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/exception.hpp>

#include "usrp_utils.h"

// for status keeping
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <utime.h>

#include <iostream>
#include <string>

#include <cstdlib>

char tstr[128];


char *get_log_time() {
    struct tm *gmt;
    struct timespec err_tm;
    int stat __attribute__ ((unused));

    stat = clock_gettime(CLOCK_REALTIME, &err_tm);

    gmt = gmtime(&err_tm.tv_sec);
    sprintf(tstr, "%04d-%02d-%02d %02d:%02d:%02d.%03d",
            1900+gmt->tm_year, gmt->tm_mon+1, gmt->tm_mday,
            gmt->tm_hour, gmt->tm_min, gmt->tm_sec,
            (int)(err_tm.tv_nsec/1e6));

    return tstr;
}


void touch_file(const std::string& fileName) {
    int fd = open(fileName.c_str(), O_WRONLY|O_CREAT|O_NOCTTY|O_NONBLOCK, 0666);
    if (fd < 0) {
        std::cerr << "Could not open file:  " << fileName << "\n";
        return;
    }

    int utime_return = utimensat(AT_FDCWD, fileName.c_str(), nullptr, 0);
    if (utime_return) {
       std::cerr << "utimensat() failed \n";
       return;
    }
}


// adds a double offset to a uhd::time_spec_t
// useful for calculating pulse times for a pulse sequence
uhd::time_spec_t offset_time_spec(uhd::time_spec_t t0, double toffset) {
    uhd::time_spec_t t1;

    int64_t full_sec = t0.get_full_secs();
    double frac_sec = t0.get_frac_secs();

    frac_sec += toffset;

    full_sec += (int64_t)floor(frac_sec);
    frac_sec -= (double)floor(frac_sec);


    t1 = uhd::time_spec_t(full_sec, frac_sec);

    return t1;
}
