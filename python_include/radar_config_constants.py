# TODO: all of these should be moved to a config file at some point..

ST_ID = 20

RESTRICT_FILE = '/home/radar/repos/SuperDARN_MSI_ROS/linux/home/radar/ros.3.6/tables/superdarn/site/site.mcm/restrict.dat.inst'

USRP_MASTER_CLOCK_FREQ = 200e6 # not all values are supported by the ettus UHD driver, changing this will require changing downconversion ratios

# seconds of delay in usrp clock time at the start of an integration period before the first pulse
# to allow for the usrp_drivers receive a command..
INTEGRATION_PERIOD_SYNC_TIME_ONESEC = .2 
INTEGRATION_PERIOD_SYNC_TIME = .2 

# DELAY before clear freq record
MIN_CLRFREQ_DELAY = .50

# Frequency Resolution (in Hz) 
# Do NOT use scientific notation
CLRFREQ_RES = 1000 # Radar and USRP frequency resolution
CFSFREQ_RES = 250 # must be a devisior of CLRFREQ_RES (CFS uses the ratio of CLRFREQ_RES / CFSFREQ_RES = # of Averages)

MAX_AGE_OF_AUTO_CLEAR_FREQ = 30 # in sec, if data is older a new clear freq record if triggered
PAUSE_TIME_BEFORE_AUTO_CLEAR_FREQ = 0.0 # is seconds


# padding in seconds between pulse sequences
# starting from the last transmit pulse within a sequence
# so, this needs to be long enough to avoid any ambiguity between pulse sequences
# for assuming a maximum range of 5000 kilometers, this could be about .033 seconds
# if this is not an integer multiple of the baseband sampling period, the values will be round down
# the rf baseband sampling period is probably 3.333333... kHz
PULSE_SEQUENCE_PADDING_TIME = .033

