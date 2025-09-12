import logging
import os
import time

consoleOutputLevel = logging.DEBUG
logfileOutputLevel = logging.DEBUG
#consoleOutputLevel = logging.INFO
#logfileOutputLevel = logging.INFO
useColor = True


logDir = "/data/log/"
if not os.path.exists(logDir):
   os.makedirs(logDir)


def initLogging(logFileName):
    logFileName = os.path.join(logDir, logFileName)
    # set up logging to file
    logging.basicConfig(level=logfileOutputLevel,
                        format='%(asctime)s.%(msecs)03d %(name)-14s %(levelname)-8s %(message)s',
                        datefmt='%Y-%m-%d %H:%M:%S',
                        filename=logFileName,
                        filemode='w')    # new file for every start
    # handler for console output
    console = logging.StreamHandler()
    console.setLevel(consoleOutputLevel)
    if useColor:
        colorFilt = ColorFilter()
        console.addFilter(colorFilt)
        consoleFormatter = logging.Formatter('%(name)-14s: %(preLevel)s%(levelname)-8s%(postLevel)s %(message)s')
    else:
        consoleFormatter = logging.Formatter('%(name)-14s: %(levelname)-8s %(message)s')

    console.setFormatter(consoleFormatter)
    logging.getLogger('').addHandler(console) # add the handler to the root logger

    logging.Formatter.converter = time.gmtime

    # Now, we can log to the root logger, or any other logger. First the root...
    logging.info('Starting logging ...')


BLUE = '\033[94m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
WHITE = '\033[37m'
RESET = '\033[0m'

colorCoding = {
    'WARNING': YELLOW,
    'INFO': WHITE,
    'DEBUG': BLUE,
    'CRITICAL': YELLOW,
    'ERROR': RED
}


class ColorFilter(logging.Filter):

    def filter(self, record):
        levelName = record.levelname
        if levelName in colorCoding:
            record.preLevel  = colorCoding[levelName]
            record.postLevel = RESET
        else:
            record.preLevel  = ""
            record.postLevel = ""
        return True
