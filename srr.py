#!/usr/bin/python3
# Software Radio Radar
#  Function to quickly start and terminate single radar processes.
#
#  Usage:
#     srr.py [status]              : shows all software radio processes 
#     srr.py init [stid]           : create symlink to array_, usrp_, and driver_config.ini files
#
#     srr.py networkTool|net       : calls networkTool.py 
#     srr.py rawView               : plot raw rx bb samples of all antennas
#     srr.py watchdog|w            : calls tools/watchdog.py
#     srr.py help                  : show this help
#
#     srr.py start   PROCESS       : start a process or process group (see below for processes) 
#     srr.py stop    PROCESS       : stop a process or process group (see below for processes) 
#     srr.py restart PROCESS       : restart a process or process group with needed wait times
#
#  Available PROCESSES:
#     cuda (or cuda_driver)        : cuda driver
#     usrps (or usrp_driver)       : all usrps defined in usrp_config.ini
#     server (or usrp_server)      : usrp_server
#   
#     errlog (or errorlog)         : errlog server            (no restart)
#     rawacf (or rawacfwrite)      : rawacf write process     (no restart)
#     fitacf (or fitacfwrite)      : fitacf write process     (no restart)
#     rtserver                     : real time display server (no restart)
#   
#     uaf_fix (or uafscan_fix)     : starts uafscan fast with fixfreq 14000 (only start)
#     uaf_fix ch_No                : starts uaf_fix on channel ch_No (1-4)
#     uaf_fix_onesec [ch_No]       : starts uafscan (onsec, fixfreq) default ch_No=1
#
#  Available PROCESS GROUPS:
#     driver                       : cuda_driver and usrp_driver
#     all                          : cuda_driver, usrp_driver and usrp_server
#     allscans                     : all processes with scan in their name
#
#
#  TIP:
#    set alias to shorten "srr.py" to "srr" and call it from anywhere:
#       alias srr="/home/radar/SuperDARN_UHD_Server/srr.py "
#    and make it permanent by including it in ~/.bash_aliases
#

#  TODO:
# restart is processes already running?
# get all differnt call types "python3 cuda_driver.py" ".../python3 ./cuda_driver.py"
# add return argument to stop commands and don't wait on restart if nothing has been shut down
# 


import sys
import os
import subprocess
import re
import socket
import numpy as np
import configparser
import subprocess
import time
import errno
import signal
from pathlib import Path
import getpass

homePath = str(Path.home())

basePath = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, basePath + '/tools')

userName = getpass.getuser()

import networkTool

sys.path.insert(0, basePath + '/python_include')
import rosmsg

# TODO: get from config
CUDADriverPort = 55420
CUDA_EXIT = ord('e')

USRPDriverPort = 54420
UHD_EXIT = ord('e')

USRP_SERVER_PORT = 45000
USRP_SERVER_QUIT  = ',' # '.'

USE_USRP_DRIVER_WRAPPER = True


# time to wait for usrps
# UHD 3.9 10s, UHD 3.10 0
nSecs_restart_pause = 10
delay_between_driver_and_server = 10

editors = ['emacs','gedit','nano','vi','vim']

def myPrint(msg):
   print("||>  {}".format(msg))
basePrintLine = "||>==============================================================="

def waitFor(nSeconds):
   print("||> Waiting for {} second(s): ".format(nSeconds), end="", flush=True)
   for i in range(nSeconds):
      print('.', end="", flush=True)
      time.sleep(1)
   else:
      print()
######################
## init
def initialize(inputArg):
   configPath = basePath + "/config" 

   array_config_target_file  = basePath + "/array_config.ini"
   driver_config_target_file = basePath + "/driver_config.ini"
   usrp_config_target_file   = basePath + "/usrp_config.ini"

   # remove old files
   myPrint(" Removing old config files")
   for targetFile in [array_config_target_file, driver_config_target_file, usrp_config_target_file]:
      if os.path.isfile(targetFile) or os.path.islink(targetFile):
         if os.path.islink(targetFile): # is a link
             myPrint("  removing link for {}".format(targetFile))
             os.unlink(targetFile)
         else: # is a file
             myPrint("  removing file {}".format(targetFile))
             os.remove(targetFile)

   # default paramter values 
   radarName    = 'lab'
   myPrint(" Starting with default: {}".format(radarName))

   # parse input arguments
   del inputArg[0]
   if len(inputArg) > 0:
      radarName = inputArg[0].lower()

   myPrint(" Initializing with: {}".format(radarName))

   array_config_source_file   = configPath + "/" + radarName + "/" + "array_config__" + radarName + ".ini"
   driver_config_source_file  = configPath + "/" + radarName + "/" + "driver_config__" + radarName + ".ini"
   usrp_config_source_file    = configPath + "/" + radarName + "/" + "usrp_config__" + radarName + ".ini"

   # linking new files   
   myPrint("Creating symlink {} ".format(array_config_target_file))
   myPrint("   -> {}".format(array_config_source_file))
   os.symlink(array_config_source_file, array_config_target_file)

   myPrint("Creating symlink {} ".format(driver_config_target_file))
   myPrint("   -> {}".format(driver_config_source_file))
   os.symlink(driver_config_source_file, driver_config_target_file)

   myPrint("Creating symlink {}".format(usrp_config_target_file))
   myPrint("   -> {}".format(usrp_config_source_file))
   os.symlink(usrp_config_source_file, usrp_config_target_file)

def show_help():
   thisFile = open( os.path.realpath(__file__), 'r')
   helpText = []
   isFirst = True
   for line in thisFile:
      if isFirst:
         isFirst = False
         continue
      if line.startswith("#"):
         helpText.append(line[1:])
         myPrint(line[1:-1])
      else:
         break
#   print(helpText)
      

def set_alias():
   # does only work for the subprocess session...
   aliasPar = 'alias srrt="{}/srr.py " '.format(basePath)
   subprocess.call(['alias', aliasPar], shell=True)
   myPrint("setting alias srr to script")

######################
## Processes:

def get_processes():
   commandList = ["ps", "-aux"]
   s = subprocess.Popen(commandList, stdout = subprocess.PIPE, stderr = subprocess.PIPE)
   out, err = s.communicate()
   processList = out.decode("UTF-8").split("\n")
   return processList

def print_status():
    processList = get_processes()
    srrProcesses = get_known_processes(processList)
   
    myPrint("Local: Found {} processes:".format(len(srrProcesses)))
    for line in sorted(srrProcesses):
       myPrint("  {}".format(line))

    remote_pc_list = get_remote_driver_host()
    for remote_pc in remote_pc_list:
       myPrint(" ")
       respond = remote_command_echo(userName, remote_pc, "ps -aux", verbose=False)
       processList = respond.decode("UTF-8").split("\n")
       srrProcesses = get_known_processes(processList)
       myPrint("Remote {}: Found {} processes:".format(remote_pc, len(srrProcesses)))
       for line in sorted(srrProcesses):
          myPrint("  {}".format(line))

def remote_stop_all():
    remote_pc_list = get_remote_driver_host()
    for remote_pc in remote_pc_list:
        myPrint(" calling srr stop for {}".format(remote_pc))
        respond = remote_command_echo(userName, remote_pc, "srr stop", verbose=False)
        print(respond)

def get_known_processes(processList):
    knownProcessList = ['./usrp_driver', "/usr/bin/python3 ./cuda_driver.py", "python3 cuda_driver.py",  "/usr/bin/python3 ./usrp_server", "uafscan", "fitacfwrite", "iqwrite", "rawacfwrite", "errlog", "shellserver", "rtserver", "python3 "+homePath+"/SuperDARN_UHD_Server/tools/srr_watchdog.py","python3 "+homePath+"/repos/SuperDARN_UHD_Server/tools/srr_watchdog.py","/usr/bin/python3 ./srr_watchdog.py", "schedule", "start.scd", "normalscan", "onebeamscan", "uafscan", "pcodescan", "interleavescan", "normalsound", "themisscan", "noopscan", "./cf_server"]
    srrProcesses = []
    for line in processList:
        wordList = [word for word in line.split(" " ) if word != ""]
        if len(wordList):
           commandString = " ".join(wordList[10:]) 
           for knownProcess in knownProcessList:
               if commandString.startswith(knownProcess):
                  srrProcesses.append( " " + commandString + "  (PID " + wordList[1] + ")")
                  break
    return srrProcesses


def pid_exists(pid):
    """Check whether pid exists in the current process table.  UNIX only. 
    """
    if pid < 0:
        return False
    if pid == 0:
        raise ValueError('invalid PID 0')
    try:
        os.kill(pid, 0)
    except OSError as err:
        if err.errno == errno.ESRCH:
            # ESRCH == No such process
            return False
        elif err.errno == errno.EPERM:
            # EPERM clearly means there's a process to deny access to
            return True
        else:
            # According to "man 2 kill" possible error values are (EINVAL, EPERM, ESRCH)
            raise
    else:
        return True

def terminate_pid(pid):
  if pid_exists(pid):
     myPrint("   killing pid {}".format(pid))
     os.kill(pid, signal.SIGTERM)

def terminate_all(pidDictList):
    if len(pidDictList):
       for process in pidDictList:
           terminate_pid(process["pid"])


#######################
## STOP:
def get_usrp_driver_processes():
    processList = get_processes()
    usrpProcesses = []
    for line in processList:
        if any(x in line for x in editors):
            continue
        if 'usrp_driver' in line:
            wordList = [word for word in line.split(" " ) if word != ""]
            try:
               antenna = int(wordList[wordList.index("--antennaA")+1])
               host = wordList[wordList.index("--host")+1]
               usrpProcesses.append(dict(pid=int(wordList[1]), host=host, antenna=antenna))
            except:
               myPrint("Failed to extract host and antenna_idx: {}".format(line))
               usrpProcesses.append(dict(pid=int(wordList[1]), host=None, antenna=None))

    return usrpProcesses

def stop_usrp_driver_soft():
    """ soft stop: connectin as server and sending quit command
        output: 0 (no driver processes have been active), 1 (soft stop worked) or -1 (driver still active) 
    """
    usrpProcesses = get_usrp_driver_processes()
    if len(usrpProcesses) == 0:
        myPrint("  No usrp_driver processes")
        return 0
    myPrint("Found {} usrp_driver processes".format(len(usrpProcesses)))
    
    dtype = np.uint8
    
    for process in usrpProcesses:
        if process['host'] == None:
            continue

        myPrint("  sending UHD_EXIT to {}:{} (pid {})".format(process['host'], int(process['host'].split(".")[2]) + USRPDriverPort, process['pid']))
        try:
           usrpsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
           usrpsock.connect(('localhost', int(process['host'].split(".")[2]) + USRPDriverPort))
    
           usrpsock.sendall(dtype(UHD_EXIT).tobytes())
        except:
           myPrint("  connection to {}:{} (pid {}) failed".format(process['host'], int(process['host'].split(".")[2]) + USRPDriverPort, process['pid']))
    
       # dstr = usrpsock.recv(dtype().nbytes )
       # myPrint('  => {}  received ?? as {} ({} / {}  bytes): {}'.format(__file__, dtype, len(dstr), dtype().nbytes , dstr  ))
       # data = np.fromstring(dstr, dtype=dtype)
       # myPrint("   data:{}".format(data))
    
    
    myPrint("  Done.")
    waitFor(2)
    myPrint("   Again checking for usrp_driver processes after soft stop...")
    usrpProcesses = get_usrp_driver_processes()
    if len(usrpProcesses) == 0:
        myPrint("   No usrp_driver processes")
        return 1
    else:
        myPrint("   Found {} usrp_driver processes".format(len(usrpProcesses)))
        return -1

def stop_usrp_driver_hard():
  #  myPrint("Stop usrp_driver hard...")
    terminate_all(get_usrp_driver_processes())


def stop_usrp_driver():
    myPrint(" Stopping usrp_driver...")
    try:
        driver_status = stop_usrp_driver_soft()
    except:
        myPrint("Soft USRP stop failed...")
    if driver_status == -1:
        stop_usrp_driver_hard()

    return abs(driver_status)


def get_cuda_driver_processes():
    processList = get_processes()
    cudaProcesses = []
    for line in processList:
        wordList = [word for word in line.split(" " ) if word != ""]
        if len(wordList):
           commandString = " ".join(wordList[10:])
           if commandString in ["/usr/bin/python3 ./cuda_driver.py", "python3 cuda_driver.py"]:
              cudaProcesses.append(dict(pid=int(wordList[1]) ))
    return cudaProcesses

def get_process_ids(processShortName):
    if processShortName == "cuda":
       processMatchString = ["/usr/bin/python3 ./cuda_driver.py", "python3 cuda_driver.py"]
       nWords = 2 
    elif processShortName == "srr_watchdog":
       processMatchString = ["/usr/bin/python3 ./srr_watchdog.py", "python3 /home/radar/repos/SuperDARN_UHD_Server/tools/srr_watchdog.py","python3 /home/radar/SuperDARN_UHD_Server/tools/srr_watchdog.py"]
       nWords = 1
    elif processShortName == "usrp_server":
       processMatchString = ["/usr/bin/python3 ./usrp_server.py"]
       nWords = 2
    elif processShortName == "rtserver":
       processMatchString = ["rtserver"]
       nWords = 0
    elif processShortName == "scan":
       processMatchString = ["uafscan", "normalscan"]
       nWords = 0
    else:
       nWords = 0
       processMatchString = [processShortName]
       ValueError("unknown process short name {}".format(processShortName))
       
     
    processList = get_processes()
    foundProcesses = []
    for iLine, line in enumerate(processList):
        wordList = [word for word in line.split(" " ) if word != ""]
        if len(wordList):
           commandString = " ".join(wordList[10:11+nWords])
           if commandString in processMatchString:
              foundProcesses.append(dict(pid=int(wordList[1]) ))
    return foundProcesses

def stop_cuda_driver():
    myPrint(" Stopping cuda_driver...")
    cudaProcesses = get_process_ids("cuda")
    if len(cudaProcesses):
       for process in cudaProcesses:
            try:
               myPrint("  sending CUDA_EXIT to localhost:{} (pid {})".format(CUDADriverPort, process['pid']))
               sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
               sock.connect(('localhost',CUDADriverPort))
               sock.sendall(np.uint8(CUDA_EXIT).tobytes())
            except:
                myPrint(" soft exit failed: localhost:{} (pid {})".format(CUDADriverPort, process['pid']))
                
        
       time.sleep(1)
       # terminate processes if they still exis
       terminate_all(cudaProcesses)
    else:
       myPrint("  No cuda_driver processes found...")
    
def stop_usrp_server():
    myPrint(" Stopping usrp_server...")
    serverProcesses = get_process_ids("usrp_server")
    if len(serverProcesses):
      try:
          for process in serverProcesses:
               myPrint("  sending SERVER_EXIT to localhost:{} (pid {})".format(USRP_SERVER_PORT, process['pid']))
               sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
               sock.connect(('localhost', USRP_SERVER_PORT))
               time.sleep(0.3)
           #    sock.sendall(np.int32(ord(USRP_SERVER_QUIT)).tobytes())
               rmsg = rosmsg.rosmsg_command(sock)
               rmsg.set_data('status', 0)
               rmsg.set_data('type', ord(USRP_SERVER_QUIT))
               rmsg.transmit()
 
          time.sleep(1)
      except:
          myPrint("Error while connecting to server. Killing PID...")
          # terminate processes if they still exis
      terminate_all(serverProcesses)
      return 1
    else:
       myPrint("  No usrp_server processes found...")
       return 0
    
def stop_rtserver():
    myPrint(" Stopping rtserver...")
    serverProcesses = get_process_ids("rtserver")
    if len(serverProcesses):
       terminate_all(serverProcesses)
       return 1
    else:
       myPrint("  No rtserver processes found...")
       return 0
    
def stop_allscans():
    myPrint(" Stopping scans...")
    serverProcesses = get_process_ids("scan")
    if len(serverProcesses):
       terminate_all(serverProcesses)
       return 1
    else:
       myPrint("  No errlog processes found...")
       return 0
def stop_errorlog():
    myPrint(" Stopping errlog...")
    serverProcesses = get_process_ids("errlog")
    if len(serverProcesses):
       terminate_all(serverProcesses)
       return 1
    else:
       myPrint("  No errlog processes found...")
       return 0
    
def stop_fitacf_write():
    myPrint(" Stopping fitacfwrite...")
    serverProcesses = get_process_ids("fitacfwrite")
    if len(serverProcesses):
       terminate_all(serverProcesses)
       return 1
    else:
       myPrint("  No fitacfwrite processes found...")
       return 0
    
def stop_rawacf_write():
    myPrint(" Stopping rawacfwrite...")
    serverProcesses = get_process_ids("rawacfwrite")
    if len(serverProcesses):
       terminate_all(serverProcesses)
       return 1
    else:
       myPrint("  No rawacfwrite processes found...")
       return 0
    
def stop_watchdog():
    myPrint(" Stopping watchdog...")
    serverProcesses = get_process_ids("srr_watchdog")

    if len(serverProcesses):
        for pro in serverProcesses:
            myPrint("   killing pid {}".format(pro['pid']))
            os.kill(pro['pid'], signal.SIGKILL)
        return 1
    else:
        myPrint("  No watchdog processes found...")
        return 0
    
    
    

########################
## START:
def read_config(fileName):
   # READ usrp_config.ini
   config = configparser.ConfigParser()
   config.read(fileName)
   return config

def get_remote_driver_host():
    fileName = os.path.join(basePath, 'usrp_config.ini')
    if not os.path.isfile(fileName):
       myPrint("  ERROR: usrp_config.ini not found! Run srr init or symlink correct init file by hand.")
       return -1
    usrp_config = read_config(fileName)
    usrpNameList = usrp_config.sections()
    all_driver_hosts = []
    for usrpName in usrpNameList:
       curr_host = usrp_config[usrpName]['driver_hostname']
       if curr_host not in all_driver_hosts and curr_host != "localhost":
          all_driver_hosts.append(curr_host)
    return all_driver_hosts

def start_usrps_from_config(usrp_sleep = False):
    myPrint("Starting usrp_driver from config:")
    fileName = os.path.join(basePath, 'usrp_config.ini')
    if not os.path.isfile(fileName):
       myPrint("  ERROR: usrp_config.ini not found! Run srr init or symlink correct init file by hand.")
       return -1
    usrp_config = read_config(fileName)
    
    usrpNameList = usrp_config.sections()
    usrpPIDlist  = []
    
    myPrint("  Found {} usrps in config {}:".format(len(usrpNameList), fileName))
    start_arg_list = []
    for usrpName in usrpNameList:
        if usrp_config[usrpName]['driver_hostname'] == 'localhost':
            myPrint("     {} : antenna {},   ip: {}".format( usrpName , usrp_config[usrpName]['array_idx'],  usrp_config[usrpName]['usrp_hostname'] ))
            if usrp_config[usrpName]['side'].lower()[0] == 'a':
               ant_arg = "--antennaA"
            elif usrp_config[usrpName]['side'].lower() == "b":
               ant_arg = "--antennaB"
            else:
                print("Error: unknown side: {}".format(usrp_config[usrpName]['side'].lower()))
                return -1

            newDevice = True
            for arg in start_arg_list:
                if arg[0] == usrp_config[usrpName]['usrp_hostname']:
                    arg += [ant_arg, usrp_config[usrpName]['array_idx'] ]
                    newDevice = False
                    break
            if newDevice:
               allArgs =[ usrp_config[usrpName]['usrp_hostname'], ant_arg, usrp_config[usrpName]['array_idx'] ] 
               if usrp_config[usrpName]['mainarray'] == "False":
                  allArgs += ["--interferometer"]

               start_arg_list.append(allArgs)
        else:
            myPrint("not local:  {} : antenna {},   ip: {}".format( usrpName , usrp_config[usrpName]['array_idx'],  usrp_config[usrpName]['usrp_hostname'] ))

      
    os.chdir(os.path.join(basePath, "usrp_driver") )   

###    baseStartArg = ['./usrp_driver', '--intclk', '--host' ] # intclock only for debug

    if USE_USRP_DRIVER_WRAPPER:
        baseStartArg = ['./usrp_driver_logging_wrapper',  '--host' ]
    else:
        baseStartArg = ['./usrp_driver',  '--host' ]

    
    for start_arg in start_arg_list:
       all_start_arg = baseStartArg + start_arg 
       myPrint("Starting {}".format(" ".join(all_start_arg) ))

       usrpPIDlist.append( subprocess.Popen(all_start_arg))

       # test: wait to avoid frame sizes < 8000 bytes
       time.sleep(0.1)
#       usrpPIDlist.append( subprocess.Popen(['./usrp_driver', '--intclk', '--antenna', usrp_config[usrpName]['array_idx']  , '--host', usrp_config[usrpName]['usrp_hostname'] ]))
#       usrpPIDlist.append( subprocess.Popen(['./usrp_driver',  '--antenna', usrp_config[usrpName]['array_idx']  , '--host', usrp_config[usrpName]['usrp_hostname'] ]))
       if usrp_sleep:
          time.sleep(8)
    os.chdir(basePath)
    
def start_usrp_driver():
    start_usrps_from_config()

def start_cuda_driver():
    myPrint("Starting cuda_driver...")
    os.chdir(os.path.join(basePath, "cuda_driver") )   
    subprocess.Popen(['./cuda_driver.py' ])
    os.chdir(basePath)

def start_clear_frequency_server():
    myPrint("Starting clear_frequency_server...")
    subprocess.Popen(['./cf_server' ])

def start_usrp_server():
    myPrint("Starting usrp_server...")
    os.chdir(os.path.join(basePath, "usrp_server") )
    print(os.getcwd())
    subprocess.Popen(['./usrp_server.py' ])
    os.chdir(basePath)

def start_uafscan_fixfreq_onesec(inputArg):
    commandList ='uafscan --stid mcm -c 1 --nowait --fixfrq 14000 --onesec --debug'.split(" ")
    if len(inputArg) == 2:
       myPrint("Starting uafscan fixfreq onesec on default channel 1")
    else:
       myPrint("Starting uafscan fixfreq onesec on default channel {}".format(inputArg[2]))
       commandList[4] = inputArg[2]
    myPrint("  >>>{}".format(" ".join(commandList)))
    subprocess.Popen(commandList)

def start_uafscan_fixfreq(inputArg):
    commandList ='uafscan --stid mcm -c 1 --nowait --fixfrq 14000 --fast --debug'.split(" ")
    if len(inputArg) == 2:
       myPrint("Starting uafscan fixfreq on default channel 1")
    else:
       myPrint("Starting uafscan fixfreq on default channel {}".format(inputArg[2]))
       commandList[4] = inputArg[2]
    myPrint("  >>>{}".format(" ".join(commandList)))
    subprocess.Popen(commandList)


def start_normalscan():
    command = 'normalscan -stid mcm -xcf 1 -fast -df 10400 -nf 10400 -c 4'
    myPrint("Starting normalscan...({})".format(command))
    subprocess.Popen(command.split(" "))
def start_2normalscans():
    command = 'normalscan -stid mcm -xcf 1 -fast -df 10400 -nf 10400 -c 3'
    myPrint("Starting first (of two) normalscan...({})".format(command))
    subprocess.Popen(command.split(" "))
    command = 'normalscan -stid mcm -xcf 1 -fast -df 10400 -nf 10400 -c 4'
    myPrint("Starting second (of two) normalscan...({})".format(command))
    subprocess.Popen(command.split(" "))

def start_rtserver():
    commandList = 'rtserver -rp 41104 -ep 41000 -tp 1401'.split(" ")
    myPrint("Starting {}  ({})".format(commandList[0], " ".join(commandList)))
    subprocess.Popen(commandList)

    commandList = 'rtserver -rp 43104 -ep 41000 -tp 1402'.split(" ")
    myPrint("Starting {}  ({})".format(commandList[0], " ".join(commandList)))
    subprocess.Popen(commandList)

def start_fitacf_write():
    commandList = 'fitacfwrite -r mcm.a -lp 41103 -ep 41000'.split(" ")
    myPrint("Starting {}  ({})".format(commandList[0], " ".join(commandList)))
    subprocess.Popen(commandList)

def start_rawacf_write():
    commandList = 'rawacfwrite -r mcm.a -lp 41102 -ep 41000'.split(" ")
    myPrint("Starting {}  ({})".format(commandList[0], " ".join(commandList)))
    subprocess.Popen(commandList)

def start_errorlog():
    commandList = 'errlog -name mcm.a -lp 41000'.split(" ")
    myPrint("Starting {} on port {} ({})".format(commandList[0], commandList[-1], " ".join(commandList)))
    subprocess.Popen(commandList)

    commandList = 'errlog -name mcm.b -lp 43000'.split(" ")
    myPrint("Starting {} on port {} ({})".format(commandList[0], commandList[-1], " ".join(commandList)))
    subprocess.Popen(commandList)


def start_network_tool():
    myPrint("Calling printStatus() form tools/networkTool.py...")
    networkTool.printStatus()
    return
   # myPrint("Starting networkTool.py...")
   # os.chdir(os.path.join(basePath, "tools") )   
   # subprocess.Popen(['./networkTool.py' ])

def start_liveRawView_tool():
    myPrint("Starting tools/plotRawSamples_usrpServer.py...")
    os.chdir(os.path.join(basePath, "tools") )   
    subprocess.Popen(['./plotRawSamples_usrpServer.py' ])

def start_watchdog():
    myPrint("Starting tools/watchdog.py...")
    os.chdir(os.path.join(basePath, "tools") )   
    subprocess.Popen(['./watchdog.py' ])

###############
def restart_all():
   myPrint("Restarting all processes")
   restart_driver()
   myPrint("done starting usrps.. waiting....")
   restart_clear_frequency_service()
   waitFor(delay_between_driver_and_server)
   myPrint("done  waiting.... starting server")
   start_usrp_server()

def restart_clear_frequency_service():
   server_was_running = stop_clear_frequency_service()
   if server_was_running:
      waitFor(nSecs_restart_pause)
   stop_clear_frequency_service()
   start_clear_frequency_server()

def stop_clear_frequency_service():
    myPrint(" Stopping clear frequency service...")
    serverProcesses = get_process_ids("cf_server") + get_process_ids("./cf_server")
    if len(serverProcesses):
       terminate_all(serverProcesses)
       return 1
    else:
       myPrint("  No clear_frequency_server process found...")
       return 0

def restart_driver():
    server_was_running = stop_usrp_server() 
    if server_was_running:
        waitFor(5)
    usrp_driver_was_running = stop_usrp_driver()
    stop_cuda_driver()
    if usrp_driver_was_running or server_was_running:
        waitFor(nSecs_restart_pause)
    start_cuda_driver()
    start_usrp_driver()


# runs a command over ssh, returns response
# this version is a bit more robust.. uses echo to pipe in commands
def remote_command_echo(user, radar, command, verbose = True, port = 22):
    commandecho = subprocess.Popen(['echo', command], stdout=subprocess.PIPE)

    try:
        signal.alarm(5)
        failed = True
        out = ''
        cmdlist = ["ssh", '-T', user + '@' + radar, '-p', str(port)]
        cmdlist = ["ssh", user + '@' + radar, '-p', str(port)]
        s = subprocess.Popen(cmdlist, stdin = commandecho.stdout, stdout = subprocess.PIPE, stderr = subprocess.PIPE)
        commandecho.stdout.close()
        if verbose:
            print(' '.join(cmdlist) + ' ' + command)
        out, err = s.communicate()
    except:
        print('command ' + command + ' radar ' + radar + ' failed')

    signal.alarm(0)
    return out


#########################
## MAIN PART:
def main():
   print(basePrintLine)
   myPrint("Software Radio Radar")
   myPrint(" ")

   inputArg = sys.argv[1:]
   nArguments = len(inputArg)
   
   if nArguments == 0:   # DEFAULT option
      print_status()
   else:
      firstArg = inputArg[0].lower()
      
      if firstArg == "status":
          print_status()
      elif firstArg == "init":
          initialize(inputArg)
      elif firstArg.lower() in ["liverawview", "rawview"]:
           start_liveRawView_tool()  
      elif firstArg.lower() in ["watchdog", "w"]:
           start_watchdog()  
      elif firstArg.lower() in ["network", "networktool", "net"]:
           start_network_tool()  
      elif firstArg == "start":
         if nArguments == 1 or inputArg[1].lower() == "all":
            myPrint("Starting all...")
            start_cuda_driver()
            start_usrp_driver()
            start_clear_frequency_server()
            waitFor(delay_between_driver_and_server)
            start_usrp_server()
         elif inputArg[1].lower() in ["usrp_driver", "usrps"]:
            start_usrp_driver()
         elif inputArg[1].lower() in ["cuda_driver", "cuda"]:
            start_cuda_driver()
         elif inputArg[1].lower() == "driver":
            start_cuda_driver()
            start_usrp_driver()
         elif inputArg[1].lower() in ["usrp_server", "server"]:
            start_usrp_server()
         elif inputArg[1].lower() in ["uaf_fix_onesec", "uafscan_fix_onesec"]:
            start_uafscan_fixfreq_onesec(inputArg)
         elif inputArg[1].lower() in ["uaf_fix", "uafscan_fix"]:
            start_uafscan_fixfreq(inputArg)
         elif inputArg[1].lower() in ["normalscan"]:
            start_normalscan()
         elif inputArg[1].lower() in ["2normalscans"]:
            start_2normalscans()
         elif inputArg[1].lower() in ["rtserver"]:
            start_rtserver()
         elif inputArg[1].lower() in ["errorlog", "errlog"]:
            start_errorlog()
         elif inputArg[1].lower() in ["fitacf", "fitacfwrite"]:
            start_fitacf_write()
         elif inputArg[1].lower() in ["rawacf", "rawacfwrite"]:
            start_rawacf_write()
         else:
            myPrint("ERROR: Unknown process to start")
            myPrint("See srr help for process names:")
            myPrint("")
            show_help()
   
      elif firstArg == "restart":
         if nArguments == 1 or inputArg[1].lower() == "all":
            restart_all()

         elif inputArg[1].lower() in ["usrp_driver", "usrps"]:
            usrp_driver_was_running = stop_usrp_driver()
            if usrp_driver_was_running:
                waitFor(nSecs_restart_pause) 
            start_usrp_driver()
   
         elif inputArg[1].lower() in ["cuda_driver", "cuda"]:
            stop_cuda_driver()
            start_cuda_driver()
   
         elif inputArg[1].lower() == "driver":
             restart_driver()
         elif inputArg[1].lower() in ["usrp_server", "server"]:
            stop_usrp_server()
            start_usrp_server()
   
         else:
            myPrint("ERROR: Unknown process to restart")
            myPrint("See srr help for process names:")
            myPrint("")
            show_help()
   
   
      elif firstArg == "stop":
         if nArguments == 1 or inputArg[1].lower() == "all":
            myPrint("Stopping all...")
            remote_stop_all()
            stop_watchdog()
            server_was_running = stop_usrp_server() 
            stop_clear_frequency_service()
            if server_was_running:
                waitFor(5)
            stop_usrp_driver()
            stop_cuda_driver()
         elif inputArg[1].lower() in ["usrp_driver", "usrps"]:
            stop_watchdog()
            stop_usrp_driver()
         elif inputArg[1].lower() in ["cuda_driver", "cuda"]:
            stop_watchdog()
            stop_cuda_driver()
         elif inputArg[1].lower() in ["usrp_server", "server"]:
            stop_watchdog()
            stop_usrp_server()
         elif inputArg[1].lower() in ["rtserver"]:
            stop_rtserver()
         elif inputArg[1].lower() == "driver":
            stop_watchdog()
            stop_usrp_driver()
            stop_cuda_driver()
         elif inputArg[1].lower() in ["watchdog", "srr_watchdog"]:
             stop_watchdog()
         elif inputArg[1].lower() in ["errorlog", "errlog"]:
            stop_errorlog()
         elif inputArg[1].lower() in ["fitacf", "fitacfwrite"]:
            stop_fitacf_write()
         elif inputArg[1].lower() in ["rawacf", "rawacfwrite"]:
            stop_rawacf_write()
         elif inputArg[1].lower() in ["scans", "allscans"]:
            stop_allscans()
         else:
            myPrint("ERROR: Unknown process to stop")
            myPrint("See srr help for process names:")
            myPrint("")
            show_help()
      elif firstArg == "help":
         show_help()
      
      else:
         myPrint("ERROR: UNKNWON COMMAND")
         myPrint("See srr help for usage:")
         myPrint("")
         show_help()
      
   myPrint(" ")
   print(basePrintLine)
   print(basePrintLine)
   
   
   
if __name__ == '__main__':
   main()
   
   
   
   
   
   

