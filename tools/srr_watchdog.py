#!/usr/bin/python3
import os
import datetime
import sys
import time
basePath = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
logPath = '/data/log/'
sys.path.insert(0, basePath)
import srr
import subprocess
import signal


wait_after_restart_all = 40 # TODO check if if time is good
wait_after_restart_driver = 30 # TODO check if if time is good
check_period = 10 # sec

# just for debug
restart_driver = True
write_log = True

# usrp server file
server_status_file = os.path.join(logPath, "usrp_server_status.txt")
file_age_limit = 90 + 10 # sec
usrp_restart_period = 60*5 # sec to next restart

watch_usrp_server = "server" in sys.argv

# LOG FILE
if write_log:
    log_file_path = os.path.join(logPath, "watchdog")
    time_now = datetime.datetime.now(datetime.UTC)
    fileName = 'watchdog__{:04d}{:02d}{:02d}_{:02d}{:02d}{:02d}.log'.format(time_now.year, time_now.month, time_now.day, time_now.hour, time_now.minute, time_now.second)
    print(log_file_path)
    if not os.path.isdir(log_file_path):
        os.mkdir(log_file_path)
        print("creating path: {}".format(log_file_path))

    log_handle = open(os.path.join(log_file_path, fileName), "wt+")


def log(msg):
    print(msg)
    if write_log:
       time_now = datetime.datetime.now(datetime.UTC).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
       log_str = '{}   {}\n'.format(time_now, msg)
       log_handle.write(log_str)
       log_handle.flush()


log("Starting up watchdog: watch_usrp_server: {}, restart_driver: {}".format(watch_usrp_server, restart_driver))


class GracefulKiller:
  kill_now = False
  def __init__(self):
    signal.signal(signal.SIGINT, self.exit_gracefully)
    signal.signal(signal.SIGTERM, self.exit_gracefully)


  def exit_gracefully(self,signum, frame):
    self.kill_now = True


class usrpDriverWatcher():
   def __init__(self, fileName, usrp_restart_period):
      self.config = srr.read_config(fileName)
      self.restart_period = usrp_restart_period
      self.last_restart = None
      usrp_name_list = self.config.sections()
      self.usrp_list = []
      known_hosts = []
      for usrpName in usrp_name_list:
         if self.config[usrpName]['driver_hostname'] == "localhost":
              if self.config[usrpName]['usrp_hostname'] in known_hosts:
                 usrp = self.usrp_list[known_hosts.index(self.config[usrpName]['usrp_hostname'])]
                 usrp.ant.append(self.config[usrpName]['array_idx'])
                 usrp.side.append(self.config[usrpName]['side'])
              else:
                 self.usrp_list.append(usrpClass(self.config[usrpName]))
                 known_hosts.append(self.config[usrpName]['usrp_hostname'])


   def check_processes(self):
      usrp_processes = srr.get_usrp_driver_processes()
      active_host_list = [process['host'] for process in usrp_processes]
      active_pid_list = [process['pid'] for process in usrp_processes]
      for usrp in self.usrp_list:
         if usrp.host in active_host_list:
            usrp.pid = active_pid_list[active_host_list.index(usrp.host)]
         else:
            usrp.pid = None


   def restart_usrps(self):
       now = time.mktime(time.localtime())
       if self.last_restart == None or (now-self.last_restart) > self.restart_period:
          os.chdir(os.path.join(basePath, "usrp_driver"))
          for usrp in self.usrp_list:
            if usrp.pid == None:
               self.last_restart = now
               start_arg = usrp.get_start_arguments()
               if restart_driver:
                  log("Starting {}".format(" ".join(start_arg)))
                  subprocess.Popen(start_arg)
               else:
                  log("Starting of driver disabled ({})".format(" ".join(start_arg)))


class usrpClass():
   """ For each USRP one"""
   def __init__(self, configDict):
      self.host = configDict['usrp_hostname']
      self.computer = configDict['driver_hostname']
      self.ant = [configDict['array_idx']]
      self.mainarray = configDict['mainarray']
      self.side = [configDict['side']]
      self.pid = None


   def get_start_arguments(self):
       if srr.USE_USRP_DRIVER_WRAPPER:
           start_arg = ["./usrp_driver_logging_wrapper", "--host", self.host]
       else:
           start_arg = ["./usrp_driver", "--host", self.host]

       for iSide, side in enumerate(self.side):
          if side.lower() == 'a':
             start_arg += ["--antennaA", self.ant[iSide]]
          elif side.lower() == 'b':
              start_arg += ["--antennaB", self.ant[iSide]]
          else:
             log("unknown usrp side: {}".format(side))

       if self.mainarray.lower() in ["false", "0", "off"]:
            start_arg += ["--interferometer"]
       return start_arg


fileName = os.path.join(basePath, 'usrp_config.ini')
usrp_driver_watcher = usrpDriverWatcher(fileName, usrp_restart_period)

killer = GracefulKiller()
restart_server = False

new_start = True

while True:

   if watch_usrp_server:
       if os.path.isfile(server_status_file):
           # check age of server status file
           m_time = os.path.getmtime(server_status_file)
       else:
           m_time = 0
       now = time.mktime(time.localtime())
       file_age = now - m_time
       restart_server = (file_age_limit < file_age) | new_start
       new_start = False

   if restart_server:
       log("Age of usrp_server status file is {} seconds. Restarting all processes (with srr.py) ...".format(file_age))
       time.sleep(10) # in case USRP_driver just shut down
       srr.restart_all()
       time.sleep(wait_after_restart_all)
       usrp_driver_watcher.last_restart = time.mktime(time.localtime())

   else:
       # check if cuda driver is running
       cuda_processes = srr.get_cuda_driver_processes()
       if len(cuda_processes) == 0:
           if watch_usrp_server:
               log("No cuda driver found. Restarting all drivers and server...")
               srr.restart_all()
               time.sleep(wait_after_restart_all)
           else:
               log("No cuda driver found. Restarting all drivers in 10 s ...")
               time.sleep(10) # in case USRP_driver just shut down
               srr.restart_driver()
               time.sleep(wait_after_restart_driver)
           usrp_driver_watcher.last_restart = time.mktime(time.localtime())

       else:
           # check USRPSs
           usrp_driver_watcher.check_processes()
           usrp_driver_watcher.restart_usrps()

   if killer.kill_now:
     log("Received SIGTERM or SIGKILL, shutting down watchdog.")
     break

   time.sleep(check_period)

