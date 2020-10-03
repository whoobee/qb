#!/usr/bin/python

#os stuff and multithreading
import os
import time
import signal
from subprocess import Popen
import threading
#required for button data base
from collections import namedtuple

#get script directory
CURRENT_DIR = os.path.dirname(os.path.realpath(__file__))
#button event tuple
Event = namedtuple("Event", "timestamp name counter")

#ros script path and log path
SCRIPT_PATH = ""
LOGS_PATH = ""

#process var
ROS_CORE_NODE_PROC = None
ROS_QB_STATE_PROC = None
ROS_NODE_PROC = None

#state var
NODE_STARTED = False
REGISTERED_EVENT = 0
SAFE_FOR_EXEC = False


#initializer
def init_button_state():
    #delete events before start
    if (os.path.exists(CURRENT_DIR+"/resources/powerbtn.event")):
        os.remove(CURRENT_DIR+"/resources/powerbtn.event")
    #delete logs before start
    if (os.path.exists(LOGS_PATH)):
        logs = os.listdir(LOGS_PATH)
        for log in logs:
            if log.endswith(".log"):
                os.remove(os.path.join(LOGS_PATH, log))

#porocess starter
def run(cmd, stdout, stderr):
    """Run a given `cmd` in a subprocess, write logs to stdout / stderr.

    Parameters
    ----------
    cmd : list of str
        Command to run.
    stdout : str or subprocess.PIPE object
        Destination of stdout output.
    stderr : str or subprocess.PIPE object
        Destination of stderr output.

    Returns
    -------
    A subprocess.Popen instance.
    """
    return Popen(cmd, stdout=stdout, stderr=stderr, shell=False,
                 preexec_fn=os.setsid)

#logger
def get_stdout_stderr(typ, datetime, dir):
    """Create stdout / stderr file paths."""
    out = '%s_%s_stdout.log' % (datetime, typ)
    err = '%s_%s_stderr.log' % (datetime, typ)
    return os.path.join(dir, out), os.path.join(dir, err)

#file sanity checker
def check_files_exist(files):
    """Check if given list of files exists.

    Parameters
    ----------
    files : list of str
        Files to check for existence.

    Returns
    -------
    None if all files exist. Else raises a ValueError.
    """
    errors = []
    for f in files:
        if not os.path.exists(f):
            errors.append(f)
    if errors:
        raise ValueError('File does not exist: %s' % errors)

#start process function
def start_process(cmd, typ, start_time, dpath_logs):
    """Start a subprocess with the given command `cmd`.

    Parameters
    ----------
    cmd : list of str
        Command to run.
    typ : str
        Type of subprocess. This will be included in the logs' file names.
    start_time : str
        Datetime string, will be included in the logs' file names as well as
        the resulting bag's name.
    dpath_logs :
        Path to log direcotry.

    Returns
    -------
    A subprocess.Popen instance.
    """
    print('Starting', typ.upper())
    stdout, stderr = get_stdout_stderr(typ, start_time, dpath_logs)
    with open(stdout, 'wb') as out, open(stderr, 'wb') as err:
        return run(cmd, stdout=out, stderr=err)



#read power button status from file
def button_event_handler(file):
    presscnt = 0
    btnevents = []
    if (os.path.exists(file)):
        #read the recorded event from the button event listener
        with open(file, "r+") as f:
            lines = [line.rstrip() for line in f]
        #save all events into the database 
        for line in lines:
            ts = int(line.split('[')[1].split(']')[0])
            evn = line.split('[')[1].split(']')[1].split(':')[0]
            cnt = int(line.split('[')[1].split(']')[1].split(':')[1], base=16)
            event = Event(ts,evn,cnt)
            btnevents.append(event)
        #detect event based on timestamp of the press
        if (len(btnevents) != 0):
            cts = btnevents[0].timestamp
            presscnt = 1
            for ev in btnevents:
                pts = cts
                cts = ev.timestamp
                if (cts-pts > 0 and cts-pts < 1000):
                    presscnt = presscnt + 1
                elif (cts-pts > 1000):
                    presscnt = 1
            #print(presscnt)
    return presscnt

#read button event (number of presses) - repeats every 1s
def get_button_state():
    global REGISTERED_EVENT
    global SAFE_FOR_EXEC
    #get the button press event
    #here we read twice to exclude the fact that we read in between 2 presses
    if(SAFE_FOR_EXEC == False):
        REGISTERED_EVENT = button_event_handler(CURRENT_DIR+"/resources/powerbtn.event")
        time.sleep(2)
        REGISTERED_EVENT = button_event_handler(CURRENT_DIR+"/resources/powerbtn.event")
        if(REGISTERED_EVENT > 0):
            SAFE_FOR_EXEC = True
    #execute every 1s
    threading.Timer(1, get_button_state).start()

#execute task based on received button event - repeats every 100ms
def exec_state_handler():
    global REGISTERED_EVENT
    global SAFE_FOR_EXEC
    global NODE_STARTED
    #if event was detected earlyer
    if(SAFE_FOR_EXEC == True):
        SAFE_FOR_EXEC = False
        #if only 1 button press was detected
        if(REGISTERED_EVENT == 1):
            #if event is valid
            if(NODE_STARTED == True):
                #stop the ROS launch node - node already running
                NODE_STARTED = False
                stop_ros_node()
            else:
                #start the ROS launch node
                NODE_STARTED = True
                start_ros_node()
        #if 2 button presses were detected
        elif (REGISTERED_EVENT == 2):
            #shutdown only in case ROS launcer node is not running
            if(NODE_STARTED == False):
                shutdown_system()
        #clear the event list in case it is not empty
        open(CURRENT_DIR+"/resources/powerbtn.event", 'w').close()
    #execute every 100ms
    threading.Timer(0.1, exec_state_handler).start()

#execution task - start the ros core node and qb_state_manager
def start_ros_state_node():
    global LOGS_PATH
    global SCRIPT_PATH
    global ROS_CORE_NODE_PROC
    global ROS_QB_STATE_PROC
    #sanity check
    check_files_exist([SCRIPT_PATH, LOGS_PATH])
    start_time = time.strftime('%Y%m%d_%H%M%S')
    #start the ROS core and qB-state-manager process
    ROS_CORE_NODE_PROC = start_process(['/home/whoobee/qb/scripts/powerbtn/powerbtn_roscore.sh'],
                                'ros', start_time, LOGS_PATH)
    print('PGID ROS: ', os.getpgid(ROS_CORE_NODE_PROC.pid))
    time.sleep(2)
    ROS_QB_STATE_PROC = start_process(['/home/whoobee/qb/scripts/powerbtn/powerbtn_qb_state_manager.sh'],
                                'qb_state', start_time, LOGS_PATH)
    print('PGID qB-STATE: ', os.getpgid(ROS_QB_STATE_PROC.pid))

#execution task - start the qb ros launcher
def start_ros_node():
    global LOGS_PATH
    global SCRIPT_PATH
    global ROS_CORE_NODE_PROC
    global ROS_NODE_PROC
    #sanity check
    check_files_exist([SCRIPT_PATH, LOGS_PATH])
    start_time = time.strftime('%Y%m%d_%H%M%S')
    #start the ROS launcher process
    ROS_NODE_PROC = start_process(['bash', SCRIPT_PATH],
                               'talker_node', start_time,
                               LOGS_PATH)
    print('PGID TALKER NODE: ', os.getpgid(ROS_NODE_PROC.pid))

#execution task - stop the qb ros launcher
def stop_ros_node():
    global ROS_CORE_NODE_PROC
    global ROS_NODE_PROC
    
    print('Killing ROS and talker node.')
    #kill ros processes
    #os.killpg(os.getpgid(ROS_CORE_NODE_PROC.pid), signal.SIGTERM)
    os.killpg(os.getpgid(ROS_NODE_PROC.pid), signal.SIGTERM)

#execution task - shutdown system
def shutdown_system():
    print("Shutting Down the System")
    os.system("shutdown -h now") 

#main thread
def main(args):
    global LOGS_PATH
    global SCRIPT_PATH

    LOGS_PATH = args.dpath_logs
    SCRIPT_PATH = args.script_node

    #wait 4s for making sure that the network is up
    time.sleep(4)
    #start the ros core and qb_state_manager nodes
    start_ros_state_node()
    #init the button state and logs
    init_button_state()
    #start the button event listener
    get_button_state()
    #start the button event handler
    exec_state_handler()
    #infinite loop
    while(1):
        time.sleep(1)
        

#init
if __name__ == '__main__':
    """Start ROS and the talker node each as a subprocess.

    Examples
    --------

    python  start_ros.py --script_node /qb/scripts/test.sh -l /qb/scripts/log/scripts
    """
    import argparse

    #argument parser
    parser = argparse.ArgumentParser()
    parser.add_argument('--script_node', type=str,
                        default='/notebooks/workspace/talker.sh')
    parser.add_argument('--dpath_logs', '-l', type=str,
                        default='/notebooks/workspace/src/scripts')
    args = parser.parse_args()
    #start main func
    main(args)