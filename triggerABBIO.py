#!/usr/bin/env python3

from lib.PythonASCIICommander import PythonASCIICommander
from lib.PythonCommanderHelper import PythonCommanderHelper
from lib.S7_IO import PLC
from lib.S7_IO import PPP_cell_IO
import time

import time
from datetime import datetime

from threading import Lock
from concurrent.futures import ThreadPoolExecutor
import threading, time
from ABB_IO_INTERFACE import ABB_IO_INTERFACE
lock = Lock()
Server_IP="192.168.1.2"
Server_Port=1025
ABB_IO=ABB_IO_INTERFACE(Server_IP,Server_Port,simulation=False)
#ABB_IO.vacuum_on()

ABB_IO.vacuum_off()

