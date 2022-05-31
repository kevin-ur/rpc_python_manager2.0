
# from lib.PythonASCIICommander import PythonASCIICommander
# from lib.PythonCommanderHelper import PythonCommanderHelper
# from lib.S7_IO import PLC
# from lib.S7_IO import PPP_cell_IO
# import time

# import time
# from datetime import datetime

# from threading import Lock
# from concurrent.futures import ThreadPoolExecutor
# import threading, time
# from ABB_IO_INTERFACE import ABB_IO_INTERFACE
# from threading import Thread

#!/usr/bin/env python3

from pickle import TRUE
from lib.PythonASCIICommander import PythonASCIICommander
from lib.PythonCommanderHelper import PythonCommanderHelper
from lib.S7_IO import PLC
from lib.S7_IO import PPP_cell_IO
import time

import time
from datetime import datetime

from threading import Lock
from concurrent.futures import ThreadPoolExecutor

# p_pick_full_tray_irb1300_tweak=[233.547,-2020.846,24.46,180,0,91.27]
# mi=p_pick_full_tray_irb1300_tweak+[0,0,30,0,0,0]
# mi=p_pick_full_tray_irb1300_tweak+[0,0,30,0,0,0]
# def foo(bar):
#     print ('hello {0}'.format(bar))
#     return "foo"

# class ThreadWithReturnValue(Thread):
#     def __init__(self, group=None, target=None, name=None,
#                  args=(), kwargs={}, Verbose=None):
#         Thread.__init__(self, group, target, name, args, kwargs, Verbose)
#         self._return = None
#     def run(self):
#         if self._Thread__target is not None:
#             self._return = self._Thread__target(*self._Thread__args,
#                                                 **self._Thread__kwargs)
#     def join(self):
#         Thread.join(self)
#         return self._return

# twrv = ThreadWithReturnValue(target=foo, args=('world!',))

# twrv.start()
# print (twrv.join())   # prints foo


print("HelloWorld")

def Robot1Prog():
    i=0
    while(i<4):
        i=i+1
        time.sleep(2)
        
        # print("thread is running")
    return "Hello"
tasks=[]
list1=[0,0,0]
list2=[0,0,0,100]
tasks.append(list1)
tasks.append(list2)
executor = ThreadPoolExecutor(max_workers=4)
    #*****3enable each thread for each robot***********************************************************
# threads = [None]
#     #Robot1--Thread，  if needed ,uncomment                                                      #*
# future1 =executor.submit(Robot1Prog)                  #*
# threads[0]=future1
# while(True):
#     res = threads[0].done()
#     # code = threads[0].result()
#     if res:
#         code = threads[0].result()
#         if code == 0:
#             print("TASK COMPLETED")
#         else:
#             print("TASK ON GOING")
#         break
#         # print("TASK ON GOING++++++++++++")
move_list=[]

move = 'CombinedMove'
move_list.append(move)
move = 'SetRobotPreset'

move_list.append(move)
print("TASK ON GOING")

pick_dest=2
if pick_dest not in range(1,3):
    print ("[RTR_CMD] Invalid cube position index")
    


# Input
# search_name = input("Enter a name for searching: ")
# found = False
# name_list = ["Mary", "John", "Peter", "kelly"]

# for name in name_list:
#     if name == search_name:
#        found = True
#        break

# print("Found") if found else print("Not found")
