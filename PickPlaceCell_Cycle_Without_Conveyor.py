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

lock = Lock()

def handle_response(response: PythonASCIICommander.CommandResult) -> None:
    if response.delayed_response is not None:
        delayed_response = response.delayed_response.get()
        return delayed_response.error_code
    return response.error_code

def waitForDelayedResponse(result: PythonASCIICommander.CommandResult):
    while result.delayed_response is not None:
        while not result.delayed_response.is_ready():
            time.sleep(0)
        result = result.delayed_response.get()
    return result.error_code

def waitForCombinedMoveResponse(result: PythonASCIICommander.CommandResult, n_moves, move_n, robot = None, device = None, value = None):

    print ('combined move contains ' + str(n_moves) + ' moves')
    n = 0
    while result.delayed_response is not None:
        while not result.delayed_response.is_ready():
            time.sleep(0)
        result = result.delayed_response.get()
        
        if result.is_feedback():
            print('     intermediate response [' + str(n) + ']: ' + str(result.error_code))
            if n == move_n - 1:
                if device is not None:
                    if value == 0:
                        print ('     Setting ' + robot + ' ' + device + ' off')
                    elif value == 1:
                        print ('     Setting ' + robot + ' ' + device + ' on')
                    if device == 'vacuum' and value == 0:
                        workcell_io.set_vacuum_off(robot)
                    elif device == 'vacuum' and value == 1:
                        workcell_io.set_vacuum_on(robot)
                    elif device == 'gripper' and value == 0:
                        workcell_io.open_gripper()
                    elif device == 'gripper' and value == 1:
                        workcell_io.close_gripper()
                    elif device == 'conveyor' and value == 0:
                        workcell_io.stop_conveyor()
                    elif device == 'conveyor' and value == 1:
                        workcell_io.start_conveyor()
                else:
                    time.sleep(0)
            n = n + 1
    print('\n')
    return result.error_code

class TaskPlanner:

    def __init__(self, rtc_ip_address, plc_io, fp):
        self.ip = rtc_ip_address
        self.io = plc_io
        self.cmd = PythonASCIICommander(self.ip, 9999)
        self.cmd_1 = PythonASCIICommander(self.ip, 9999)
        self.cmd_2 = PythonASCIICommander(self.ip, 9999)
        self.cmd_3 = PythonASCIICommander(self.ip, 9999)
        self.cmd_4 = PythonASCIICommander(self.ip, 9999)
        self.speed_factor = 0.1
        self.helper = PythonCommanderHelper(self.ip)
        project_info = self.helper.get_project_info()
        self.robots = project_info['robots']
        self.robots = ["UR5","RV5AS","IRB1200","IRB1300"]
        self.abb_offset_pick=[0,0,0,0,0,10,8]
        self.ur5_offset_pick=[0,5,5,5,0,0,0]
        self.melfa_offset_pick=[0,0,0,0,0,0,0]
        self.target = self.cmd.MoveGoalType.TARGET
        self.pose = self.cmd.MoveGoalType.POSE

        self.abb_index = [1,2,3]
        self.ur_index = [1,2,3]
        self.melfa_index = [6,5,4]
        


        self.init_logging(fp)
        self.cmd.connect()
        self.EnterOperationMode()

        ret = self.SetRobotPreset('UR5', 'tray_pick')
        if ret != 0:
            return ret

        ret = self.SetRobotPreset('RV5AS', 'vacuum_pick')
        if ret != 0:
            return ret

        ret = self.SetRobotPreset('IRB1200', 'vacuum_pick')
        if ret != 0:
            return ret

        ret = self.Move('UR5', 'target', 'ur5_actuator_home', 'planning')
        if ret != 0:
            return ret
        # ret = self.Move('UR5', 'target', 'ur5_vacuum_home', 'planning')
        # if ret != 0:
        #     return ret    

        ret = self.Move('RV5AS', 'target', 'melfa_standby', 'planning')
        if ret != 0:
            return ret

        ret = self.Move('IRB1200', 'target', 'abb_home', 'planning')
        if ret != 0:
            return ret

    def init_logging(self,fp):
        self.fp = fp
        self.fp.write('\n')

    def log(self,msg):
        log_msg = f'[{datetime.now()}] {msg}\n'
        self.fp.write(log_msg)
        print(log_msg)

    def advance_task(self,robot_idx):
        robot = self.robots[robot_idx]
        task_list = self.tasks[robot_idx]
        arg_list = self.args[robot_idx]
        task_idx = self.task_idxs[robot_idx]

        if task_idx > len(task_list)-1:
            self.end_times[robot_idx] = time.time()
            self.pick_and_place[robot_idx] = False
            self.threads[robot_idx] = None
            self.log(f'Robot {self.robots[robot_idx]} has finished!')
        else:


            move = task_list[task_idx]
            args = arg_list[task_idx]

            print (str(task_idx) + ': ' + str(move))
            if move == 'SetRobotPreset':
                future = self.executor.submit(self.SetRobotPreset,args[0], args[1])
            elif move == 'SetObjectState':
                future = self.executor.submit(self.SetObjectStates,args[0], args[1])
            elif move == 'SetVacuumOn':
                future = self.executor.submit(self.io.set_vacuum_on, args)
            elif move == 'SetVacuumOff':
                future = self.executor.submit(self.io.set_vacuum_off, args)
            elif move == 'Move':
                future = self.executor.submit(self.Move, args[0], args[1], args[2], args[3], relative_pose = args[4], blind_move = args[5])
            elif move == 'CombinedMove':
                if len(args) == 10:
                    future = self.executor.submit(self.CombinedMove, args[0], args[1], args[2], args[3], args[4], args[5], args[6], args[7], args[8], actuate_device = args[9])
                elif len(args) == 13:
                   future = self.executor.submit(self.CombinedMove, args[0], args[1], args[2], args[3], args[4], args[5], args[6], args[7], args[8], actuate_device = args[9], actuation_timing=args[10], device=args[11], value=args[12])
 
            self.threads[robot_idx] = future

    def start(self, abb_flag, ur_flag, melfa_flag):

        self.tasks = []
        self.args = []
        
        [move_list_0_1, arg_list_0_1] = generate_put_cube_to_tray_task('UR5', self.ur_index[0], 3)
        [move_list_0_2, arg_list_0_2] = generate_put_cube_to_tray_task('UR5', self.ur_index[1], 2)
        [move_list_0_3, arg_list_0_3] = generate_put_cube_to_tray_task('UR5', self.ur_index[2], 1)

        [move_list_1_1, arg_list_1_1] = generate_put_cube_to_tray_task('RV5AS', self.melfa_index[0], 3)
        [move_list_1_2, arg_list_1_2] = generate_put_cube_to_tray_task('RV5AS', self.melfa_index[1], 2)
        [move_list_1_3, arg_list_1_3] = generate_put_cube_to_tray_task('RV5AS', self.melfa_index[2], 1)


        [move_list_2_1, arg_list_2_1] = generate_put_cube_to_tray_task('IRB1200', self.abb_index[0], 1)
        [move_list_2_2, arg_list_2_2] = generate_put_cube_to_tray_task('IRB1200', self.abb_index[1], 2)
        [move_list_2_3, arg_list_2_3] = generate_put_cube_to_tray_task('IRB1200', self.abb_index[2], 3)

       

        move_list_0 = move_list_0_1 + move_list_0_2 + move_list_0_3
        arg_list_0 = arg_list_0_1 + arg_list_0_2 + arg_list_0_3

        move_list_1 = move_list_1_1 + move_list_1_2 + move_list_1_3
        arg_list_1 = arg_list_1_1 + arg_list_1_2 + arg_list_1_3
        
        move_list_2 = move_list_2_1 + move_list_2_2 + move_list_2_3
        arg_list_2 = arg_list_2_1 + arg_list_2_2 + arg_list_2_3

        if not abb_flag:
            move_list_2 = []
            arg_list_2 = []
        
        if not ur_flag:
            move_list_0 = []
            arg_list_0 = []
        
        if not melfa_flag:
            move_list_1 = []
            arg_list_1 = []

        self.tasks.append(move_list_0)
        self.tasks.append(move_list_1)
        self.tasks.append(move_list_2)

        self.args.append(arg_list_0)
        self.args.append(arg_list_1)
        self.args.append(arg_list_2)

        self.executor = ThreadPoolExecutor(max_workers=4)
        self.threads = [None] * (len(self.robots)-1)
        self.pick_and_place = [True]*(len(self.robots)-1) # cycle time stop watch
        self.end_times = [None]*(len(self.robots)-1) # cycle time stop watch
        self.task_idxs = [0]*(len(self.robots)-1) # current index of the hub list
        self.interlocking = [False]*(len(self.robots)-1) # project idx of the robot that had to retreat

        self.start_time = time.time()

        self.advance_task(0)
        self.advance_task(1)
        self.advance_task(2)

        while True in self.pick_and_place:
            for robot_idx in range(0,len(self.robots)-1):
                if self.pick_and_place[robot_idx]:
                    res = self.threads[robot_idx].done()
                    if res:
                        code = self.threads[robot_idx].result()
                        if code == 0:
                            self.log(f'Robot {self.robots[robot_idx]} completed {self.tasks[robot_idx][self.task_idxs[robot_idx]]}!')
                            self.task_idxs[robot_idx] += 1   
                            self.advance_task(robot_idx)
                        else:
                            self.advance_task(robot_idx)

    def unpack(self, abb_flag, ur_flag, melfa_flag,abb_put_back_index=[1,2,3],ur_put_back_index=[1,2,3],melfa_put_back_index=[1,2,3]):

        self.tasks = []
        self.args = []
        
        [move_list_0_1, arg_list_0_1] = generate_put_tray_to_cube_task('UR5', 1, ur_put_back_index[0])
        [move_list_0_2, arg_list_0_2] = generate_put_tray_to_cube_task('UR5', 2, ur_put_back_index[1])
        [move_list_0_3, arg_list_0_3] = generate_put_tray_to_cube_task('UR5', 3, ur_put_back_index[2])

        [move_list_1_1, arg_list_1_1] = generate_put_tray_to_cube_task('RV5AS', 1, melfa_put_back_index[0])
        [move_list_1_2, arg_list_1_2] = generate_put_tray_to_cube_task('RV5AS', 2, melfa_put_back_index[1])
        [move_list_1_3, arg_list_1_3] = generate_put_tray_to_cube_task('RV5AS', 3, melfa_put_back_index[2])


        [move_list_2_1, arg_list_2_1] = generate_put_tray_to_cube_task('IRB1200', 1, abb_put_back_index[0])
        [move_list_2_2, arg_list_2_2] = generate_put_tray_to_cube_task('IRB1200', 2, abb_put_back_index[1])
        [move_list_2_3, arg_list_2_3] = generate_put_tray_to_cube_task('IRB1200', 3, abb_put_back_index[2])

     

        move_list_0 = move_list_0_1 + move_list_0_2 + move_list_0_3
        arg_list_0 = arg_list_0_1 + arg_list_0_2 + arg_list_0_3

        move_list_1 = move_list_1_1 + move_list_1_2 + move_list_1_3
        arg_list_1 = arg_list_1_1 + arg_list_1_2 + arg_list_1_3
        
        move_list_2 = move_list_2_1 + move_list_2_2 + move_list_2_3
        arg_list_2 = arg_list_2_1 + arg_list_2_2 + arg_list_2_3

        if not abb_flag:
            move_list_2 = []
            arg_list_2 = []
        
        if not ur_flag:
            move_list_0 = []
            arg_list_0 = []
        
        if not melfa_flag:
            move_list_1 = []
            arg_list_1 = []

        self.tasks.append(move_list_0)
        self.tasks.append(move_list_1)
        self.tasks.append(move_list_2)

        self.args.append(arg_list_0)
        self.args.append(arg_list_1)
        self.args.append(arg_list_2)

        self.executor = ThreadPoolExecutor(max_workers=4)
        self.threads = [None] * (len(self.robots)-1)
        self.pick_and_place = [True]*(len(self.robots)-1) # cycle time stop watch
        self.end_times = [None]*(len(self.robots)-1) # cycle time stop watch
        self.task_idxs = [0]*(len(self.robots)-1) # current index of the hub list
        self.interlocking = [False]*(len(self.robots)-1) # project idx of the robot that had to retreat

        self.start_time = time.time()

        self.advance_task(0)
        self.advance_task(1)
        self.advance_task(2)

        while True in self.pick_and_place:
            for robot_idx in range(0,len(self.robots)-1):
                if self.pick_and_place[robot_idx]:
                    res = self.threads[robot_idx].done()
                    if res:
                        code = self.threads[robot_idx].result()
                        if code == 0:
                            self.log(f'Robot {self.robots[robot_idx]} completed {self.tasks[robot_idx][self.task_idxs[robot_idx]]}!')
                            self.task_idxs[robot_idx] += 1   
                            self.advance_task(robot_idx)
                        else:
                            self.advance_task(robot_idx)



    def SetSpeedFactor(self, speed):
        self.speed_factor = speed

    def SetABBCubePickIndex(self, index):
        self.abb_index = index

    def SetURCubePickIndex(self, index):
        self.ur_index = index

    def SetMelfaCubePickIndex(self, index):
        self.melfa_index = index

    def EnterOperationMode(self):
        res = self.cmd.enter_operation_mode()
        code = res.error_code
        if code != 0:
            print ('[Error] Entering to operation mode failed')
            print ('[RTR] Return Code: ' + str(code))
            return code
        
        return 0

    def SetObjectStates(self, object_list, object_state_list):

        if len(object_list) != len(object_state_list):
            print ('[Error] The sizes of the object list and object state list do not match')
            return -1
        
        return_codes = []
        for i in range(0, len(object_list)):
            code = -1
            while code != 0:
                with lock:
                    res = self.cmd.set_object_state(object_list[i], object_state_list[i])
                code = handle_response(res)
                time.sleep(0.1)
            print ("[RTR] SetObjectStates (" + str(object_list[i]) + ", " + str(object_state_list[i]) + "): " + str(code))
            return_codes.append(code)
        
        sum_err = 0
        for code in return_codes:
            sum_err = sum_err + code
        
        if sum_err != 0:
            return return_codes
        else:
            return 0 

    def SetRobotPreset(self, robot, preset):

        print ("Executing Set Robot Preset")

        with lock:
            if robot == self.robots[0]:
                res = self.cmd_1.set_robot_preset(robot, preset)
            if robot == self.robots[1]:
                res = self.cmd_2.set_robot_preset(robot, preset)
            if robot == self.robots[2]:
                res = self.cmd_3.set_robot_preset(robot, preset)
            if robot == self.robots[3]:
                res = self.cmd_3.set_robot_preset(robot, preset)    
            code = handle_response(res)
            print ("Executing Set Robot Preset: " + str(code) )   
        return code     

    def Move(self, robot, goal_typ, goal_val, move_typ, blind_move = False, relative_pose = False, speed = 1.0,smooth=0.0):

        col_check = True
        if blind_move:
            col_check = False
        code = -1
        while code == -1:

            if goal_typ == 'pose' or goal_typ == 'POSE' or goal_typ == 'p' or goal_typ == 'P':
                if robot == self.robots[0]:
                    with lock:
                        res = self.cmd_1.move( robot_name = robot, speed = self.speed_factor * speed, goal_type=self.pose, 
                        goal_value=goal_val, relative = relative_pose, smoothing=smooth, move_type=move_typ, interp='j', collision_check=col_check,collision_check_dsm=True)
                    code = handle_response(res)

                elif robot == self.robots[1]:
                    with lock:
                        res = self.cmd_2.move( robot_name = robot, speed = self.speed_factor * speed, goal_type=self.pose, 
                        goal_value=goal_val, relative = relative_pose, smoothing=smooth, move_type=move_typ, interp='j', collision_check=col_check,collision_check_dsm=True)
                    code = handle_response(res)
                elif robot == self.robots[2]:
                    with lock:
                        res = self.cmd_3.move( robot_name = robot, speed = self.speed_factor * speed, goal_type=self.pose, 
                        goal_value=goal_val, relative = relative_pose, smoothing=smooth, move_type=move_typ, interp='j', collision_check=col_check,collision_check_dsm=True)
                    code = handle_response(res)
                elif robot == self.robots[3]:
                    with lock:
                        res = self.cmd_4.move( robot_name = robot, speed = self.speed_factor * speed, goal_type=self.pose, 
                        goal_value=goal_val, relative = relative_pose, smoothing=smooth, move_type=move_typ, interp='j', collision_check=col_check,collision_check_dsm=True)
                    code = handle_response(res)    
                        
            elif goal_typ == 'target' or goal_typ == 'Target' or goal_typ == 'TARGET' or goal_typ == 't' or goal_typ == 'T':
                if robot == self.robots[0]:
                    with lock:
                        res = self.cmd_1.move( robot_name = robot, speed = self.speed_factor * speed, goal_type=self.target, 
                        goal_value=goal_val, smoothing=smooth, move_type=move_typ, interp='j', collision_check=col_check, collision_check_dsm=True)
                    code = handle_response(res)
                elif robot == self.robots[1]:
                    with lock:
                        res = self.cmd_2.move( robot_name = robot, speed = self.speed_factor * speed, goal_type=self.target, 
                        goal_value=goal_val, smoothing=smooth, move_type=move_typ, interp='j', collision_check=col_check, collision_check_dsm=True)
                    code = handle_response(res)
                elif robot == self.robots[2]:
                    with lock:
                        res = self.cmd_3.move( robot_name = robot, speed = self.speed_factor * speed, goal_type=self.target, 
                        goal_value=goal_val, smoothing=smooth, move_type=move_typ, interp='j', collision_check=col_check, collision_check_dsm=True)
                    code = handle_response(res)
                elif robot == self.robots[3]:
                    with lock:
                        res = self.cmd_4.move( robot_name = robot, speed = self.speed_factor * speed, goal_type=self.target, 
                        goal_value=goal_val, smoothing=smooth, move_type=move_typ, interp='j', collision_check=col_check, collision_check_dsm=True)
                    code = handle_response(res)
                     
            print ("[RTR] Move (" + str(robot) + ", " + str(goal_val) + "): " + str(code))
            #time.sleep(0.5)
        return code

    def CombinedMove(self, robot, goal_types, goal_values, move_types, rel_pose_flags, smoothing_values, col_check_flags, speed_factors, interps, actuate_device = False, actuation_timing = None, device = None, value = None):

        combined_moves = []

        for x in range(0, len(goal_types)):
            with lock:
                if goal_types[x] == 'pose' or goal_types[x] == 'POSE' or goal_types[x] == 'p' or goal_types[x] == 'P':
                    g_type = self.pose
                elif goal_types[x] == 'target' or goal_types[x] == 'Target' or goal_types[x] == 'TARGET' or goal_types[x] == 't' or goal_types[x] == 'T':
                    g_type = self.target
                if robot == self.robots[0]:
                    move = self.cmd_1.generate_move_data_dictionary(
                        robot_name=robot, speed = self.speed_factor * speed_factors[x], goal_type=g_type, goal_value=goal_values[x], move_type=move_types[x], relative=rel_pose_flags[x], smoothing=smoothing_values[x], collision_check=col_check_flags[x], interp=interps[x])
                elif robot == self.robots[1]:
                    move = self.cmd_2.generate_move_data_dictionary(
                        robot_name=robot, speed = self.speed_factor * speed_factors[x], goal_type=g_type, goal_value=goal_values[x], move_type=move_types[x], relative=rel_pose_flags[x], smoothing=smoothing_values[x], collision_check=col_check_flags[x], interp=interps[x])
                elif robot == self.robots[2]:
                    move = self.cmd_3.generate_move_data_dictionary(
                        robot_name=robot, speed = self.speed_factor * speed_factors[x], goal_type=g_type, goal_value=goal_values[x], move_type=move_types[x], relative=rel_pose_flags[x], smoothing=smoothing_values[x], collision_check=col_check_flags[x], interp=interps[x])
                elif robot == self.robots[3]:
                    move = self.cmd_4.generate_move_data_dictionary(
                        robot_name=robot, speed = self.speed_factor * speed_factors[x], goal_type=g_type, goal_value=goal_values[x], move_type=move_types[x], relative=rel_pose_flags[x], smoothing=smoothing_values[x], collision_check=col_check_flags[x], interp=interps[x])
                     
                
            combined_moves.append(move)

        code = -1
        #while code != 0:
        with lock:
            if robot == self.robots[0]:
                res = self.cmd_1.combined_move(robot, combined_moves)
            if robot == self.robots[1]:
                res = self.cmd_2.combined_move(robot, combined_moves)
            if robot == self.robots[2]:
                res = self.cmd_3.combined_move(robot, combined_moves)
            if robot == self.robots[3]:
                res = self.cmd_4.combined_move(robot, combined_moves)

        if actuate_device==False:
            code = waitForCombinedMoveResponse(res, len(combined_moves), len(combined_moves)-1)
        elif actuate_device==True:
            code = waitForCombinedMoveResponse(res, len(combined_moves), actuation_timing, robot, device, value)
        print ("[RTR_CMD] CombinedMove (" + str(robot) + ", " + str(goal_values) + "): " + str(code))
        #time.sleep(0.5)
        return code

def Initialize_Objects(rtr):
                          
    objects = ['placed_yellow_cube_01','placed_yellow_cube_02','placed_yellow_cube_03',
               'placed_green_cube_01','placed_green_cube_02','placed_green_cube_03',
               'placed_purple_cube_01','placed_purple_cube_02','placed_purple_cube_03',
               'tray01','tray02','tray03','tray04','tray05',
               'fulltray_on_front_conveyor','fulltray_on_front_conveyor_in',
               'purple_cubic_01','purple_cubic_02','purple_cubic_03','purple_cubic_04','purple_cubic_05','purple_cubic_06',
               'yellow_cubic_01','yellow_cubic_02','yellow_cubic_03','yellow_cubic_04','yellow_cubic_05','yellow_cubic_06',
               'green_cubic_01','green_cubic_02','green_cubic_03','green_cubic_04','green_cubic_05','green_cubic_06']

    states = ['Suppressed', 'Suppressed', 'Suppressed',
              'Suppressed', 'Suppressed', 'Suppressed',
              'Suppressed', 'Suppressed', 'Suppressed',
              'ready','ready','ready','ready','Suppressed',
              'Suppressed','Suppressed',
              'ready','ready','ready','ready','ready','ready',
              'ready','ready','ready','ready','ready','ready',
              'ready','ready','ready','ready','ready','ready']


    ret = rtr.SetObjectStates(objects, states)
    if ret != 0:
        return ret
    
    return 0

def UR5_Set_Tray(rtr, tray):

    robot = 'UR5'
    tray_obj = []
    tray_pick = []
    tray_pick_approach = []

    if tray not in [1,2,3,4]:
        print ("[RTR_CMD] Invalid tray input")
        return -1
    
    home = 'ur5_actuator_home'
    tray_pick_approach = 'tray_' + str(tray) + '_pick_approach'

    tray_pick = 'tray_' + str(tray) + '_pick'

    ret = rtr.SetRobotPreset('UR5', 'tray_pick')
    if ret != 0:
        return ret

    goal_types = ['target', 'target','target']
    goal_values = [home, tray_pick_approach,tray_pick]
    move_types = ['roadmap','roadmap','roadmap']
    rel_pose_flags = [False, False, False]
    smoothing_values = [10.0, 10.0,0]
    col_check_flags = [True, True,True]
    speed_factors = [1.0, 1.0,1.0]
    interps = ['j','j','j']
    act_device = True

    ret = rtr.CombinedMove(robot, goal_types, goal_values, move_types, rel_pose_flags, smoothing_values, col_check_flags, speed_factors, interps, actuate_device=False)
    if ret != 0:
        return ret

    ret = rtr.io.close_gripper()
    
    pick_tray_object = 'tray0' + str(tray)
    ret = rtr.SetObjectStates( [pick_tray_object],['Suppressed'])
    if ret != 0:
        return ret


    ret = rtr.SetRobotPreset('UR5', 'tray_place')
    if ret != 0:
        return ret 

    goal_types = ['pose', 'target', 'pose']
    goal_values = [[0, 0, 60, 0, 0, 0], 'tray_place_approach', [0,0,-35,0,0,0]]
    move_types = ['direct','roadmap', 'direct']
    rel_pose_flags = [True, False, True]
    smoothing_values = [10.0, 10.0, 10.0]
    col_check_flags = [False, True, False]
    speed_factors = [1.0, 1.0, 1.0]
    interps = ['j','j','j']

    ret = rtr.CombinedMove(robot, goal_types, goal_values, move_types, rel_pose_flags, smoothing_values, col_check_flags, speed_factors, interps, actuate_device=False)
    if ret != 0:
        return ret

    ret = rtr.io.open_gripper()

    ret = rtr.SetRobotPreset('UR5', 'tray_pick')
    if ret != 0:
        return ret
    ret = rtr.SetObjectStates( ['tray05'],['ready'])
    if ret != 0:
        return ret
    goal_types = ['pose', 'target', 'target']
    goal_values = [[0, 0, 35, 0, 0, 0], 'ur5_actuator_home', 'transition_to_vacuum_home']
    move_types = ['direct','roadmap','roadmap']
    rel_pose_flags = [True, False, False]
    smoothing_values = [10.0, 10.0, 10.0]
    col_check_flags = [False, True, True]
    speed_factors = [1.0, 1.0, 1.0]
    interps = ['j','j','j']

    ret = rtr.CombinedMove(robot, goal_types, goal_values, move_types, rel_pose_flags, smoothing_values, col_check_flags, speed_factors, interps, actuate_device=False)
    if ret != 0:
        return ret

    ret = rtr.SetRobotPreset('UR5','vacuum_pick')
    if ret != 0:
        return ret

    goal_types = ['target']
    goal_values = ['ur5_vacuum_home']
    move_types = ['roadmap']
    rel_pose_flags = [False, False]
    smoothing_values = [10.0, 10.0]
    col_check_flags = [True, True]
    speed_factors = [1.0, 1.0]
    interps = ['j','j']

    ret = rtr.CombinedMove(robot, goal_types, goal_values, move_types, rel_pose_flags, smoothing_values, col_check_flags, speed_factors, interps, actuate_device=False)
    if ret != 0:
        return ret

    return 0

def generate_put_cube_to_tray_task(robot, cube, dest):

    cube_obj = []
    cube_pick = []
    cube_place = []
    placed_cube_obj = []
    pick_cube_obj = []
    if cube not in range(1,13):
        print ("[RTR_CMD] Invalid cube position index")
        return -1

    if robot == 'UR5':
        color = 'yellow'
        home = 'ur_standby'
    elif robot == 'RV5AS':
        color = 'purple'
        home = 'melfa_standby'
    elif robot == 'IRB1200':
        color = 'green'
        home = 'abb_home'

    if dest not in [1,2,3]:
        print ("[RTR_CMD] Invalid tray slot index")
        return -1

    placed_cube_obj = 'placed_' + str(color) + '_cube_0' + str(dest)

    if cube < 10:
        cube_pick = str(color) + '_cube_0' + str(cube) + '_pick_approach'
        pick_cube_obj = str(color) + '_cubic_0' + str(cube)

    else:
        cube_pick = str(color) + '_cube_' + str(cube) + '_pick_approach'
        pick_cube_obj = str(color) + '_cubic_' + str(cube)

    cube_next_index = -1
    cube_next = ''

    if robot == 'UR5': 
        if cube == task_planner.ur_index[0]:
            cube_next_index = task_planner.ur_index[1]
        elif cube == task_planner.ur_index[1]:
            cube_next_index = task_planner.ur_index[2]    
        elif cube == task_planner.ur_index[2]:
            cube_next = 'ur_standby'

    if robot == 'IRB1200': 
        if cube == task_planner.abb_index[0]:
            cube_next_index = task_planner.abb_index[1]
        elif cube == task_planner.abb_index[1]:
            cube_next_index = task_planner.abb_index[2]    
        elif cube == task_planner.abb_index[2]:
            cube_next = 'abb_home'
    
    if robot == 'RV5AS':
        if cube == task_planner.melfa_index[0]:
            cube_next_index = task_planner.melfa_index[1]
        elif cube == task_planner.melfa_index[1]:
            cube_next_index = task_planner.melfa_index[2]    
        elif cube == task_planner.melfa_index[2]:
            cube_next = 'melfa_standby'  

    if cube_next_index != -1:
        if cube_next_index < 10:
            cube_next = str(color) + '_cube_0' + str(cube_next_index) + '_pick_approach'
        else:
            cube_next = str(color) + '_cube_' + str(cube_next_index) + '_pick_approach'       
    print(cube_next)
    cube_place = str(color) + '_cube_place_0' + str(dest) + '_approach'

    ###################################################################################

    move_list = []
    arg_list = []
    # start home--->pick
    if (robot == 'UR5' and cube == task_planner.ur_index[0]) or (robot == 'IRB1200' and cube == task_planner.abb_index[0]) or (robot == 'RV5AS' and cube == task_planner.melfa_index[0]):

        goal_types = ['target', 'pose']
        if robot == 'UR5':

            goal_values = [cube_pick, [0, 0, -48, 0, 0, 0]] #cube1    
        elif robot == 'RV5AS':
            goal_values = [cube_pick, [0, 0, -48, 0, 0, 0]]   ####-43   ---   -48               
        elif robot == 'IRB1200':
            goal_values = [cube_pick, [0, 0, -45, 0, 0, 0]]            
        move_types = ['roadmap', 'direct']
        rel_pose_flags = [False, True]
        smoothing_values = [10.0, 0]
        col_check_flags = [True, False]
        speed_factors = [1.0, 1.0]
        interps = ['j', 'l']
        act_device = True

        move = 'CombinedMove'
        args = [robot, goal_types, goal_values, move_types, rel_pose_flags, smoothing_values, col_check_flags, speed_factors, interps, True, 1,'vacuum',1]

        move_list.append(move)
        arg_list.append(args)

    objects = [pick_cube_obj] 
    states = ['Suppressed']
    move = 'SetObjectState'
    args = [objects, states]
    
    move_list.append(move)
    arg_list.append(args)

    preset = 'vacuum_place'
    move = 'SetRobotPreset'
    args = [robot, preset]

    move_list.append(move)
    arg_list.append(args)


# //pick---->place
    goal_types = ['pose', 'target', 'pose']
    if robot == 'UR5':
        goal_values = [[0, 0, 48, 0, 0, 0], cube_place, [0,0,-30,0,0,0]]    
    elif robot == 'RV5AS':
        goal_values = [[0, 0, 48, 0, 0, 0], cube_place, [0,0,-35,0,0,0]]
    elif robot == 'IRB1200':
        goal_values = [[0, 0, 48, 0, 0, 0], cube_place, [0,0,-35,0,0,0]]
    move_types = ['direct','roadmap', 'direct']
    rel_pose_flags = [True, False, True]
    smoothing_values = [10.0, 10.0, 10.0]
    col_check_flags = [False, True, False]
    speed_factors = [1.0, 1.0, 1.0]
    interps = ['j','j','j']
    act_device = False

    move = 'CombinedMove'
    args = [robot, goal_types, goal_values, move_types, rel_pose_flags, smoothing_values, col_check_flags, speed_factors, interps, False]

    move_list.append(move)
    arg_list.append(args)

    move = 'SetVacuumOff'
    args = robot
    move_list.append(move)
    arg_list.append(args)  

    objects = [placed_cube_obj] 
    states = ['Active']
    move = 'SetObjectState'
    args = [objects, states]

    move_list.append(move)
    arg_list.append(args)

    preset = 'vacuum_pick'
    
    move = 'SetRobotPreset'
    args = [robot, preset]

    move_list.append(move)
    arg_list.append(args)

    if cube_next == home:
    #    place---->home---movement
        goal_types = ['pose', 'target']
        if robot == 'UR5':
            goal_values = [[0,0,38,0,0,0], cube_next]    
        elif robot == 'RV5AS':
            goal_values = [[0,0,36,0,0,0], cube_next]
        elif robot == 'IRB1200':
            goal_values = [[0,0,32,0,0,0], cube_next]

        move_types = ['direct','roadmap']
        rel_pose_flags = [True, False]
        smoothing_values = [10.0, 10.0]
        col_check_flags = [False, True]
        speed_factors = [1.0, 1.0]
        interps = ['j','j']
        act_device = False

        move = 'CombinedMove'
        args = [robot, goal_types, goal_values, move_types, rel_pose_flags, smoothing_values, col_check_flags, speed_factors, interps, False]

        move_list.append(move)
        arg_list.append(args)

    else:
        # place--->pick 
        goal_types = ['pose','target', 'pose']
        if robot == 'UR5':
            goal_values = [[0,0,38,0,0,0], cube_next, [0, 0, -48, 0, 0, 0]]    
        elif robot == 'RV5AS':
            goal_values = [[0,0,36,0,0,0], cube_next, [0, 0, -48, 0, 0, 0]]
        elif robot == 'IRB1200':
            goal_values = [[0,0,32,0,0,0], cube_next, [0, 0, -48, 0, 0, 0]]

        move_types = ['direct','roadmap','direct']
        rel_pose_flags = [True, False, True]
        smoothing_values = [10.0, 10.0, 0]
        col_check_flags = [False, True, False]
        speed_factors = [1.0, 1.0, 1.0]
        interps = ['j', 'j', 'j']
        act_device = False        

        move = 'CombinedMove'
        args = [robot, goal_types, goal_values, move_types, rel_pose_flags, smoothing_values, col_check_flags, speed_factors, interps, True, 1, 'vacuum', 1]

        move_list.append(move)
        arg_list.append(args)

    return [move_list, arg_list]


def generate_put_tray_to_cube_task(robot, pick_dest, place_dest):

    cube_obj = []
    cube_pick = []
    cube_place = []
    placed_cube_obj = []
    pick_cube_obj = []
    if pick_dest not in range(0,4):
        print ("[RTR_CMD] Invalid cube position index")
        return -1

    if place_dest not in [1,2,3,4,5,6]:
        print ("[RTR_CMD] Invalid tray slot index")
        return -1

    if robot == 'UR5':
        color = 'yellow'
        home = 'ur_standby'
    elif robot == 'RV5AS':
        color = 'purple'
        home = 'melfa_standby'
    elif robot == 'IRB1200':
        color = 'green'
        home = 'abb_home'

    if pick_dest < 10:
        cube_pick =  str(color) + '_cube_place_0' + str(pick_dest) + '_approach'
    else:
        cube_pick =  str(color) + '_cube_place_' + str(pick_dest) + '_approach'
      
    pick_cube_obj = 'placed_' + str(color) + '_cube_0' + str(pick_dest)
    
    task_planner.abb_index = [1,2,3]
    task_planner.ur_index = [1,2,3]
    task_planner.melfa_index = [1,2,3]

    cube_next_index = -1
    cube_next = ''

    if robot == 'UR5': 
        if pick_dest == task_planner.ur_index[0]:
            cube_next_index = task_planner.ur_index[1]
        elif pick_dest == task_planner.ur_index[1]:
            cube_next_index = task_planner.ur_index[2]    
        elif pick_dest == task_planner.ur_index[2]:
            cube_next = 'ur_standby'

    if robot == 'IRB1200': 
        if pick_dest == task_planner.abb_index[0]:
            cube_next_index = task_planner.abb_index[1]
        elif pick_dest == task_planner.abb_index[1]:
            cube_next_index = task_planner.abb_index[2]    
        elif pick_dest == task_planner.abb_index[2]:
            cube_next = 'abb_home'
    
    if robot == 'RV5AS':
        if pick_dest == task_planner.melfa_index[0]:
            cube_next_index = task_planner.melfa_index[1]
        elif pick_dest == task_planner.melfa_index[1]:
            cube_next_index = task_planner.melfa_index[2]    
        elif pick_dest == task_planner.melfa_index[2]:
            cube_next = 'melfa_standby'  

    if cube_next_index != -1:
        if cube_next_index < 10:
            cube_next = str(color) + '_cube_place_0' + str(cube_next_index) + '_approach'
        else:
            cube_next = str(color) + '_cube_place_' + str(cube_next_index) + '_approach'      
    print(cube_next)

    if place_dest < 10:
        cube_place = str(color) + '_cube_0' + str(place_dest) + '_pick_approach'
        placed_cube_obj = str(color) + '_cubic_0' + str(place_dest)

    else:
        cube_place = str(color) + '_cube_' + str(place_dest) + '_pick_approach'
        placed_cube_obj = str(color) + '_cubic_' + str(place_dest)
     
    ###################################################################################
    


    move_list = []
    arg_list = []
    # start home--->pick
    if (robot == 'UR5' and pick_dest == task_planner.ur_index[0]) or (robot == 'IRB1200' and pick_dest == task_planner.abb_index[0]) or (robot == 'RV5AS' and pick_dest == task_planner.melfa_index[0]):

        goal_types = ['target', 'pose']
        if robot == 'UR5':

            goal_values = [cube_pick, [0, 0, -40, 0, 0, 0]]#### 
        elif robot == 'RV5AS':
            goal_values = [cube_pick, [0, 0, -46, 0, 0, 0]]   ####-          
        elif robot == 'IRB1200':
            goal_values = [cube_pick, [0, 0, -44, 0, 0, 0]]            
        move_types = ['roadmap', 'direct']
        rel_pose_flags = [False, True]
        smoothing_values = [10.0, 10.0]
        col_check_flags = [True, False]
        speed_factors = [1.0, 1.0]
        interps = ['j', 'j']
        act_device = True

        move = 'CombinedMove'
        args = [robot, goal_types, goal_values, move_types, rel_pose_flags, smoothing_values, col_check_flags, speed_factors, interps, True, 1,'vacuum',1]

        move_list.append(move)
        arg_list.append(args)

    objects = [pick_cube_obj] 
    states = ['Suppressed']
    move = 'SetObjectState'
    args = [objects, states]
    
    move_list.append(move)
    arg_list.append(args)

    preset = 'vacuum_place'
    move = 'SetRobotPreset'
    args = [robot, preset]

    move_list.append(move)
    arg_list.append(args)


# //pick---->place--->cube
    goal_types = ['pose', 'target', 'pose']
    if robot == 'UR5':
        goal_values = [[0, 0, 48, 0, 0, 0], cube_place, [0,0,-40,0,0,0]]    
    elif robot == 'RV5AS':
        goal_values = [[0, 0, 48, 0, 0, 0], cube_place, [0,0,-35,0,0,0]]
    elif robot == 'IRB1200':
        goal_values = [[0, 0, 45, 0, 0, 0], cube_place, [0,0,-30,0,0,0]]
    move_types = ['direct','roadmap', 'direct']
    rel_pose_flags = [True, False, True]
    smoothing_values = [10.0, 10.0, 10.0]
    col_check_flags = [False, True, False]
    speed_factors = [1.0, 1.0, 1.0]
    interps = ['j','j','j']
    act_device = False

    move = 'CombinedMove'
    args = [robot, goal_types, goal_values, move_types, rel_pose_flags, smoothing_values, col_check_flags, speed_factors, interps, False]

    move_list.append(move)
    arg_list.append(args)

    move = 'SetVacuumOff'
    args = robot
    move_list.append(move)
    arg_list.append(args)  

    objects = [placed_cube_obj] 
    states = ['ready']
    move = 'SetObjectState'
    args = [objects, states]

    move_list.append(move)
    arg_list.append(args)

    preset = 'vacuum_pick'
    
    move = 'SetRobotPreset'
    args = [robot, preset]

    move_list.append(move)
    arg_list.append(args)

    if cube_next == home:
    #    place---->home---movement
        goal_types = ['pose', 'target']
        if robot == 'UR5':
            goal_values = [[0,0,38,0,0,0], cube_next]    
        elif robot == 'RV5AS':
            goal_values = [[0,0,36,0,0,0], cube_next]
        elif robot == 'IRB1200':
            goal_values = [[0,0,40,0,0,0], cube_next]

        move_types = ['direct','roadmap']
        rel_pose_flags = [True, False]
        smoothing_values = [10.0, 10.0]
        col_check_flags = [False, True]
        speed_factors = [1.0, 1.0]
        interps = ['j','j']
        act_device = False

        move = 'CombinedMove'
        args = [robot, goal_types, goal_values, move_types, rel_pose_flags, smoothing_values, col_check_flags, speed_factors, interps, False]

        move_list.append(move)
        arg_list.append(args)

    else:
        # place--->pick 
        goal_types = ['pose','target', 'pose']
        if robot == 'UR5':
            goal_values = [[0,0,40,0,0,0], cube_next, [0, 0, -40, 0, 0, 0]]    
        elif robot == 'RV5AS':
            goal_values = [[0,0,40,0,0,0], cube_next, [0, 0, -46, 0, 0, 0]]
        elif robot == 'IRB1200':
            goal_values = [[0,0,32,0,0,0], cube_next, [0, 0, -44, 0, 0, 0]]

        move_types = ['direct','roadmap','direct']
        rel_pose_flags = [True, False, True]
        smoothing_values = [10.0, 10.0, 10.0]
        col_check_flags = [False, True, False]
        speed_factors = [1.0, 1.0, 1.0]
        interps = ['j', 'j', 'j']
        act_device = False        

        move = 'CombinedMove'
        args = [robot, goal_types, goal_values, move_types, rel_pose_flags, smoothing_values, col_check_flags, speed_factors, interps, True, 1, 'vacuum', 1]

        move_list.append(move)
        arg_list.append(args)

    return [move_list, arg_list]


#s7_plc = PLC('dummy')
s7_plc = PLC('10.164.2.200')
workcell_io= PPP_cell_IO(s7_plc)
fp = open('cell_task_log.txt','a')
task_planner = TaskPlanner("127.0.0.1", workcell_io, fp)

def main():
    ret = workcell_io.plc.is_connected()
    if ret == True:
        print('     connected...')  

    d_in = dict([('assy_tray', 0), ('tray_1',1), ('tray_2', 2), ('tray_3', 3), ('tray_4', 4), ('lane_1_sensor', 5), ('lane_2_sensor', 5), ('lane_2_place', 7)])

    print("n[RTR_CMD] Connecting to the Realtime Controller... \n")

    try:
        
        while(True):
            task_planner.io.set_all_devices_off()
            time.sleep(1)

            task_planner.SetSpeedFactor(0.3)

            start_time = time.time()

            ## Uncomment this section to set a tray to the assembly area

            ret = Initialize_Objects(task_planner)
            if ret != 0:
                return  

            ret = UR5_Set_Tray(task_planner, 1) ## currently only tray 1 position is in the project
            if ret != 0:
                return    
    
            handle_response(task_planner.cmd.set_interrupt_behavior('RV5AS', 1, 3))
            handle_response(task_planner.cmd.set_interrupt_behavior('ABB1200', 1, 3))
            handle_response(task_planner.cmd.set_interrupt_behavior('UR5', 1, 3))
            
            handle_response(task_planner.cmd.set_alternate_location('RV5AS', 1, 0, 'melfa_standby'))
            handle_response(task_planner.cmd.set_alternate_location('ABB1200', 1, 0, 'abb_standby')) 
            handle_response(task_planner.cmd.set_alternate_location('UR5', 1, 0, 'ur_vacuum_home')) 

        
            ## set cube index here: [1,2,3] means the cubes that the cube will be picked in the order cube position 1, 2, then 3 (1 - 12 can be used but only up to 6 is in the current project)
            #pack
            task_planner.SetABBCubePickIndex([1,2,3])
            task_planner.SetURCubePickIndex([1,2,3])
            task_planner.SetMelfaCubePickIndex([1,2,3])
            task_planner.start(True, True, True) #uncomment to move all robots: still testing in simulation and comment the upper section doing the pick and place in sequence
            

            #unpack
            task_planner.unpack(True,True,True,abb_put_back_index=[1,2,3],ur_put_back_index=[1,2,3],melfa_put_back_index=[1,2,3])

            ret=pick_tray_to_stroage(task_planner, 1)
            if ret != 0:
                return 
            end_time = time.time()
            time_s = end_time - start_time
            time_mm = round(time_s // 60)
            time_ss = round((time_s/60 - time_mm) * 60.0, 2)

            if time_ss < 10:
                task_planner.log(f'The task was finished in 00:0{time_mm}:0{time_ss}!')
            else:
                task_planner.log(f'The task was finished in 00:0{time_mm}:{time_ss}!')
    finally:
        fp.close()

def pick_tray_to_stroage(rtr, tray):


    robot_name='UR5'
    ret = task_planner.SetRobotPreset(robot_name, 'tray_pick')
    if ret != 0:
        return ret  
    robot = 'UR5'
    tray_obj = []
    tray_pick = []
    tray_pick_approach = []

    if tray not in [1,2,3,4]:
        print ("[RTR_CMD] Invalid tray input")
        return -1
    
    home = 'ur5_actuator_home'
    tray_pick_approach = 'tray_' + str(tray) + '_pick_approach'

    
    ret = rtr.SetRobotPreset('UR5', 'tray_pick')
    if ret != 0:
        return ret

    goal_types = ['target', 'pose']
    goal_values = [ 'tray_place_approach', [0,0,-35,0,0,0]]
    move_types = ['roadmap', 'direct']
    rel_pose_flags = [ False, True]
    smoothing_values = [ 10.0, 10.0]
    col_check_flags = [ True, False]
    speed_factors = [ 1.0, 1.0]
    interps = ['j','j']

    ret = rtr.CombinedMove(robot, goal_types, goal_values, move_types, rel_pose_flags, smoothing_values, col_check_flags, speed_factors, interps, actuate_device=False)
    if ret != 0:
        return ret

    ret = rtr.io.close_gripper()
    
    ret = task_planner.SetObjectStates(['tray05'], ['Suppressed'])
    if ret != 0:
        return ret

    ret = rtr.SetRobotPreset('UR5', 'tray_place')
    if ret != 0:
        return ret 

    p_tray_1_pick_up= [276.704, -402.431, 40,0.498,-178.78,-89.637]
    p_tray_1_pick= [276.704, -402.431, 30,0.498,-178.78,-89.637]

    goal_types = ['pose', 'target', 'pose', 'pose']
    goal_values = [[0, 0, 60, 0, 0, 0], tray_pick_approach, p_tray_1_pick_up,p_tray_1_pick]
    move_types = ['direct','roadmap', 'direct','direct']
    rel_pose_flags = [True, False, False,False]
    smoothing_values = [10.0, 10.0, 10.0,0]
    col_check_flags = [False, True, False,False]
    speed_factors = [1.0, 1.0, 1.0,1.0]
    interps = ['j','j','j','l']

    ret = rtr.CombinedMove(robot, goal_types, goal_values, move_types, rel_pose_flags, smoothing_values, col_check_flags, speed_factors, interps, actuate_device=False)
    if ret != 0:
        return ret

    ret = rtr.io.open_gripper()

    ret = rtr.SetRobotPreset('UR5', 'tray_pick')
    if ret != 0:
        return ret

    pick_tray_object = 'tray0' + str(tray)
    ret = rtr.SetObjectStates( [pick_tray_object],['ready'])
    if ret != 0:
        return ret


    
    goal_types = ['pose', 'target']
    goal_values = [[0, 0, 45, 0, 0, 0], 'ur5_actuator_home']
    move_types = ['direct','roadmap']
    rel_pose_flags = [True, False]
    smoothing_values = [10.0, 10.0]
    col_check_flags = [False, True]
    speed_factors = [1.0, 1.0]
    interps = ['j','j']
    ret = rtr.CombinedMove(robot, goal_types, goal_values, move_types, rel_pose_flags, smoothing_values, col_check_flags, speed_factors, interps, actuate_device=False)
    if ret != 0:
        return ret

    return 0


def pick_tray_from_conveyor():
        robot_name='UR5'
        ret = task_planner.SetRobotPreset(robot_name, 'tray_pick')
        if ret != 0:
            return ret         

        
        ret = task_planner.Move('UR5', 'target', 'pick_frm_convey_approach', 'roadmap')
        if ret != 0:
            return ret

        # pick_tray_frm_convey_tweak=[-3.662,-2008.929,23,180,0,91.28]    
        if task_planner.Move(robot_name, 'pose', [0,0,0,0,0,0], 'direct',relative_pose=True,blind_move=True) != 0:
            return -1        
        
        task_planner.io.close_gripper()
        
        ret = task_planner.SetObjectStates(['fulltray_on_front_conveyor_in'], ['Suppressed'])
        if ret != 0:
            return ret
        ret = task_planner.SetRobotPreset(robot_name, 'tray_place_with_cube')
        if ret != 0:
            return ret       
        
        if task_planner.Move(robot_name, 'pose', [0,0,0,0,0,0], 'direct',relative_pose=True,blind_move=True) != 0:
            return -1        
        
        ret = task_planner.Move('UR5', 'target', 'tray_place_approach', 'roadmap')
        if ret != 0:
            return ret
        #tray_place_tweak=[-3.662,-2008.929,23,180,0,91.28]    
        if task_planner.Move(robot_name, 'pose', [0,0,-35,0,0,0], 'direct',relative_pose=True,blind_move=True) != 0:
            return -1      
        
        task_planner.io.open_gripper()
        ret = task_planner.SetRobotPreset(robot_name, 'tray_pick')
        if ret != 0:
            return ret  
       

        objects = ['placed_yellow_cube_01','placed_yellow_cube_02','placed_yellow_cube_03',
               'placed_green_cube_01','placed_green_cube_02','placed_green_cube_03',
               'placed_purple_cube_01','placed_purple_cube_02','placed_purple_cube_03'
               ,'tray05'
              ]
        states = ['Active', 'Active', 'Active',
              'Active', 'Active', 'Active',
              'Active', 'Active', 'Active',
              'ready'
             ]

        ret = task_planner.SetObjectStates(objects, states)
        if ret != 0:
            return ret

        time.sleep(1)
        
        ret = task_planner.Move('UR5', 'pose', [0,0,70,0,0,0], 'direct',relative_pose=True,blind_move=True)
        if ret != 0:
            return ret   
        ret = task_planner.Move('UR5', 'target', 'ur5_actuator_home', 'roadmap')
        if ret != 0:
            return ret     
        ret = task_planner.SetRobotPreset(robot_name, 'vacuum_pick')
        if ret != 0:
            return ret                
        ret = task_planner.Move('UR5', 'target', 'ur_standby', 'roadmap')
        if ret != 0:
            return ret
         

           
        # 
        #     
        return 0



def pick_tray_to_conveyor():
        ret = task_planner.SetRobotPreset('UR5', 'tray_pick')
        
        if ret != 0:
            return ret
        
        ret = task_planner.Move('UR5', 'target', 'ur5_actuator_home', 'roadmap',smooth=20)
        if ret != 0:
            return ret

        ret = task_planner.Move('UR5', 'target', 'pickTrayToConveyorPt', 'roadmap')
        if ret != 0:
            return ret
        task_planner.io.close_gripper()
   

        ret = task_planner.SetObjectStates(['tray05'], ['Suppressed'])
        if ret != 0:
            return ret
        objects = ['placed_yellow_cube_01','placed_yellow_cube_02','placed_yellow_cube_03',
               'placed_green_cube_01','placed_green_cube_02','placed_green_cube_03',
               'placed_purple_cube_01','placed_purple_cube_02','placed_purple_cube_03'
              ]
        states = ['Suppressed', 'Suppressed', 'Suppressed',
              'Suppressed', 'Suppressed', 'Suppressed',
              'Suppressed', 'Suppressed', 'Suppressed',
             ]
        

        ret = task_planner.SetObjectStates(objects, states)
        if ret != 0:
            return ret

        time.sleep(1)
        ret = task_planner.SetRobotPreset('UR5', 'tray_place_with_cube')
        if ret != 0:
            return ret
        
        ret = task_planner.Move('UR5', 'pose', [0,0,70,0,0,0], 'direct',relative_pose=True,blind_move=True)
        if ret != 0:
            return ret    
        
        ret = task_planner.Move('UR5', 'target', 'placeToConveyor', 'roadmap',smooth=20)
        if ret != 0:
            return ret

        ret = task_planner.Move('UR5', 'pose', [0,0,-15,0,0,0], 'direct',relative_pose=True,blind_move=True)
        if ret != 0:
            return ret  
        
        task_planner.io.open_gripper()
        
        ret = task_planner.SetRobotPreset('UR5', 'tray_pick')
        if ret != 0:
            return ret

        ret = task_planner.SetObjectStates(['fulltray_on_front_conveyor'], ['ready'])
        if ret != 0:
            return ret

        time.sleep(1)
        
        ret = task_planner.Move('UR5', 'pose', [0,0,50,0,0,0], 'direct',relative_pose=True,blind_move=True)
        if ret != 0:
            return ret       
        
        ret = task_planner.Move('UR5', 'target', 'ur5_actuator_home', 'roadmap')
        if ret != 0:
            return ret
        ##trigger conveyor to move
        # 
        #     
        return 0


if __name__ == "__main__":
    main()        