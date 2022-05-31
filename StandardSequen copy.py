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
                else:
                    time.sleep(0)
            n = n + 1
    print('\n')
    return result.error_code

class TaskPlanner:

    def __init__(self, rtc_ip_address, fp):
        self.ip = rtc_ip_address
        self.cmd = PythonASCIICommander(self.ip, 9999)
        self.cmd_1 = PythonASCIICommander(self.ip, 9999)
        self.cmd_2 = PythonASCIICommander(self.ip, 9999)
        self.cmd_3 = PythonASCIICommander(self.ip, 9999)
        self.speed_factor = 1.0
        self.helper = PythonCommanderHelper(self.ip)
        project_info = self.helper.get_project_info()
        self.robots = project_info['robots']

        self.target = self.cmd.MoveGoalType.TARGET
        self.pose = self.cmd.MoveGoalType.POSE

        self.abb_index = [1,2,3]
        self.ur_index = [1,2,3]
        self.melfa_index = [6,5,4]

        self.init_logging(fp)
        self.cmd.connect()
        self.EnterOperationMode()
        #ret = self.Move('Robot1', 'target', 'ur5_home', 'planning')
        #if ret != 0:
        #    return ret

        #ret = self.Move('Robot2', 'target', 'mit_home', 'planning')
        #if ret != 0:
        #    return ret

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
            if move == 'Move':
                future = self.executor.submit(self.Move, args[0], args[1], args[2], args[3])
            self.threads[robot_idx] = future

    def start(self, abb_flag, ur_flag, melfa_flag):


        self.tasks=['Move','Move']
        self.args=[[['Robot1', 'target', 'Target4', 'planning'],['Robot1', 'target', 'Target4', 'planning']]
        ,[['Robot2', 'target', 'Target3', 'planning'],['Robot2', 'target', 'Target3', 'planning']]]
        self.robots=['Robot1','Robot2']


        self.executor = ThreadPoolExecutor(max_workers=4)
        self.threads = [None] * len(self.robots)
        self.pick_and_place = [True]*len(self.robots) # cycle time stop watch
        self.end_times = [None]*len(self.robots) # cycle time stop watch
        self.task_idxs = [0]*len(self.robots) # current index of the hub list
        self.interlocking = [False]*len(self.robots) # project idx of the robot that had to retreat

        self.start_time = time.time()

        self.advance_task(0)
        self.advance_task(1)
        #self.advance_task(2)

        while True in self.pick_and_place:
            for robot_idx in range(0,len(self.robots)):
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

    def Move(self, robot, goal_typ, goal_val, move_typ, blind_move = False, relative_pose = False, speed = 1.0):

        col_check = True
        if blind_move:
            col_check = False
        code = -1
        while code == -1:

            if goal_typ == 'pose' or goal_typ == 'POSE' or goal_typ == 'p' or goal_typ == 'P':
                if robot == self.robots[0]:
                    with lock:
                        res = self.cmd_1.move( robot_name = robot, speed = self.speed_factor * speed, goal_type=self.pose, 
                        goal_value=goal_val, relative = relative_pose, smoothing=0.0, move_type=move_typ, interp='j', collision_check=col_check,collision_check_dsm=True)
                    code = handle_response(res)

                elif robot == self.robots[1]:
                    with lock:
                        res = self.cmd_2.move( robot_name = robot, speed = self.speed_factor * speed, goal_type=self.pose, 
                        goal_value=goal_val, relative = relative_pose, smoothing=0.0, move_type=move_typ, interp='j', collision_check=col_check,collision_check_dsm=True)
                    code = handle_response(res)
                elif robot == self.robots[2]:
                    with lock:
                        res = self.cmd_3.move( robot_name = robot, speed = self.speed_factor * speed, goal_type=self.pose, 
                        goal_value=goal_val, relative = relative_pose, smoothing=0.0, move_type=move_typ, interp='j', collision_check=col_check,collision_check_dsm=True)
                    code = handle_response(res)
                        
            elif goal_typ == 'target' or goal_typ == 'Target' or goal_typ == 'TARGET' or goal_typ == 't' or goal_typ == 'T':
                if robot == self.robots[0]:
                    with lock:
                        res = self.cmd_1.move( robot_name = robot, speed = self.speed_factor * speed, goal_type=self.target, 
                        goal_value=goal_val, smoothing=0.0, move_type=move_typ, interp='j', collision_check=col_check, collision_check_dsm=True)
                    code = handle_response(res)
                elif robot == self.robots[1]:
                    with lock:
                        res = self.cmd_2.move( robot_name = robot, speed = self.speed_factor * speed, goal_type=self.target, 
                        goal_value=goal_val, smoothing=0.0, move_type=move_typ, interp='j', collision_check=col_check, collision_check_dsm=True)
                    code = handle_response(res)
                elif robot == self.robots[2]:
                    with lock:
                        res = self.cmd_3.move( robot_name = robot, speed = self.speed_factor * speed, goal_type=self.target, 
                        goal_value=goal_val, smoothing=0.0, move_type=move_typ, interp='j', collision_check=col_check, collision_check_dsm=True)
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
                    move = self.cmd_2.generate_move_data_dictionary(
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

        if actuate_device==False:
            code = waitForCombinedMoveResponse(res, len(combined_moves), len(combined_moves)-1)
        elif actuate_device==True:
            code = waitForCombinedMoveResponse(res, len(combined_moves), actuation_timing, robot, device, value)
        print ("[RTR_CMD] CombinedMove (" + str(robot) + ", " + str(goal_values) + "): " + str(code))
        #time.sleep(0.5)
        return code

def Initialize_Objects(rtr):
                          
    objects = ['TrayInAssmeblyZone', 'Tray1', 'Tray2', 'Tray3', 'Tray4', 
               'yellowCubicInTray01','yellowCubicInTray02','yellowCubicInTray03',
               'PurpleCubicInTray01','PurpleCubicInTray02','PurpleCubicInTray03',
               'GreenCubicInTray01','GreenCubicInTray02','GreenCubicInTray03',
               'yellowCubic01','yellowCubic02','yellowCubic03',
               'yellowCubic04','yellowCubic05','yellowCubic06',
               'purpleCubic01','purpleCubic02','purpleCubic03',
               'purpleCubic04','purpleCubic05','purpleCubic06',
               'greenCubic01','greenCubic02','greenCubic03',
               'greenCubic04','greenCubic05','greenCubic06']
    states = ['Suppressed', 'Ready', 'Ready', 'Ready', 'Ready',
              'Suppressed', 'Suppressed', 'Suppressed',
              'Suppressed', 'Suppressed', 'Suppressed',
              'Suppressed', 'Suppressed', 'Suppressed',
              'Ready', 'Ready', 'Ready',
              'Ready', 'Ready', 'Ready', 
              'Ready', 'Ready', 'Ready', 
              'Ready', 'Ready', 'Ready',
              'Ready', 'Ready', 'Ready', 
              'Ready', 'Ready', 'Ready']

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

    goal_types = ['target', 'target', 'target']
    goal_values = [home, tray_pick_approach, tray_pick]
    move_types = ['roadmap','roadmap', 'roadmap']
    rel_pose_flags = [False, False, False]
    smoothing_values = [10.0, 10.0, 10.0]
    col_check_flags = [True, True, True]
    speed_factors = [1.0, 1.0, 1.0]
    interps = ['j','j','j']
    act_device = True

    ret = rtr.CombinedMove(robot, goal_types, goal_values, move_types, rel_pose_flags, smoothing_values, col_check_flags, speed_factors, interps, actuate_device=False)
    if ret != 0:
        return ret

    ret = rtr.io.close_gripper()

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

    if cube not in range(1,13):
        print ("[RTR_CMD] Invalid cube position index")
        return -1

    if robot == 'UR5':
        color = 'yellow'
        home = 'ur5_vacuum_home'
    elif robot == 'RV5AS':
        color = 'purple'
        home = 'melfa_home'
    elif robot == 'IRB1200':
        color = 'green'
        home = 'abb_home'

    if dest not in [1,2,3]:
        print ("[RTR_CMD] Invalid tray slot index")
        return -1

    if cube < 10:
        cube_pick = str(color) + '_cube_0' + str(cube) + '_pick_approach'
    else:
        cube_pick = str(color) + '_cube_' + str(cube) + '_pick_approach'

    cube_next_index = -1
    cube_next = ''

    if robot == 'UR5': 
        if cube == task_planner.ur_index[0]:
            cube_next_index = task_planner.ur_index[1]
        elif cube == task_planner.ur_index[1]:
            cube_next_index = task_planner.ur_index[2]    
        elif cube == task_planner.ur_index[2] :
            cube_next = 'ur5_vacuum_home'

    if robot == 'IRB1200': 
        if cube == task_planner.abb_index[0]:
            cube_next_index = task_planner.abb_index[1]
        elif cube == task_planner.abb_index[1]:
            cube_next_index = task_planner.abb_index[2]    
        elif cube == task_planner.abb_index[2] :
            cube_next = 'abb_home'
    
    if robot == 'RV5AS':
        if cube == task_planner.melfa_index[0]:
            cube_next_index = task_planner.melfa_index[1]
        elif cube == task_planner.abb_index[1]:
            cube_next_index = task_planner.melfa_index[2]    
        elif cube == task_planner.melfa_index[2] :
            cube_next = 'melfa_home'  

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

    print (cube)
    print (task_planner.melfa_index[0])
    print ('#########################################')
    if (robot == 'UR5' and cube == task_planner.ur_index[0]) or (robot == 'IRB1200' and cube == task_planner.abb_index[0]) or (robot == 'RV5AS' and cube == task_planner.melfa_index[0]):

        goal_types = ['target', 'pose']
        if robot == 'UR5':
            goal_values = [cube_pick, [0, 0, -38, 0, 0, 0]]
        elif robot == 'RV5AS':
            goal_values = [cube_pick, [0, 0, -43, 0, 0, 0]]                  
        elif robot == 'IRB1200':
            goal_values = [cube_pick, [0, 0, -45, 0, 0, 0]]            
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

    preset = 'vacuum_place'

    move = 'SetRobotPreset'
    args = [robot, preset]

    move_list.append(move)
    arg_list.append(args)


    goal_types = ['pose', 'target', 'pose']
    if robot == 'UR5':
        goal_values = [[0, 0, 38, 0, 0, 0], cube_place, [0,0,-38,0,0,0]]    
    elif robot == 'RV5AS':
        goal_values = [[0, 0, 43, 0, 0, 0], cube_place, [0,0,-36,0,0,0]]
    elif robot == 'IRB1200':
        goal_values = [[0, 0, 45, 0, 0, 0], cube_place, [0,0,-32,0,0,0]]
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

    preset = 'vacuum_pick'
    
    move = 'SetRobotPreset'
    args = [robot, preset]

    move_list.append(move)
    arg_list.append(args)

    if cube_next == home:

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
        goal_types = ['pose','target', 'pose']
        if robot == 'UR5':
            goal_values = [[0,0,38,0,0,0], cube_next, [0, 0, -38, 0, 0, 0]]    
        elif robot == 'RV5AS':
            goal_values = [[0,0,36,0,0,0], cube_next, [0, 0, -43, 0, 0, 0]]
        elif robot == 'IRB1200':
            goal_values = [[0,0,32,0,0,0], cube_next, [0, 0, -50, 0, 0, 0]]

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
    
##Start
def Robot2Prog(rtr,robot_name):
   
#    initilized object state

    objects = ["fulltray_on_end_left","fulltray_on_end_right"]

    states = ['Suppressed', 'ready']

    ret = rtr.SetObjectStates(objects, states)
    if ret != 0:
        return ret
    ret = rtr.SetRobotPreset('IRB1300', 'vaccum_pick')
    if ret != 0:
        return ret
    ret = rtr.Move(robot_name, 'target', 'irb1300_home', 'planning')
    if ret != 0:
        return ret
    ret = rtr.Move(robot_name, 'target', 'pick_full_tray_irb1300', 'roadmap')
    if ret != 0:
        return ret
    ret = rtr.SetObjectStates(['fulltray_on_end_right'],['Suppressed'])
    if ret != 0:
        return ret
    ret = rtr.SetRobotPreset('IRB1300', 'vaccum_place')
    if ret != 0:
        return ret
    ret = rtr.Move(robot_name, 'target', 'place_full_tray_irb1300', 'roadmap')
    if ret != 0:
        return ret    
    ret = rtr.SetRobotPreset('IRB1300', 'vaccum_pick')
    if ret != 0:
        return ret    
    ret = rtr.SetObjectStates(['fulltray_on_end_left'],['ready'])
    if ret != 0:
        return ret    
    ret = rtr.Move(robot_name, 'target', 'irb1300_home', 'roadmap')
    if ret != 0:
        return ret
    print('ProjectDone:===='+robot_name)

fp = open('cell_task_log.txt','a')
task_planner = TaskPlanner("127.0.0.1", fp)

 
def main():
    try:   
        task_planner.SetSpeedFactor(1.0)
        start_time = time.time()
        handle_response(task_planner.cmd.set_interrupt_behavior('Robot1', 1, 3))
        # handle_response(task_planner.cmd.set_interrupt_behavior('Robot2', 1, 3))
        while True:
             thread_1=threading.Thread(target=Robot2Prog,args=(task_planner,'IRB1300'))
            #  thread_2=threading.Thread(target=Robot2Prog)    
             thread_1.start()
            #  thread_2.start()
            #  while thread_1.is_alive() and thread_2.is_alive():                                                                                                                                                                                                                                                                                                                                                      
                #   pass                                                                                                                                                             
             thread_1.join()                                                                                                                              
            #  thread_2.join()                                                                                                                                                                                                                                                   
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

if __name__ == "__main__":
    main()        
