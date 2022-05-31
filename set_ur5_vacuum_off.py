#!/usr/bin/env python3

from lib.PythonASCIICommander import PythonASCIICommander
from lib.S7_IO import PLC
from lib.S7_IO import PPP_cell_IO
import time

class RTR_TaskManager:

    def __init__(self, rtc_ip_address, plc_io):
        self.ip = rtc_ip_address
        self.io = plc_io
        self.cmd = PythonASCIICommander(self.ip, 9999, 1)
        self.speed_factor = 1.0

        self.target = self.cmd.MoveGoalType.TARGET
        self.pose = self.cmd.MoveGoalType.POSE

        self.cmd.connect()

    def waitForDelayedResponse(self, result: PythonASCIICommander.CommandResult):
        while result.delayed_response is not None:
            while not result.delayed_response.is_ready():
                time.sleep(0)
            result = result.delayed_response.get()
        return result.error_code

    def waitForCombinedMoveResponse(self, result: PythonASCIICommander.CommandResult, n_moves, move_n, robot = None, device = None, value = None):

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
                            self.io.set_vacuum_off(robot)
                        elif device == 'vacuum' and value == 1:
                            self.io.set_vacuum_on(robot)
                        elif device == 'gripper' and value == 0:
                            self.io.open_gripper()
                        elif device == 'gripper' and value == 1:
                            self.io.close_gripper()
                        elif device == 'conveyor' and value == 0:
                            self.io.stop_conveyor()
                        elif device == 'conveyor' and value == 1:
                            self.io.start_conveyor()
                    else:
                        time.sleep(0)
                n = n + 1
        print('\n')
        return result.error_code

    def SetSpeedFactor(self, speed):
        self.speed_factor = speed

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
        
        sum_error = 0
        for i in range(0, len(object_list)):
            res = self.cmd.set_object_state(object_list[i], object_state_list[i])
            code = res.error_code
            sum_error = sum_error + code

            if code != 0:
                print ('[Error] Setting the object: ' + str(object_list[i]) + ' to the state: ' + str(object_state_list[i]) + ' resulted in an error')
                print ('[RTR] Return Code: ' + str(code))
                return code
        
        print ("[RTR] SetObjectStates: ")
        print ("     " + str(object_list))
        print ("     Return Code: " + str(sum_error))
        return sum_error

    def SetRobotPreset(self, robot, preset):
        res = self.cmd.set_robot_preset(robot, preset)
        code = res.error_code

        print ("[RTR] SetRobotPreset (" + str(robot) + "): " + str(code))

        if code != 0:
            print ('[Error] Setting the robot: "' + str(robot) + '" to the preset: "' + str(preset) + '" resulted in an error')
            print ('[RTR] Return Code: ' + str(code))
            return code
        return 0     

    def Move(self, robot, goal_typ, goal_val, move_typ, blind_move = False, relative_pose = False, speed = 1.0):

        col_check = True
        if blind_move:
            col_check = False
        code = 0
        if goal_typ == 'pose' or goal_typ == 'POSE' or goal_typ == 'p' or goal_typ == 'P':
            res = self.cmd.move( robot_name = robot, speed = self.speed_factor * speed, goal_type=self.pose, 
            goal_value=goal_val, relative = relative_pose, smoothing=0.0, move_type=move_typ, interp='j', collision_check=col_check,collision_check_dsm=True)      

            code = self.waitForDelayedResponse(res)   
        
        elif goal_typ == 'target' or goal_typ == 'Target' or goal_typ == 'TARGET' or goal_typ == 't' or goal_typ == 'T':
            res = self.cmd.move( robot_name = robot, speed = self.speed_factor * speed, goal_type=self.target, 
            goal_value=goal_val, smoothing=0.0, move_type=move_typ, interp='j', collision_check=col_check, collision_check_dsm=True)      

            code = self.waitForDelayedResponse(res)   
        
        print ("[RTR_CMD] Move (" + str(goal_val) + ") : " + str(code))
        return code

    def CombinedMove(self, robot, goal_types, goal_values, move_types, rel_pose_flags, smoothing_values, col_check_flags, speed_factors, interps, actuate_device = False, actuation_timing = None, device = None, value = None):

        combined_moves = []

        for x in range(0, len(goal_types)):
            if goal_types[x] == 'pose' or goal_types[x] == 'POSE' or goal_types[x] == 'p' or goal_types[x] == 'P':
                g_type = self.pose
            elif goal_types[x] == 'target' or goal_types[x] == 'Target' or goal_types[x] == 'TARGET' or goal_types[x] == 't' or goal_types[x] == 'T':
                g_type = self.target

            move = self.cmd.generate_move_data_dictionary(
                robot_name=robot, speed = speed_factors[x], goal_type=g_type, goal_value=goal_values[x], move_type=move_types[x], relative=rel_pose_flags[x], smoothing=smoothing_values[x], collision_check=col_check_flags[x], interp=interps[x])
            combined_moves.append(move)

        res = self.cmd.combined_move(robot, combined_moves)

        if actuate_device==False:
            code = self.waitForCombinedMoveResponse(res, len(combined_moves), len(combined_moves)-1)
        else:
            code = self.waitForCombinedMoveResponse(res, len(combined_moves), actuation_timing, robot, device, value)
        print ("[RTR_CMD] CombinedMove : " + str(code))
        return code
              
def UR5_Pick_Tray(rtr, tray):

    tray_obj = []
    tray_pick = []
    tray_pick_approach = []

    if tray not in [1,2,3,4]:
        print ("[RTR_CMD] Invalid tray input")
        return -1
    
    if tray == 3:
        tray_obj = 'Tray4'
    elif tray == 4:
        tray_obj = 'Tray3'
    else:
        tray_obj = 'Tray' + str(tray)

    if tray == 1:
        tray_pick_approach = 'tray' + str(tray) + 'PickTargetUp02'
    else:
        tray_pick_approach = 'tray' + str(tray) + 'PickTargetUp01'

    if tray == 1:
        tray_pick = 'tray' + str(tray) + 'PickTarget'
    else:
        tray_pick = 'tray' + str(tray) + '_PickTarget'

    robot = 'UR5'

    objects = ['TrayInAssmeblyZone', tray_obj]
    states = ['Suppressed', 'Ready']

    ret = rtr.SetObjectStates(objects, states)
    if ret != 0:
        return ret

    ret = rtr.SetRobotPreset('UR5', 'actuator_Open')
    if ret != 0:
        return ret

    time.sleep(1)

    ret = rtr.Move(robot, 'target', 'ur5_actuator_home', 'planning')
    if ret != 0:
        return ret

    ret = rtr.Move(robot, 'target', tray_pick_approach, 'roadmap')
    if ret != 0:
        return ret

    ret = rtr.Move(robot, 'target', tray_pick, 'roadmap')
    if ret != 0:
        return ret

    ret = rtr.SetRobotPreset('UR5', 'actuator_Close')
    if ret != 0:
        return ret 

    objects = [tray_obj] 
    states = ['Suppressed']

    ret = rtr.SetObjectStates(objects, states)
    if ret != 0:
        return ret

    time.sleep(1)

    ret = rtr.Move(robot, 'target', 'place_tray_assyzone_up02', 'roadmap', blind_move=True)
    if ret != 0:
        return ret

    ret = rtr.Move(robot, 'pose', [0,0,-19,0,0,0], 'direct', relative_pose=True, blind_move=True) 
    if ret != 0:
        return ret

    ret = rtr.SetRobotPreset('UR5', 'actuator_Open')
    if ret != 0:
        return ret

    objects = ['TrayInAssmeblyZone']
    states = ['Ready']

    ret = rtr.SetObjectStates(objects, states)
    if ret != 0:
        return ret

    time.sleep(1)

    ret = rtr.Move(robot, 'pose', [0,0,49,0,0,0], 'direct', relative_pose=True, blind_move=True) 
    if ret != 0:
        return ret

    return 0

def UR5_Pick_Cube(rtr, peripheral_device, act_val):

    robot = 'UR5'
    ret = rtr.SetRobotPreset('UR5', 'Vaccum_Pick')
    if ret != 0:
        return ret

    goal_types = ['pose', 'pose', 'pose']
    goal_values = [[127.775 , -50.649, 201.619, 1.724, -179.921, -179.977 ], 
                   [128.327, 266.584, 94.3, 0, -180.0, -180.0 ],
                   [128.327, 266.584, 40, 0, -180.0, -180.0 ]]
    move_types = [0,0,0]
    rel_pose_flags = [False, False, False]
    smoothing_values = [10.0, 10.0, 10.0]
    col_check_flags = [True, True, False]
    speed_factors = [1.0, 1.0, 1.0]
    interps = ['j','j','j']
    act_device = True

    ret = rtr.CombinedMove(robot, goal_types, goal_values, move_types, rel_pose_flags, smoothing_values, col_check_flags, speed_factors, interps, actuate_device=act_device, actuation_timing=len(goal_values)-1,device=peripheral_device, value = act_val)
    if ret != 0:
        return ret

    return 0

def UR5_Place_Cube(rtr, peripheral_device, act_val):

    robot = 'UR5'
    ret = rtr.SetRobotPreset('UR5', 'Vaccum_Pick')
    if ret != 0:
        return ret

    goal_types = ['pose', 'pose', 'pose']
    goal_values = [[128.327, 266.584, 94.3, 0, -180.0, -180.0 ], 
                   [-126.966, 42.887, 140.403, 0, -180.0, -180.0],
                   [-126.966, 42.887, 46.43, 0, -180.0, -180.0 ]]
    move_types = [0,0,0]
    rel_pose_flags = [False, False, False]
    smoothing_values = [10.0, 10.0, 10.0]
    col_check_flags = [False, True, False]
    speed_factors = [1.0, 1.0, 1.0]
    interps = ['j','j','j']
    act_device = True

    ret = rtr.CombinedMove(robot, goal_types, goal_values, move_types, rel_pose_flags, smoothing_values, col_check_flags, speed_factors, interps, actuate_device=act_device, actuation_timing = len(goal_values), device=peripheral_device, value = act_val)
    if ret != 0:
        return ret

    return 0

def UR5_Go_Home_from_Place(rtr):

    robot = 'UR5'
    ret = rtr.SetRobotPreset('UR5', 'Vaccum_Pick')
    if ret != 0:
        return ret

    goal_types = ['pose', 'pose']
    goal_values = [[-126.966, 42.887, 140.403, 0, -180.0, -180.0 ], 
                   [127.775 , -50.649, 201.619, 1.724, -179.921, -179.977 ]]
    move_types = [0,0]
    rel_pose_flags = [False, False]
    smoothing_values = [10.0, 10.0]
    col_check_flags = [False, True]
    speed_factors = [1.0, 1.0]
    interps = ['j','j']

    ret = rtr.CombinedMove(robot, goal_types, goal_values, move_types, rel_pose_flags, smoothing_values, col_check_flags, speed_factors, interps)
    if ret != 0:
        return ret

    return 0

def UR5_Retrieve_Cube(rtr, peripheral_device, act_val):

    robot = 'UR5'
    ret = rtr.SetRobotPreset('UR5', 'Vaccum_Pick')
    if ret != 0:
        return ret

    goal_types = ['pose', 'pose']
    goal_values = [[-126.966, 42.887, 140.403, 0, -180.0, -180.0 ], 
                   [-126.966, 42.887, 40, 0, -180.0, -180.0 ]]
    move_types = [0,0]
    rel_pose_flags = [False, False]
    smoothing_values = [10.0, 10.0]
    col_check_flags = [True, False]
    speed_factors = [1.0, 1.0]
    interps = ['j','j']
    act_device = True

    ret = rtr.CombinedMove(robot, goal_types, goal_values, move_types, rel_pose_flags, smoothing_values, col_check_flags, speed_factors, interps, actuate_device=act_device, actuation_timing = len(goal_values) -1, device=peripheral_device, value = act_val)
    if ret != 0:
        return ret

    return 0

def UR5_Return_Cube(rtr, peripheral_device, act_val):

    robot = 'UR5'
    ret = rtr.SetRobotPreset('UR5', 'Vaccum_Pick')
    if ret != 0:
        return ret

    goal_types = ['pose', 'pose', 'pose']
    goal_values = [[-126.966, 42.887, 140.403, 0, -180.0, -180.0 ], 
                   [128.327, 266.584, 94.3, 0, -180.0, -180.0 ],
                   [128.327, 266.584, 40, 0, -180.0, -180.0 ]]
    move_types = [0,0,0]
    rel_pose_flags = [False, False, False]
    smoothing_values = [10.0, 10.0, 10.0]
    col_check_flags = [False, True, False]
    speed_factors = [1.0, 1.0, 1.0]
    interps = ['j','j','j']
    act_device = True

    ret = rtr.CombinedMove(robot, goal_types, goal_values, move_types, rel_pose_flags, smoothing_values, col_check_flags, speed_factors, interps, actuate_device=act_device, actuation_timing=len(goal_values), device=peripheral_device, value = act_val)
    if ret != 0:
        return ret

    return 0

def UR5_Go_Home_from_Return(rtr):

    robot = 'UR5'
    ret = rtr.SetRobotPreset('UR5', 'Vaccum_Pick')
    if ret != 0:
        return ret

    goal_types = ['pose', 'pose']
    goal_values = [[128.327, 266.584, 94.3, 0, -180.0, -180.0 ], 
                   [127.775 , -50.649, 201.619, 1.724, -179.921, -179.977 ]]
    move_types = [0,0]
    rel_pose_flags = [False, False]
    smoothing_values = [10.0, 10.0]
    col_check_flags = [False, True]
    speed_factors = [1.0, 1.0]
    interps = ['j','j']

    ret = rtr.CombinedMove(robot, goal_types, goal_values, move_types, rel_pose_flags, smoothing_values, col_check_flags, speed_factors, interps)
    if ret != 0:
        return ret

    return 0

def main():

    print ('Connecting to Siemens Simatic S7-1200 PLC...')
    s7_plc = PLC('10.164.2.200')
    #s7_plc = PLC('dummy')
    d_in = dict([('assy_tray', 0), ('tray_1',1), ('tray_2', 2), ('tray_3', 3), ('tray_4', 4), ('lane_1_sensor', 5), ('lane_2_sensor', 5), ('lane_2_place', 7)])
    workcell_io = PPP_cell_IO(s7_plc)
    ret = workcell_io.plc.is_connected()
    if ret == True:
        print('     connected...')  


    # if workcell_io.plc.read_input(0,d_in['lane_1_sensor']) != True:
    #    print("HelloWorld")
    #    return
    
    #workcell_io.close_gripper()
    workcell_io.open_gripper()
    #workcell_io.set_vacuum_on('RV5AS')
    workcell_io.set_vacuum_off('RV5AS')
    #workcell_io.set_vacuum_off('UR5')
    workcell_io.set_vacuum_off('UR5')
    #workcell_io.start_conveyor()
    #workcell_io.stop_conveyor()
    # workcell_io.stop_conveyor()
    workcell_io.set_vacuum_off('IRB1200')
    #workcell_io.set_vacuum_on('IRB1200')
    #workcell_io.set_vacuum_off('RV5AS')
    return
if __name__ == "__main__":
    main()