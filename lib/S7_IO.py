import snap7
from snap7 import util
import time

device = dict([('5AS_vac', 0), ('irb1200_vac',1), ('ur5_vac', 2), ('ur5_grp_1', 3), ('ur5_grp_2', 4), ('conveyor_start', 5), ('conveyor_direction', 6)])

class PLC:
    def __init__(self, ip_address):
        self.plc = snap7.client.Client()
        self.dummy = False

        if ip_address != 'dummy':
            self.plc.connect(ip_address, 0, 1)
        else:
            self.dummy = True
    
    def is_connected(self):
        if self.dummy:
            return True
        else:
            ret = self.plc.get_connected()
            return ret

    def write_output_bit(self, plc_byte, plc_bit, new_value):
        if self.dummy:
            return 0
        else:
            print('write output to DB')
            data = self.plc.read_area(snap7.types.Areas.DB, 1, plc_byte, 1)
            print(data)
            snap7.util.set_bool(data, plc_byte, plc_bit, new_value)
            ret = self.plc.write_area(snap7.types.Areas.DB, 1, plc_byte, data)
            if ret == None:
                return 0
            else:
                return ret
    
    def write_output_bits(self, plc_byte, plc_bits, new_value):
        if self.dummy:
            return 0
        else:
            print('write output to DB')
            data = self.plc.read_area(snap7.types.Areas.DB, 1, plc_byte, 1)
            for plc_bit in plc_bits:
                snap7.util.set_bool(data, plc_byte, plc_bit, new_value)       
            ret = self.plc.write_area(snap7.types.Areas.DB, 1, plc_byte, data)
            if ret == None:
                return 0
            else:
                return ret

    def read_input(self, plc_byte, plc_bit):
        if self.dummy:
            return True
        else:
            data = self.plc.read_area(snap7.types.Areas.DB, 2, plc_byte, 1)
            return snap7.util.get_bool(data,0,plc_bit)

class PPP_cell_IO:

    def __init__(self, PLC):
        self.plc = PLC 
    
    def set_vacuum_on(self, robot):
        ret = -1
        while ret != 0:
            if robot == 'RV5AS':
                ret = self.plc.write_output_bit(0,device['5AS_vac'], 1)
                # time.sleep(1)
            elif robot == 'IRB1200':
                ret = self.plc.write_output_bit(0,device['irb1200_vac'], 1)
                # time.sleep(1)
            elif robot == 'UR5':
                ret =self.plc.write_output_bit(0,device['ur5_vac'], 1)
                # time.sleep(1)
            else:
                print ('[Error] Invalid input: ' + str(robot))
                return -1
            time.sleep(0)
        #time.sleep(0.1)
        return 0

    def set_vacuum_off(self, robot):
        ret = -1
        while ret != 0:
            if robot == 'RV5AS':
                ret = self.plc.write_output_bit(0,device['5AS_vac'], 0)
            elif robot == 'IRB1200':
                ret = self.plc.write_output_bit(0,device['irb1200_vac'], 0)
            elif robot == 'UR5':
                ret = self.plc.write_output_bit(0,device['ur5_vac'], 0)
            else:
                print ('[Error] Invalid input')  
                return -1
            time.sleep(0)
        #time.sleep(0.1)
        return 0

    def close_gripper(self):
        ret = -1
        while ret != 0:
            ret = self.plc.write_output_bits(0,[device['ur5_grp_1'], device['ur5_grp_2']],1)
            time.sleep(0)
        #time.sleep(0.1)
        return 0

    def open_gripper(self):
        ret = -1
        while ret != 0:
            ret = self.plc.write_output_bits(0,[device['ur5_grp_1'], device['ur5_grp_2']],0)
            time.sleep(0)
        #time.sleep(0.1)
        return 0

    def start_conveyor(self):
        ret = -1
        while ret != 0:
            ret = self.plc.write_output_bit(0,device['conveyor_start'], 1)
            time.sleep(0)
        #time.sleep(0.1)
        return 0

    def stop_conveyor(self):
        ret = -1
        while ret != 0:
            ret = self.plc.write_output_bit(0,device['conveyor_start'], 0)
            time.sleep(0)
        #time.sleep(0.1)
        return 0

    def set_conveyor_direction(self, direction_flag):
        ret = -1
        while ret != 0:
            ret = self.plc.write_output_bit(0,device['conveyor_direction'], direction_flag)
            time.sleep(0)
        #time.sleep(0.1)
        return 0

    def set_all_devices_off(self):

        ret = -1
        while ret != 0:
            ret = self.plc.write_output_bits(0,[0,1,2,3,4,5,6,7],0)
            time.sleep(0)
        time.sleep(0.1)
        return 0

    def set_all_devices_on(self):
        ret = -1
        while ret != 0:
            ret = self.plc.write_output_bits(0,[0,1,2,3,4,5,6,7],1)
            time.sleep(0)
        time.sleep(0.1)
        return 0