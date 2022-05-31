#!/usr/bin/env python3

import requests

class ApiError(Exception):
    """An API Error Exception"""
    def __init__(self, status):
        self.status = status
    def __str__(self):
        return f'APIError: status={self.status}'

class PythonCommanderHelper(object):
    get_state = '/api/v2/appliance/state/'
    project = '/api/v2/projects/'
    robots = '/api/v2/projects/:project/robots'
    load_project = '/api/v2/projects/:project/load'
    unload_project = '/api/v2/projects/:project/unload'
    config_mode = '/api/v2/appliance/mode/config/'
    clear_faults = '/api/v2/appliance/clear_faults/'
    teleport_robot = '/api/v2/projects/:project/hubs/:hub/'
    
    def __init__(self,ip_adr):
        self.ip_adr = ip_adr

    def send_get_request(self,extension):
        '''
        This function sends a get request to the passed URL extension and returns the response in JSON form

        Parameters:
            extension (str): the suffix, after http://<ip_address>, of the URL
        
        Returns:
            resp.json: The REST API's response in JSON form
        '''
        url = f'http://{self.ip_adr}{extension}'
        resp = requests.get(url)
        print(f'\n[INFO] Sent Get request to {url}')

        if resp.status_code != 200:
            # This means something went wrong.
            raise ApiError(f'GET {url} {resp.status_code}')
        
        return resp.json()

    def send_put_request(self,extension):
        '''
        This function sends a put request to the passed URL extension

        Parameters:
            extension (str): the suffix, after http://<ip_address>, of the URL
        '''
        url = f'http://{self.ip_adr}{extension}'
        resp = requests.put(url)
        print(f'\n[INFO] Sent Put request to {url}')

        if resp.status_code != 200:
            # This means something went wrong.
            raise ApiError(f'PUT {url} {resp.status_code}')

    def get_control_panel_state(self):
        '''
        This function is called with no arguments and returns the current state of the control panel

        Returns:
            state (str): The current state of the control panel
        '''
        state_resp = self.send_get_request(self.get_state)
        state = state_resp['state']

        return state

    def get_project_info(self):
        '''
        This function is called with no arguments and returns a nested dictionary with all the group names. For each group it also returns if
        the group is loaded and the projects installed in that group.

        Returns:
            group_info (nested dict): dictionary of the form {'group name': {'loaded': bool, 'projects': [str]}}
        '''
        project_resp = self.send_get_request(self.project)
        project_info = {}
        for project in project_resp:
            project_name = project['name']
            robots = project['robots']
            robot_names = []
            for robot in robots:
                robot_names.append(robot['name'])

            project_info.update({'project_name':project_name, 'robots': robot_names})
        
        return project_info

    def get_installed_projects(self):
        '''
        This function is called with no arguments and returns a list of all projects installed on the Controller
            
        Returns:
            project_list (list of strings): A list of strings where each string is the name of a project installed on the Controller    
        '''
        projs = self.send_get_request(self.installed_proj)
        project_list = projs['projects']
        return project_list

    def put_load_project(self,group_name):
        '''
        This function is passed a group name and attempts to load that group in the Control Panel

        Parameters:
            group_name (str): name of the group to be loaded in the Control Panel    
        '''
        extension,place_holer = self.load_group.split(':')
        extension = extension + group_name + '/'
        self.send_put_request(extension)
    
    def put_unload_group(self,group_name):
        '''
        This function is passed a group name and attempts to unload that group from the Control Panel

        Parameters:
            group_name (str): name of the group to be unloaded from the Control Panel           
        '''
        extension,place_holer = self.unload_group.split(':')
        extension = extension + group_name + '/'
        self.send_put_request(extension)
    
    def put_config_mode(self):
        '''
        This function puts the controller in config mode            
        '''
        self.send_put_request(self.config_mode)
    
    def put_teleport_robot(self,project,hub):
        '''
        This function teleports a robot to a specified hub. NOTE: the controller must be in config mode to teleport a simulated robot

        Parameters:
            project (str): The name of the project to teleport
            hub (str): The name of the hub the robot will be teleported to            
        '''
        state = self.get_control_panel_state()
        if state != 'CONFIG':
            print('Control Panel must be in CONFIG mode to teleport robot!')
            return

        split_string = self.teleport_robot.split(':')
        prefix = split_string[0]
        project = '%s/hubs/'%(project)
        hub = '%s'%(hub)

        teleport_string = prefix + project + hub
        self.send_put_request(teleport_string)