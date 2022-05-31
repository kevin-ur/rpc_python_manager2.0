import concurrent.futures
import dataclasses
import queue
import select
import socket
from enum import Enum
from typing import Any, Dict, List, NamedTuple, Optional, Tuple, Union
import uuid

import yaml
from lib.rtr_utils.rtr_logging import rtr_error
from lib.rtr_utils.rtr_timer import SimpleTimer

''' A Appliance Commander through ASCII API

    Design Doc: https://realtimerobotics.atlassian.net/wiki/spaces/DD/pages/2189393981/Python+ASCII+Commander+for+2.0
'''


# TODO(RAPID-11553)
class Result(NamedTuple):
    ''' Return type that allow value or error. Similar to c++ expected or Rust result (our checked)

    This is a quick and easy implementation, for output of either value or error. Thus None can
    not be used as return value or error value.
    '''
    value: Optional[Any] = None
    error: Optional[str] = None


class ASCIISocket():
    ''' Wrapper class for using TCP sockets and it's "streaming" characteristic.

    TCP's "streaming" property means segments could be concatenated or split. This class is a
    wrapper around a TCP socket to handle situation like this. The technic used is to split
    TCP segments using per-determented delimiter character.
    '''

    _RECV_BUFFER_SIZE = 65536  # 2^16 bytes

    def __init__(self,
                 new_socket: socket.socket,
                 delimiter: str,
                 encoding: str,
                 recv_buffer_size: int = _RECV_BUFFER_SIZE) -> None:
        self._socket = new_socket
        self._delimiter = delimiter
        self._encoding = encoding
        self._receive_buffer = ""
        self._recv_buffer_size = recv_buffer_size

    def get_residual_string(self) -> str:
        ''' Get residual string on the class internal buffer.
        '''
        return self._receive_buffer

    def clear_residual_string(self) -> None:
        ''' Clean up any residual string on the class's receiving buffer.

        This do not clear the OS's socket buffer, just the class internal buffer.
        '''
        self._receive_buffer = ""

    def send_all(self, data_str: str) -> Result:
        ''' Try to send all the data through the underlying TCP socket.
        '''
        try:
            self._socket.sendall(data_str.encode(self._encoding))
        except OSError as e:
            return Result(error=f"SocketError, send error [{e}]")
        return Result(value=True)

    def receive_one_response(self, timeout_s: float) -> Result:
        ''' Help handle socket receive

        Will keep receiving until a full response (end with \r\n) received or reached timeout

        Return:
            CheckedResponse: received respons or a error string
        '''
        t1 = SimpleTimer()
        while True:
            if self._delimiter in self._receive_buffer:
                remaining_timeout = 0
                # Not breaking to ensure system socket buffer is cleared.
            else:
                # prevent timeout < 0 being fed to socket receive.
                remaining_timeout = max(0, t1.remaining_time(timeout_s))

            recv_list, _, _ = select.select([self._socket], [], [], remaining_timeout)
            if self._socket not in recv_list:
                break
            try:
                # Select told use we can have something on socket now.
                response_byte = self._socket.recv(self._recv_buffer_size)
                if len(response_byte) == 0:
                    rtr_error("Socket closed by other side")
                    break
            except OSError as e:
                return Result(error=f"Error on socket recv [{e}]")

            try:
                self._receive_buffer += response_byte.decode(self._encoding)
            except UnicodeDecodeError as e:
                return Result(error=f"decoding error [{e}]; raw responses: [{response_byte}]")
            # Don't bother receiving anymore if already had a full response.

        if self._delimiter in self._receive_buffer:
            # There is a valid response:
            [first_segment, self._receive_buffer] = self._receive_buffer.split(self._delimiter, 1)
            return Result(value=first_segment)

        # received something but not a full response
        return Result(error=f"Did not get a full response segment within [{timeout_s}s]")


class PythonASCIICommander:
    ''' Class to help command appliance through ASCII API.

    The ASCII command function in this class should conform to appliance's ASCII API

    # TODO change link to real public ASCII API document when it exists.
    https://realtimerobotics.atlassian.net/wiki/spaces/PM/pages/2135818336/ASCII+API+2.0.0+Beta+YAML+Format

    '''
    _DEFAULT_SOCKET_POOL_SIZE = 5
    _MIN_SOCKET_POOL_SIZE = 1
    _MAX_SOCKET_POOL_SIZE = 1000
    # Immediate response could take a while, specially connecting to many robots
    _IMMEDIATE_RESPONSE_TIMEOUT_S = 60
    # This timeout will be part of the socket's attribute to make it non blocking
    _SOCKET_DEFAULT_TIMEOUT_S = 0.5
    _DEFAULT_DELAY_TIMEOUT_S = 5 * 60  # default 5 min delayed response

    class RequestOptArgs(NamedTuple):
        ''' This is used to capture the common optional argument for all commands.

            Caller could optionally choose to construct this object and pass to any command function, the
            content of the corresponding key will be filled in just before serializing to YAML string.
        '''
        id: Optional[str] = None
        type: Optional[str] = None

    class DelayedResponse:
        ''' c++ future like object specifically designed for ASCII commander to use.
        '''

        def __init__(self, future: concurrent.futures.Future, sent_string: str) -> None:
            self._future: concurrent.futures.Future = future
            self._sent_string: str = sent_string

        def is_ready(self) -> bool:
            return self._future.done()

        def get(self, timeout_s: Optional[float] = None) -> 'PythonASCIICommander.CommandResult':
            ''' Get the delayed response
            '''
            # timeout=0: no wait, timeout=None: blocking forever.
            try:
                return self._future.result(timeout=timeout_s)
            except concurrent.futures.TimeoutError as e:
                return PythonASCIICommander._generate_error_return(
                    f"TimeoutError: Timeout getting output from future: {e}",
                    send_str=self._sent_string)
            except Exception as e:
                return PythonASCIICommander._generate_error_return(
                    f"PythonException: Unexpected exception from concurrent thread: {e}",
                    send_str=self._sent_string)

    @dataclasses.dataclass()
    class CommandResult():
        ''' Hold all info about a result received from a command.

        dataclass For holding received information, and also some helper function to do simple
        parsing of the received data.
        '''

        @dataclasses.dataclass()
        class DebugInfo():
            ''' simple dataclass to hold debug information.
            '''
            sent_string: Optional[str] = None
            received_string: Optional[str] = None
            residual_string: Optional[str] = None

        response_dict: Optional[Dict[str, Any]] = None
        response_tuple: Optional[Tuple[Any]] = None
        delayed_response: Optional[
            'PythonASCIICommander.DelayedResponse'] = None  # forward reference
        debug_info: DebugInfo = dataclasses.field(default_factory=DebugInfo)
        commander_error: Optional[str] = None

        # In case of CSV, let constructor of the response to hint weather the last item would be non-seq number.
        _no_seq_hint = False

        def set_no_seq_hint(self) -> None:
            ''' After this is called, in csv case will always return no seq number
            '''
            self._no_seq_hint = True

        def is_csv(self) -> bool:
            ''' Check if this is a csv response by checking if response tuple is not None
            '''
            return self.response_tuple is not None

        @property
        def error_code(self) -> int:
            ''' Gets the old fashion return code with 0 for non-error, -1 for commander error,
            and other code from appliance

            Returns: int
            '''
            if self.commander_error is not None:
                return -1
            if self.is_csv():
                return self.response_tuple[1]
            if "error" in self.response_dict:
                return self.response_dict["error"]["code"]
            # Final case of YAML format and no error
            return 0

        def has_error(self) -> bool:
            ''' Check if the command gives any error.'''
            if self.error_code == 0:
                return False
            return True

        def is_feedback(self) -> bool:
            return (True if self.return_type == "Feedback" else False)

        @property
        def return_type(self) -> Optional[str]:
            if self.is_csv():
                # A helper list for CSV to findout command types
                possible_type_list = ["Command", "Response", "DelayedResponse", "Feedback"]
                for type_str in possible_type_list:
                    if type_str in self.response_tuple[0]:
                        return type_str
                return None

            return self.response_dict["type"]

        @property
        def seq_num(self) -> Optional[int]:
            ''' Try to get the sequence number if it exists

            Return: Optional[int] - None if no sequence number.
            '''
            if self.has_error():
                return None
            if self.is_csv():
                if (len(self.response_tuple) > 2) and (not self._no_seq_hint):
                    try:
                        return int(self.response_tuple[-1])
                    except ValueError:
                        return None
                return None
            return self.response_dict["data"].get("seq", None)

        @property
        def error_msg(self) -> Optional[str]:
            ''' Try to get error message. (will use error code for case of csv)
            '''
            if self.commander_error is not None:
                return self.commander_error
            if not self.has_error():
                return None
            if self.is_csv():
                return f"Error code {self.error_code}"
            return self.response_dict["error"]["msg"]

        # END of CommandResult Dataclass

    def __init__(self,
                 ip: str,
                 port: int,
                 socket_pool_size: int = _DEFAULT_SOCKET_POOL_SIZE,
                 set_yaml_resp: bool = True,
                 auto_id_base: Optional[str] = None):
        ''' Initialize the commander with socket pool and thread executor

        Args:
            ip : string of ip address of appliance
            port : default port number for appliance.
            socket_pool_size: pool size for sockets, determined the limit of concurrent socket calls
            set_yaml_resp = True: Set all sockets to use yaml response format or not.
            auto_id_base: str - base string for generating auto ID. (will add a 8 int uuid after words)
        Raises:
            OSError when it socket can not be created.
            ValueError: When socket pool size is too large or too small
        '''
        # Make sure user doesn't give too bad a socket pool size
        if socket_pool_size < self._MIN_SOCKET_POOL_SIZE:
            raise ValueError("Socket pool size too small")

        if socket_pool_size > self._MAX_SOCKET_POOL_SIZE:
            raise ValueError("Socket pool size too large")

        # Make the socket pool.
        self._socket_pool: queue.Queue[ASCIISocket] = queue.Queue(maxsize=socket_pool_size)
        self._delay_executor = concurrent.futures.ThreadPoolExecutor(
            max_workers=socket_pool_size, thread_name_prefix="Delay_Thread")

        self.auto_id_base = auto_id_base

        appliance_addr = (ip, port)
        for _ in range(socket_pool_size):
            try:
                new_socket = socket.create_connection(
                    appliance_addr, PythonASCIICommander._SOCKET_DEFAULT_TIMEOUT_S)
            except OSError as e:
                raise OSError(f"Fail to create ASCII API socket, with error {e}")
            else:
                ascii_socket = ASCIISocket(new_socket, delimiter='\r\n', encoding='ascii')
                self._socket_pool.put(ascii_socket)
                # TODO(LEO) Set response mode to YAML right after socket construction

    def _send_and_receive(self,
                          request_dict: Dict[str, Any],
                          delayed_timeout_s: Optional[float] = None,
                          request_opt_args: Optional[RequestOptArgs] = None,
                          forever_receiving=False) -> CommandResult:
        ''' Send the request, receive the response, parse it and return outcome.

        Will construct the request_dict to flow style YAML. Get a socket from socket pool and send it.
        Any exception and error will be catch and returned with the same response_dict format.
        If there will be a delayed response, a thread will be launched to catch it.
        Generally this function should not be directly used, other ASCII Command function should be used.

        Args:
            request_dict: this dict will be feed to PyYaml to get the request string and sent through socket
            delayed_timeout: set a timeout waitting for delayed response (if we ever expects one).
                This is also used to hint weather we should get sequence number! make sure it
                is left empty on command without delayed response
            response_opt_args: common keys that can be added to any command (id and sent_type_key)
        Return:
            CommandResult
        '''
        # Get a socket from socket pool
        try:
            cmd_socket: ASCIISocket = self._socket_pool.get_nowait()
        except queue.Empty as e:
            return self._generate_error_return(f"SocketError, Empty socket pool [{e}]")

        if self.auto_id_base is not None:
            request_dict['id'] = f"{self.auto_id_base}_{str(uuid.uuid4())[0:8]}"
        if request_opt_args is not None:
            if request_opt_args.type is not None:
                request_dict['type'] = request_opt_args.type
            if request_opt_args.id is not None:
                request_dict['id'] = request_opt_args.id

        # Try to send request
        request_str = yaml.dump(request_dict,
                                default_flow_style=True,
                                line_break="\r\n",
                                width=float("Infinity"))

        send_result = cmd_socket.send_all(request_str)

        command_result = None
        if send_result.error is not None:
            command_result = self._generate_error_return(send_result.error, send_str=request_str)
        else:
            command_result = self._process_one_response(cmd_socket,
                                                        self._IMMEDIATE_RESPONSE_TIMEOUT_S,
                                                        request_str)

        # Make sure to pass down this hint.
        if delayed_timeout_s is None:
            command_result.set_no_seq_hint()

        # have seq mean there is a delayed response.
        if (command_result.seq_num is not None):
            delay_timeout = delayed_timeout_s
            if delay_timeout is None:
                delay_timeout = self._DEFAULT_DELAY_TIMEOUT_S
                rtr_error(
                    "No delayed timeout is given, but this response does have sequence number! Check the Command function!"
                )
            future = self._delay_executor.submit(self._handle_delay_response,
                                                 cmd_socket,
                                                 delay_timeout,
                                                 request_str,
                                                 forever_receiving=forever_receiving)
            command_result.delayed_response = self.DelayedResponse(future, request_str)
            return command_result

        # No delayed response, could return socket now
        self._socket_pool.put(cmd_socket)
        return command_result

    @classmethod
    def _generate_error_return(cls,
                               err_str: str,
                               send_str: Optional[str] = None,
                               recv_str: Optional[str] = None) -> CommandResult:
        ''' Helper to generate error response. This is more of a static helper function.

        Args:
            err_cmd: str. The string for the command key.
            send_str
            recv_str
        '''
        debug_info = cls.CommandResult.DebugInfo()
        debug_info.sent_string = send_str
        debug_info.received_string = recv_str
        return cls.CommandResult(commander_error=err_str, debug_info=debug_info)

    def _handle_delay_response(self,
                               cmd_socket: ASCIISocket,
                               delayed_timeout_s: float,
                               sent_str: str,
                               forever_receiving: bool = False) -> CommandResult:
        ''' The function get thrown into thread to wait for delayed response.
            Same return format as other command functions

        Potentionally upgrade to a recursive function when multiple delay response is implimented in appliance.
        '''

        command_result = self._process_one_response(cmd_socket, delayed_timeout_s, sent_str)

        # TODO Quick hack for Rapid Opti to receive delayed response forever.
        if (not command_result.has_error()) and forever_receiving:
            future = self._delay_executor.submit(self._handle_delay_response,
                                                 cmd_socket,
                                                 delayed_timeout_s,
                                                 sent_str,
                                                 forever_receiving=forever_receiving)
            delayed_response = self.DelayedResponse(future, sent_str)
            command_result.delayed_response = delayed_response
            return command_result

        # hack above should be merged into this when the appliance side is fixed
        # TODO remove this line when combine move is fully tested!
        if command_result.is_feedback():  # feed back type mean we have more to come
            future = self._delay_executor.submit(self._handle_delay_response, cmd_socket,
                                                 delayed_timeout_s, sent_str)
            delayed_response = self.DelayedResponse(future, sent_str)
            command_result.delayed_response = delayed_response
            return command_result

        command_result.debug_info.residual_string = cmd_socket.get_residual_string()
        if command_result.debug_info.residual_string:
            # Nothing should be received after delayed response.
            rtr_error(
                f"Socket received more stuff after delayed response ! :[{command_result.debug_info.residual_string}]"
            )
            rtr_error("!!! This should not happen !! ")
            cmd_socket.clear_residual_string()

        self._socket_pool.put(cmd_socket)
        return command_result

    def _process_one_response(self,
                              cmd_socket: ASCIISocket,
                              recv_timeout_s: float,
                              sent_str: str = "") -> CommandResult:
        ''' Receive and parse one response.

        '''
        received_data: Result = cmd_socket.receive_one_response(recv_timeout_s)

        if received_data.error is not None:
            return self._generate_error_return(
                f"SocketError, delayed response {received_data.error}", send_str=sent_str)
        else:
            command_result = self._parse_response(received_data.value)
            command_result.debug_info = self.CommandResult.DebugInfo(
                sent_string=sent_str, received_string=received_data.value)
            return command_result

    # TODO deprecate this function and have receiving logic directly call under laying functions
    def _parse_response(self, resp: str) -> CommandResult:
        ''' Try to parse the response from appliance.

        Returns a CommandResult.
        In case of other error (correputed string), will still return the correct type with error code -1.
        For yaml, detect the topic key to decide if string is corrupted.
        '''
        # check for yaml or csv
        if resp[0] == '{':
            return self.CommandResult(response_dict=self._parse_response_yaml(resp))
        else:
            return self.CommandResult(response_tuple=tuple(self._parse_response_csv(resp)))

    def _parse_response_yaml(self, yaml_resp: str) -> Dict[str, Any]:
        ''' Parse a yaml response string into dictionary.

        Output will be returned using the same ASCII API dictionary format
        '''
        try:
            response_dict = yaml.safe_load(yaml_resp)
            return response_dict
        except yaml.YAMLError as e:
            return {
                "topic": "CommanderError",
                "error": {
                    "code": -1,
                    "msg": f"Corrupted Yaml String: [{e}]"
                }
            }

    def _parse_response_csv(self, csv_resp: str) -> List[Any]:
        try:
            tokens = csv_resp.split(',')
            tokens[1] = int(tokens[1])
            return tokens
        except IndexError:
            return ['CommanderError', -1]

##################################
########
######## Testing function
########
##################################

    def send_raw_yaml(self,
                      raw_yaml: Dict[str, Any],
                      request_opt_args: RequestOptArgs = RequestOptArgs(),
                      delay_timeout: Optional[float] = None) -> CommandResult:
        '''This method sends whatever dictionary you pass to it to the appliance. Use this method
        when you require greater flexibility for testing needs than the ASCII Command functions
        allow.
        '''
        return self._send_and_receive(raw_yaml,
                                      delayed_timeout_s=delay_timeout,
                                      request_opt_args=request_opt_args)


##################################
########
######## ASCII Command function
########
##################################

    def clear_faults(self, request_opt_args: RequestOptArgs = RequestOptArgs()) -> CommandResult:
        return self._send_and_receive({"topic": "ClearFaults"}, request_opt_args=request_opt_args)

    def get_mode(self, request_opt_args: RequestOptArgs = RequestOptArgs()) -> CommandResult:
        return self._send_and_receive({"topic": "GetMode"}, request_opt_args=request_opt_args)

    def load_project(self,
                     project_name: str,
                     request_opt_args: RequestOptArgs = RequestOptArgs(),
                     delay_timeout: float = _DEFAULT_DELAY_TIMEOUT_S) -> CommandResult:
        cmd = {"topic": "LoadProject", "data": {"project_name": project_name}}
        return self._send_and_receive(cmd,
                                      delayed_timeout_s=delay_timeout,
                                      request_opt_args=request_opt_args)

    def unload_project(self, request_opt_args: RequestOptArgs = RequestOptArgs()) -> CommandResult:
        cmd = {"topic": "UnloadProject"}
        return self._send_and_receive(cmd, request_opt_args=request_opt_args)

    def connect(
        self,
        workcell_preset: Optional[str] = None,
        robot_name: Optional[str] = None,
        request_opt_args: RequestOptArgs = RequestOptArgs()
    ) -> CommandResult:
        cmd: Dict[str, Any] = {"topic": "Connect", "data": {}}
        if workcell_preset is not None:
            cmd["data"]["workcell_preset"] = workcell_preset
        if robot_name is not None:
            cmd["data"]["robot_name"] = robot_name
        return self._send_and_receive(cmd, request_opt_args=request_opt_args)

    def disconnect(
        self,
        robot_name: Optional[str] = None,
        request_opt_args: RequestOptArgs = RequestOptArgs()
    ) -> CommandResult:
        '''This command disconnects from all robots in the project (unless a robot is specified)
        and returns them to the Initialized status.  Disconnect can only be called successfully
        when in CONFIG mode.

        This command is useful for saving time by keeping the project loaded on the controller
        while allowing the robots to perform tasks independent of Realtime control like rebooting.
        '''

        cmd: Dict[str, Any] = {"topic": "Disconnect", "data": {}}
        if robot_name is not None:
            cmd["data"]["robot_name"] = robot_name
        return self._send_and_receive(cmd, request_opt_args=request_opt_args)

    def enter_operation_mode(
            self, request_opt_args: RequestOptArgs = RequestOptArgs()) -> CommandResult:
        return self._send_and_receive({"topic": "EnterOperationMode"},
                                      request_opt_args=request_opt_args)

    def enter_configuration_mode(
            self, request_opt_args: RequestOptArgs = RequestOptArgs()) -> CommandResult:
        return self._send_and_receive({"topic": 'EnterConfigurationMode'},
                                      request_opt_args=request_opt_args)

    def release_control(
        self,
        robot_name: str,
        preset_name: Optional[str] = None,
        request_opt_args: RequestOptArgs = RequestOptArgs()
    ) -> CommandResult:
        cmd: Dict[str, Any] = {"topic": "ReleaseControl", "data": {"robot_name": robot_name}}
        if preset_name is not None:
            cmd["data"]["preset_name"] = preset_name
        return self._send_and_receive(cmd, request_opt_args=request_opt_args)

    def acquire_control(
        self,
        robot_name: str,
        preset_name: Optional[str] = None,
        request_opt_args: RequestOptArgs = RequestOptArgs()
    ) -> CommandResult:
        cmd: Dict[str, Any] = {"topic": "AcquireControl", "data": {"robot_name": robot_name}}
        if preset_name is not None:
            cmd["data"]["preset_name"] = preset_name
        return self._send_and_receive(cmd, request_opt_args=request_opt_args)

    def cancel_move(
            self, robot_name: str,
            request_opt_args: RequestOptArgs = RequestOptArgs()) -> CommandResult:
        cmd: Dict[str, Any] = {'topic': 'CancelMove', 'data': {'robot_name': robot_name}}
        return self._send_and_receive(cmd, request_opt_args=request_opt_args)

    # Move is the most complicated command. Thus have extra data class to support using it.
    class MoveGoalType(Enum):
        ''' Helper enum to fill in the goal type in move command.
        '''
        POSE: str = "pose"
        TARGET: str = "target"
        CONFIG: str = "config"

    def move(
            self,
            robot_name: str,
            speed: float,
            goal_type: MoveGoalType,
            goal_value: Union[str, List[float]],
            smoothing: Optional[float] = None,
            move_type: Optional[str] = None,
            ext_axes: Optional[List[float]] = None,
            interp: Optional[str] = None,
            relative: Optional[int] = None,
            ref_frame: Optional[str] = None,
            collision_check: Optional[int] = None,
            collision_check_dsm: Optional[int] = None,
            timeout: Optional[float] = None,  # This is timeout within ASCII API
            request_opt_args: RequestOptArgs = RequestOptArgs(),
            delay_timeout: float = _DEFAULT_DELAY_TIMEOUT_S) -> CommandResult:

        # These are the required parameters.
        data: Dict[str, Any] = {
            "robot_name": robot_name,
            "speed": speed,
            goal_type.value: goal_value  # value of enum class is the correct key for it.
        }
        # Optional arguments
        if smoothing is not None:
            data["smoothing"] = smoothing
        if move_type is not None:
            data["move_type"] = move_type
        if ext_axes is not None:
            data["ext_axes"] = ext_axes
        if interp is not None:
            data["interp"] = interp
        if relative is not None:
            data["relative"] = relative
        if ref_frame is not None:
            data["ref_frame"] = ref_frame
        if collision_check is not None:
            data["collision_check"] = collision_check
        if collision_check_dsm is not None:
            data["collision_check_dsm"] = collision_check_dsm
        if timeout is not None:
            data["timeout"] = timeout

        cmd: Dict[str, Any] = {"topic": "Move", "data": data}
        return self._send_and_receive(cmd,
                                      delayed_timeout_s=delay_timeout,
                                      request_opt_args=request_opt_args)

    def generate_move_data_dictionary(
            self,
            robot_name: str,
            speed: float,
            goal_type: MoveGoalType,
            goal_value: Union[str, List[float]],
            smoothing: Optional[float] = None,
            move_type: Optional[str] = None,
            ext_axes: Optional[List[float]] = None,
            interp: Optional[str] = None,
            relative: Optional[int] = None,
            ref_frame: Optional[str] = None,
            collision_check: Optional[int] = None,
            collision_check_dsm: Optional[int] = None,
            timeout: Optional[float] = None,  # This is timeout within ASCII API
    ) -> Dict[str, Any]:
        ''' This is to generate the yaml dictionary for a move command.

        This is intended to be used before calling combined move.
        '''
        data: Dict[str, Any] = {
            "robot_name": robot_name,
            "speed": speed,
            goal_type.value: goal_value  # value of enum class is the correct key for it.
        }
        # Optional arguments
        if smoothing is not None:
            data["smoothing"] = smoothing
        if move_type is not None:
            data["move_type"] = move_type
        if ext_axes is not None:
            data["ext_axes"] = ext_axes
        if interp is not None:
            data["interp"] = interp
        if relative is not None:
            data["relative"] = relative
        if ref_frame is not None:
            data["ref_frame"] = ref_frame
        if collision_check is not None:
            data["collision_check"] = collision_check
        if collision_check_dsm is not None:
            data["collision_check_dsm"] = collision_check_dsm
        if timeout is not None:
            data["timeout"] = timeout
        return data

    def combined_move(self,
                      robot_name: str,
                      moves: List[Dict[str, Any]],
                      request_opt_args: RequestOptArgs = RequestOptArgs(),
                      delayed_timeout_s: float = _DEFAULT_DELAY_TIMEOUT_S) -> CommandResult:
        ''' Combined multiple move commands to be executed as if it was one motion.
        Args:
            robot_name - The robot that this combined move is for
            moves - An array of up to 10 Move commands to be concatenated together. Each element
                of this list should be generated through generate_move_command_dictionary
        Return: CommandResult object
        '''
        data: Dict[str, Any] = {"robot_name": robot_name, "moves": moves}
        cmd: Dict[str, Any] = {'topic': 'CombinedMove', 'data': data}
        return self._send_and_receive(cmd,
                                      delayed_timeout_s=delayed_timeout_s,
                                      request_opt_args=request_opt_args)

    def set_interrupt_behavior(
        self,
        robot_name: str,
        timeout: int,
        max_replans: Optional[int] = None,
        request_opt_args: RequestOptArgs = RequestOptArgs()
    ) -> CommandResult:
        '''	This function should be invoked towards the beginning of a robot program. It sets
        the behavior for the server with respect to replanning and timeout behavior. If this
        function is not called, the default behavior for a project will be timeout = 30 and
        max_replans = 10
        '''
        #Required parameters
        cmd: Dict[str, Any] = {
            "topic": "SetInterruptBehavior",
            "data": {
                "robot_name": robot_name,
                "timeout": timeout
            }
        }

        # Optional arguments
        if max_replans is not None:
            cmd["data"]["max_replans"] = max_replans

        return self._send_and_receive(cmd, request_opt_args=request_opt_args)

    def get_joint_angles(
            self, robot_name: str,
            request_opt_args: RequestOptArgs = RequestOptArgs()) -> CommandResult:
        '''	This command is used to request the current joint angles of a robot.
        '''
        cmd: Dict[str, Any] = {"topic": "GetJointAngles", "data": {"robot_name": robot_name}}

        return self._send_and_receive(cmd, request_opt_args=request_opt_args)

    def get_tcp_pose(
        self,
        robot_name: str,
        ref_frame: Optional[str] = None,
        request_opt_args: RequestOptArgs = RequestOptArgs()
    ) -> CommandResult:
        '''	This command is used to request the current tcp pose of a robot. Optionally, a reference frame can be specified.
        '''
        cmd: Dict[str, Any] = {"topic": "GetTcpPose", "data": {"robot_name": robot_name}}

        if ref_frame is not None:
            cmd["data"]["ref_frame"] = ref_frame

        return self._send_and_receive(cmd, request_opt_args=request_opt_args)

    def set_alternate_location(
        self,
        robot_name: str,
        enabled: int,
        mode: int = 0,  # mode=0: MoveToAlternateLocation; mode=1: MoveAlongPathToGoal
        location: Union[str, List[float]] = None,
        complete_move: Optional[int] = None,  # only optional if enabled = 0
        request_opt_args: RequestOptArgs = RequestOptArgs()
    ) -> CommandResult:
        '''This command is used to enable and set the alternate location for a robot. With this
        feature enabled, in the event of a 4009 response to a MoveToPose or MoveToTarget command
        to the Realtime Controller, the Realtime Controller will not return the 4009 response code,
        and will instead automatically plan to the specified alternate position, and depending on
        the behavior of complete move either return a response or continue to plan to the location
        that was first specified in the move command.
        '''
        #Required parameters
        cmd: Dict[str, Any] = {
            "topic": "SetAlternateLocation",
            "data": {
                "robot_name": robot_name,
                "enabled": enabled,
                "mode": mode
            }
        }

        # Optional arguments
        if enabled and mode == 0:
            if type(location) is str:
                cmd["data"]["target_name"] = location
            elif type(location) is list:
                cmd["data"]["pose"] = location

        if complete_move is not None:
            cmd["data"]["complete_move"] = complete_move

        return self._send_and_receive(cmd, request_opt_args=request_opt_args)

    def set_default_project(
        self, project_name: str, request_opt_args: RequestOptArgs = RequestOptArgs()
    ) -> CommandResult:
        '''This command is used to define which project is the Default project that will
        automatically be loaded when RapidPlan powers on or is otherwise not in Operation Mode.
        '''
        cmd: Dict[str, Any] = {'topic': 'SetDefaultProject', 'data': {'project_name': project_name}}
        return self._send_and_receive(cmd, request_opt_args=request_opt_args)

    class ResponseType(Enum):
        ''' Helper enum for the api response type
        '''
        YAML: str = "yaml"
        CSV: str = "csv"

    def set_response_type(
        self, response_type: ResponseType, request_opt_args: RequestOptArgs = RequestOptArgs()
    ) -> CommandResult:
        '''This indicates which format the Realtime Controller will use when sending responses;
        either CSV or yaml. The default response type is yaml. The argument can be either case
        insensitive text or its numeric equivalent.
        '''
        cmd: Dict[str, Any] = {'topic': 'SetResponseType', 'data': {'response_type': response_type}}
        return self._send_and_receive(cmd, request_opt_args=request_opt_args)

    def set_units(
        self,
        length: Union[int, str],
        angle: Union[int, str],
        request_opt_args: RequestOptArgs = RequestOptArgs()
    ) -> CommandResult:
        '''This specifies the unit of measure for length and angles. The unit can be specified
        with a case insensitive string or its corresponding integer value.
            - length units: 0 (m), 1 (cm), 2 (mm), 3 (ft), 4 (in)
            - angle units:  0 (rads), 1 (degs)
           This configuration is stored per client.
           By default, appliance set units as 'millimeters' and 'degrees'
        '''
        cmd: Dict[str, Any] = {'topic': 'SetUnits', 'data': {'length': length, 'angle': angle}}
        return self._send_and_receive(cmd, request_opt_args=request_opt_args)

    def set_robot_preset(
            self,
            robot_name: str,
            preset_name: str,
            request_opt_args: RequestOptArgs = RequestOptArgs()) -> CommandResult:
        '''This command changes the active robot preset for a specific robot. This command can only
        fail with invalid arguments. When a preset is changed, current robot motions are not
        re-validated, but new move commands will respect the new active preset.
        '''
        cmd: Dict[str, Any] = {
            'topic': 'SetRobotPreset',
            'data': {
                'robot_name': robot_name,
                'preset_name': preset_name
            }
        }
        return self._send_and_receive(cmd, request_opt_args=request_opt_args)

    def set_object_state(
            self,
            object_name: str,
            state_name: str,
            request_opt_args: RequestOptArgs = RequestOptArgs()) -> CommandResult:
        '''	This command changes the active state of a specified object. This command can only fail
        with invalid arguments. When an object state is changed, current robot motions are not
        re-validated, but new move commands will respect the new active state.
        '''
        cmd: Dict[str, Any] = {
            'topic': 'SetObjectState',
            'data': {
                'object_name': object_name,
                'state_name': state_name
            }
        }
        return self._send_and_receive(cmd, request_opt_args=request_opt_args)

    def create_target(self,
                      target_name: str,
                      robot_name: str,
                      request_opt_args: RequestOptArgs = RequestOptArgs(),
                      delay_timeout: float = _DEFAULT_DELAY_TIMEOUT_S) -> CommandResult:
        '''This command creates a target in the active robot preset for the specified robot. The
        target will attempt to connect to all other targets assigned to the robot in the current
        preset with a joint space interpolation type.

        The target is created using the robots current joint angles which means the controller must
        be in CONFIG mode and the robot state must be connected.
        '''
        cmd: Dict[str, Any] = {
            'topic': 'CreateTarget',
            'data': {
                'target_name': target_name,
                'robot_name': robot_name
            }
        }
        return self._send_and_receive(cmd,
                                      delayed_timeout_s=delay_timeout,
                                      request_opt_args=request_opt_args)

    def update_target(self,
                      target_name: str,
                      robot_name: str,
                      request_opt_args: RequestOptArgs = RequestOptArgs(),
                      delay_timeout: float = _DEFAULT_DELAY_TIMEOUT_S) -> CommandResult:
        '''This command updates the specified targets position to be at the specified robots TCP
        location in its current preset. The targets new position will be used for all assigned
        robots.

        The target is created using the robots current joint angles which means the controller must
        be in CONFIG mode and the robot state must be connected.
        '''
        cmd: Dict[str, Any] = {
            'topic': 'UpdateTarget',
            'data': {
                'target_name': target_name,
                'robot_name': robot_name
            }
        }
        return self._send_and_receive(cmd,
                                      delayed_timeout_s=delay_timeout,
                                      request_opt_args=request_opt_args)

    #############################
    ####### DSM commands ########
    #############################

    def dsm_add_box(
        self,
        box_name: str,
        size: List[float],
        parent_frame: Optional[str] = None,
        offset: Optional[List[float]] = None,
        request_opt_args: RequestOptArgs = RequestOptArgs()
    ) -> CommandResult:
        ''' This command adds a box into the Dynamic Scene Model. Once added, all robots in all 
        robot presets will avoid it. Additionally, current robot motions will avoid the box when added.

        The box has a fixed offset relative to the parent frame, and if the parent frame’s position 
        is updated, the position of the box will be as well.

        If a box is being added to a robots tcp, that robot must be stationary.

        The Dynamic Scene Model is not stored to disk, so when the Project is unloaded, the Dynamic 
        Scene Model is reset.    
        '''
        cmd: Dict[str, Any] = {'topic': 'AddBox', 'data': {'box_name': box_name, 'size': size}}

        if parent_frame is not None:
            cmd['data']['parent_frame'] = parent_frame

        if offset is not None:
            cmd['data']['offset'] = offset

        return self._send_and_receive(cmd, request_opt_args)

    def dsm_add_object(
        self,
        object_name: str,
        object_type: str,
        parent_frame: str = None,
        offset: Optional[List[float]] = None,
        request_opt_args: RequestOptArgs = RequestOptArgs()
    ) -> CommandResult:
        ''' This command adds a CAD object from the RPC project into the Dynamic Scene Model. 
        Once added, all robots in all robot presets will avoid it. Additionally, current robot 
        motions will avoid the box when added.

        The object has a fixed offset relative to the parent frame, and if the parent frame’s 
        position is updated, the position of the object will be as well.

        The Dynamic Scene Model is not stored to disk, so when the Project is unloaded, the 
        Dynamic Scene Model is reset.   
        '''
        cmd: Dict[str, Any] = {
            'topic': 'AddObject',
            'data': {
                'object_name': object_name,
                'object_type': object_type
            }
        }

        if parent_frame is not None:
            cmd['data']['parent_frame'] = parent_frame

        if offset is not None:
            cmd['data']['offset'] = offset

        return self._send_and_receive(cmd, request_opt_args)

    def dsm_add_frame(
        self,
        frame_name: str,
        parent_frame: Optional[str] = None,
        offset: Optional[List[float]] = None,
        request_opt_args: RequestOptArgs = RequestOptArgs()
    ) -> CommandResult:
        '''This command adds a frame to the scene. The frame can be used as a parent in the DSM 
        or as a reference frame for a relative move type.

        The frame can be used as a reference/parent frame for:
            Move commands
            Dynamic Scene Model boxes and objects
            Other frames   
        '''
        cmd: Dict[str, Any] = {'topic': 'AddFrame', 'data': {'frame_name': frame_name,}}

        if parent_frame is not None:
            cmd['data']['parent_frame'] = parent_frame

        if offset is not None:
            cmd['data']['offset'] = offset

        return self._send_and_receive(cmd, request_opt_args)

    def dsm_update_frame(
        self,
        frame_name: str,
        offset: Optional[List[float]] = None,
        pose: Optional[List[float]] = None,
        reference_frame: str = None,
        request_opt_args: RequestOptArgs = RequestOptArgs()
    ) -> CommandResult:
        '''This command updates the position of a frame and all of it’s children. A frame can be 
        updated by applying an offset from it’s current position with the offset argument. 
        A frame can also be relocated to a new position in a given frame. 
        It is important to note that this cannot re-parent the frame.

        Frames can only be updated if they:
            Are static (eg. they are not attached to a robot)
            Are not attached to stateful objects (eg. none of the parent mates can have more than one state)
            Have no children aside from:
                DSM Boxes
                DSM Objects
                Other frames 
        '''
        cmd: Dict[str, Any] = {'topic': 'UpdateFrame', 'data': {'frame_name': frame_name,}}

        if reference_frame is not None:
            cmd['data']['reference_frame'] = reference_frame

        if offset is not None:
            cmd['data']['offset'] = offset

        if pose is not None:
            cmd['data']['pose'] = pose

        return self._send_and_receive(cmd, request_opt_args)

    def dsm_remove_frames(
        self,
        frame_names: Optional[List[str]] = None,
        request_opt_args: RequestOptArgs = RequestOptArgs()
    ) -> CommandResult:
        '''This command removes a specified frame/s added to the scene. When a frame is removed, 
        any DSM boxes or objects with that frame as a parent will also be removed. 
        If no arguments are passed, all frames will be removed, or an array of the frame names 
        can be passed to selectively remove.
        '''
        cmd: Dict[str, Any] = {'topic': 'RemoveFrames', 'data': {}}

        if frame_names is not None:
            cmd['data']['frame_names'] = frame_names

        return self._send_and_receive(cmd, request_opt_args)

    def dsm_remove_boxes(
        self,
        box_names: Optional[List[str]] = None,
        request_opt_args: RequestOptArgs = RequestOptArgs()
    ) -> CommandResult:
        '''This command removes a specified box/es from the Dynamic Scene Model. 
        If no arguments are passed, all boxes will be removed, or an array of 
        the box names can be passed to selectively remove.
        '''
        cmd: Dict[str, Any] = {'topic': 'RemoveBoxes', 'data': {}}

        if box_names is not None:
            cmd['data']['box_names'] = box_names

        return self._send_and_receive(cmd, request_opt_args)

    def dsm_remove_objects(
        self,
        object_names: Optional[List[str]] = None,
        request_opt_args: RequestOptArgs = RequestOptArgs()
    ) -> CommandResult:
        '''This command removes a specified object/es from the Dynamic Scene Model. 
        If no arguments are passed, all boxes will be removed, or an array of the 
        box names can be passed to selectively remove.
        '''
        cmd: Dict[str, Any] = {'topic': 'RemoveObjects', 'data': {}}

        if object_names is not None:
            cmd['data']['object_names'] = object_names

        return self._send_and_receive(cmd, request_opt_args)

    def dsm_clear(
        self,
        boxes: Optional[bool] = False,
        objects: Optional[bool] = False,
        request_opt_args: RequestOptArgs = RequestOptArgs()
    ) -> CommandResult:
        '''This command removes all boxes and or objects from the Dynamic Scene Model. 
        If no arguments are passed, all objects and boxes will be removed. If a flag is specified, 
        then only that type will be removed, and multiple flags can be True at once.
        '''
        cmd: Dict[str, Any] = {'topic': 'ClearDSM', 'data': {}}

        if boxes:
            cmd['data']['boxes'] = True

        if objects:
            cmd['data']['objects'] = True

        return self._send_and_receive(cmd, request_opt_args)

    def dsm_release_box(
            self, box_name: str,
            request_opt_args: RequestOptArgs = RequestOptArgs()) -> CommandResult:
        '''This command breaks (aka releases) the fixed transform between the specified box and 
        the robot it is currently attached to. After this command, the box will remain at 
        the same position, and will no longer move with the robot.
        '''
        cmd: Dict[str, Any] = {'topic': 'ReleaseBox', 'data': {'box_name': box_name,}}

        return self._send_and_receive(cmd, request_opt_args)

    def dsm_reparent(
        self, box_name: str, parent_frame: str, request_opt_args: RequestOptArgs = RequestOptArgs()
    ) -> CommandResult:
        ''' This command will re-assign the parent frame for a specified DSM box. 
        After this command executes, the box will have a new parent frame with an offset such that 
        it remains in it’s current position. If the box’s new parent frame is moved, 
        the box will move with it with that offset. 

        A box’s parent frame can also be the TCP of a robot, and as the robot moves, the box moves 
        with it maintaining the initial offset. A robot’s default TCP should be used that way 
        it is independent of presets. A robot’s default TCP can be specified as {robot_name}_default_tcp. 
        In order to assign a robot’s tcp as the parent of a box, the robot must be stationary. 
        '''
        cmd: Dict[str, Any] = {
            'topic': 'Reparent',
            'data': {
                'box_name': box_name,
                'parent_frame': parent_frame
            }
        }

        return self._send_and_receive(cmd, request_opt_args)

    #############################
    ### Choreography commands ###
    #############################

    def choreography_install(self, zip_file_path: str) -> CommandResult:
        cmd: Dict[str, Any] = {
            "topic": "ChoreographyInstall",
            "data": {
                "zip_file_path": zip_file_path
            }
        }
        return self._send_and_receive(cmd)

    def choreography_load(self, project_name: str) -> CommandResult:
        cmd: Dict[str, Any] = {"topic": "ChoreographyLoad", "data": {"project_name": project_name}}
        return self._send_and_receive(cmd)

    def choreography_stage(self, speed_factor: float) -> CommandResult:
        cmd: Dict[str, Any] = {"topic": "ChoreographyStage", "data": {"speed_factor": speed_factor}}
        return self._send_and_receive(cmd)

    def choreography_run(self,
                         auto_step_at_target: Optional[bool] = None,
                         num_cycles: Optional[int] = None,
                         request_opt_args: RequestOptArgs = RequestOptArgs(),
                         delay_timeout: float = _DEFAULT_DELAY_TIMEOUT_S) -> CommandResult:
        cmd: Dict[str, Any] = {"topic": "ChoreographyRun", "data": {}}
        if auto_step_at_target is not None:
            cmd["data"]["auto_step_at_target"] = bool(auto_step_at_target)
        if num_cycles is not None:
            cmd["data"]["num_cycles"] = num_cycles
        return self._send_and_receive(cmd,
                                      delayed_timeout_s=delay_timeout,
                                      request_opt_args=request_opt_args,
                                      forever_receiving=True)

    def choreography_step(self,
                          performer_name: str,
                          segment_index: int,
                          cycle_number: int,
                          request_opt_args: RequestOptArgs = RequestOptArgs(),
                          delay_timeout: float = _DEFAULT_DELAY_TIMEOUT_S) -> CommandResult:
        cmd: Dict[str, Any] = {
            "topic": "ChoreographyStep",
            "data": {
                "performer_name": performer_name,
                "segment_index": segment_index,
                "cycle_number": cycle_number
            }
        }
        return self._send_and_receive(cmd,
                                      delayed_timeout_s=delay_timeout,
                                      request_opt_args=request_opt_args)

    def choreography_cancel(self) -> CommandResult:
        cmd: Dict[str, Any] = {"topic": "ChoreographyCancel"}
        return self._send_and_receive(cmd)

    def choreography_unload(self) -> CommandResult:
        cmd: Dict[str, Any] = {"topic": "ChoreographyUnload"}
        return self._send_and_receive(cmd)

    def choreography_unstage(self) -> CommandResult:
        cmd: Dict[str, Any] = {"topic": "ChoreographyUnstage"}
        return self._send_and_receive(cmd)

    def choreography_uninstall(self, project_name: str) -> CommandResult:
        cmd: Dict[str, Any] = {
            "topic": "ChoreographyUninstall",
            "data": {
                "project_name": project_name
            }
        }
        return self._send_and_receive(cmd)
