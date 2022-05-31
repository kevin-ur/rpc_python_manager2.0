''' Python logging module similar to our c++ logging module 

This module is designed to operate the same way as our c++ logging module 
Module will search for python_config.yaml in project folder to set RTR_DEFAULT_DIR
LOG format is set by env-var RTR_LOG_STYLE
LOG file location is set to 1.pass in parameter, 2. env-var RTR_LOG_DIR 3. RTR_DEFAULT_DIR
If neither locations are not available, then no logs will be generated. 

typical usage
executable script: 
from rtr_utils import rtr_logging
from rtr_utils.rtr_logging import rtr_debug
from rtr_utils.rtr_logging import rtr_info
from rtr_utils.rtr_logging import rtr_warn
from rtr_utils.rtr_logging import rtr_error
...
rtr_logging.initialize_logger(... , ...)
rtr_info("program start")

for non-executable modules:
from rtr_utils.rtr_logging import rtr_debug
from rtr_utils.rtr_logging import rtr_info
from rtr_utils.rtr_logging import rtr_warn
from rtr_utils.rtr_logging import rtr_error
...
rtr_info("log message")

'''

import logging
from logging.handlers import RotatingFileHandler
import sys
import os
import pathlib
import warnings
import yaml

FORMAT_PATTERN = '[%(asctime)s.%(msecs)d] (%(process)d:%(thread)d-%(threadName)s) [%(filename)s:%(lineno)d] [%(levelname)s] %(message)s'
ALT1_FORMAT_PATTERN = '[%(asctime)s.%(msecs)d] [%(levelname)s] %(message)s'
ALT2_FORMAT_PATTERN = '[%(asctime)s.%(msecs)d] (%(process)d:%(thread)d-%(threadName)s) [%(levelname)s] %(message)s'
ALT3_FORMAT_PATTERN = '[%(asctime)s.%(msecs)d] (%(process)d:%(thread)d-%(threadName)s) [%(filename)s:%(lineno)d] [%(levelname)s] %(message)s'

DATE_FORMAT = '%Y-%m-%d:%H:%M:%S'
ALT1_DATE_FORMAT = '%s'

RTR_DEFAULT_DIR = None
_rtr_logger = logging.getLogger("rtr_logger")

# The following alias should be imported for logging.
rtr_debug = _rtr_logger.debug
rtr_info = _rtr_logger.info
rtr_warn = _rtr_logger.warning
rtr_error = _rtr_logger.error
rtr_exception = _rtr_logger.exception

# Try to find RTR_DEFAULT_DIR using cmake config file on module load
if RTR_DEFAULT_DIR is None:
    module_path = pathlib.Path(__file__)
    cmake_path = module_path.parent / 'python_config.yaml'
    if cmake_path.is_file():
        with open(cmake_path) as f:
            data = yaml.safe_load(f)
            cmake_install_path = pathlib.Path(data['cmake_install_prefix'])
            RTR_DEFAULT_DIR = cmake_install_path / 'var/log/rtr'
    else:
        warnings.warn(
            "Cannot find yaml config for default directory. "
            "Setting default directory to empty string. Is rtr_utils built?", RuntimeWarning, 2)
    # Not setting RTR_DEFAULT_DIR keeps it a None and used for other logics


def _get_log_level():
    level = os.getenv('RTR_LOG_LEVEL')
    if level == "ERROR":
        return logging.ERROR
    elif level == "WARN":
        return logging.WARN
    elif level == "INFO":
        return logging.INFO
    else:
        return logging.DEBUG


def _get_log_format():
    style = os.getenv('RTR_LOG_STYLE')
    if style == "ALT":
        return (FORMAT_PATTERN, DATE_FORMAT)
    elif style == "ALT1":
        return (ALT1_FORMAT_PATTERN, ALT1_DATE_FORMAT)
    elif style == "ALT2":
        return (ALT2_FORMAT_PATTERN, ALT1_DATE_FORMAT)
    elif style == "ALT3":
        return (ALT3_FORMAT_PATTERN, ALT1_DATE_FORMAT)
    else:
        return (ALT3_FORMAT_PATTERN, DATE_FORMAT)


def _optimize_logger_settings(log_style):
    global _rtr_logger

    if _rtr_logger is None:
        warnings.warn("rtr_logger not initialized", RuntimeWarning, 2)
        return

    # Runtime optimization if file tracing is unused
    if (log_style == ALT1_FORMAT_PATTERN) or (log_style == ALT2_FORMAT_PATTERN):
        _rtr_logger._srcfile = None

    # Runtime optimization if PID/TID tracing is unused
    if log_style == ALT1_FORMAT_PATTERN:
        _rtr_logger.logThreads = 0
        _rtr_logger.logProcesses = 0


def _get_prioritized_log_dir(args_dir, conf_dir):
    if args_dir is not None:
        return args_dir

    env_dir = os.getenv('RTR_LOG_DIR')
    if env_dir is not None:
        return env_dir

    if conf_dir is not None:
        return conf_dir

    return RTR_DEFAULT_DIR


def _initialize_log_dir(args_dir, conf_dir):
    log_dir = _get_prioritized_log_dir(args_dir, conf_dir)
    if log_dir is None:
        return None

    p = pathlib.Path(log_dir)
    # resolve the path to absolute path. this also resolve symbol link
    p = p.resolve()
    p.mkdir(parents=True, exist_ok=True)

    return p


def initialize_logger(program_name, args_dir=None, conf_dir=None):
    global _rtr_logger

    # If _rtr_logger have handler under it, means it is already initialized.
    if _rtr_logger.handlers:
        rtr_warn("Intializing logging when it is already initialized!")
        return

    # Reset logger in case rospy is imported. Importing rospy overwrites the
    # default logger class and ROS sucks because they don't revert this setting
    # after they create their own loggers.
    logging.setLoggerClass(logging.Logger)
    _rtr_logger = logging.getLogger("rtr_logger")

    # Construct log format
    level = _get_log_level()
    log_fmt, date_fmt = _get_log_format()
    formatter = logging.Formatter(fmt=log_fmt, datefmt=date_fmt)

    _rtr_logger.setLevel(level)
    _rtr_logger.propagate = False

    # Construct console stream handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(level)
    console_handler.setFormatter(formatter)
    _rtr_logger.addHandler(console_handler)

    # Construct file I/O handler
    log_dir = _initialize_log_dir(args_dir, conf_dir)

    if log_dir is None:
        rtr_warn("Not creating file handler since no log path is available")
    else:
        fname = log_dir / "{}.{}".format(program_name, 'log')

        log_size = 1024 * 1024 * 100  # 100 MB
        num_files = 10
        file_handler = logging.handlers.RotatingFileHandler(fname,
                                                            maxBytes=log_size,
                                                            backupCount=num_files)
        file_handler.setLevel(level)
        file_handler.setFormatter(formatter)
        _rtr_logger.addHandler(file_handler)
        rtr_info("Logging to {}".format(fname))

    # Optimize logger settings
    _optimize_logger_settings(log_fmt)

    rtr_info("Logging initialized.")
