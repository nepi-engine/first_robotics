#!/bin/bash
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##



import copy

from nepi_interfaces.msg import Targets

import Obstacle, Obstacles

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav



from nepi_sdk.nepi_sdk import logger as Logger
log_name = "nepi_process"
logger = Logger(log_name = log_name)


SOURCE_MESSAGE_DICT = {'Targets' : Targets}

DATA_DICT = {
    # Required Fields
    'data_time': 0.0,
    'process_time': 0.0,

}


PROCESSES_DICT = dict()

DEFAULT_PROCESS = 'process_1'

#########################
# Process Process Functions
#########################




############################

process_1_settings = {
    # Required Fields
    'setting_1': 5,

    # Custom Fields. Automatically Populated in RUI
    'controls_dict': {
        'control_1': 2.0
    }
}


def process_1(process_data_dict, 
                process_settings_dict
                ):

    #logger.log_info("******")
    #logger.log_info("*** Processs Solution Update Starting ***")
    #logger.log_info("******")
    start_time = nepi_utils.get_time()
            
    return process_data_dict, process_settings_dict



PROCESSES_DICT['process_1'] = {'process_function': process_1, 
                                'default_settings_dict': process_1_settings}


############################



process_2_settings = {
    # Required Fields
    'setting_1': 5,

    # Custom Fields. Automatically Populated in RUI
    'controls_dict': {
        'control_1': 2.0
    }
}



def process_2(process_data_dict, 
                process_settings_dict
                ):

    #logger.log_info("******")
    #logger.log_info("*** Processs Solution Update Starting ***")
    #logger.log_info("******")
    start_time = nepi_utils.get_time()
            
    return process_data_dict, process_settings_dict



PROCESSES_DICT['process_2'] = {'process_function': process_2, 
                                'default_settings_dict': process_2_settings}



#########################
# Process Utility Functions
#########################

def create_processes_dict():
    processes_dict = dict()
    for process_name in PROCESSES_DICT.keys():
        processes_dict[process_name] = PROCESSES_DICT[process_name]['default_settings_dict']
    return processes_dict

def update_processes_dict(process_processes_dict):
    clean_process_dict = create_processes_dict()
    for process_process in clean_process_dict.keys():
        if process_process in process_processes_dict.keys():
            for key in clean_process_dict[process_process].keys():
                if key in process_processes_dict[process_process].keys() and key != 'process_controls_dict':
                    clean_process_dict[process_process][key] = process_processes_dict[process_process][key]
            for key in clean_process_dict[process_process]['process_controls_dict'].keys():
                if key in process_processes_dict[process_process]['process_controls_dict'].keys():
                    clean_process_dict[process_process]['process_controls_dict'][key] = process_processes_dict[process_process]['process_controls_dict'][key]
    return clean_process_dict

def get_blank_data_dict():
    return copy.deepcopy(DATA_DICT)