#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_apps) repo
# (see https://https://github.com/nepi-engine/nepi_apps)
#
# License: nepi applications are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#

import copy

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_obstacles

from nepi_api.messages_if import MsgIF
from nepi_api.obstacles_if import ObstaclesIF


#########################################
# Node Class
#########################################

class NepiObstaclesApp(object):

    #######################
    ### Node Initialization
    DEFAULT_NODE_NAME = "app_obstacles"  # Can be overwritten by launch command

    PROCESS_DESCRIPTION = 'Obstacle localization process'

    obstacles_if = None

    def __init__(self):
        #### APP NODE INIT SETUP ####
        nepi_sdk.init_node(name = self.DEFAULT_NODE_NAME)
        self.class_name = type(self).__name__
        self.base_namespace = nepi_sdk.get_base_namespace()
        self.node_name = nepi_sdk.get_node_name()
        self.node_namespace = nepi_sdk.get_node_namespace()

        ##############################
        # Create Msg Class
        self.msg_if = MsgIF(log_name = self.class_name)
        self.msg_if.pub_info("Starting Node Initialization Processes")

        ##############################
        # Create the Obstacles IF.
        #
        # controls_dict is the SDK module's control DEFINITION dict (the
        # nepi_controls init-dict form). ObstaclesIF hands it to a ControlsIF,
        # and hands the ControlsIF's live controls_dict back to processData on
        # every cycle. The node never reads control values itself.
        controls_dict = copy.deepcopy(nepi_obstacles.PROCESS_CONTROLS)

        self.obstacles_if = ObstaclesIF(
                            namespace = self.node_namespace,
                            description = self.PROCESS_DESCRIPTION,
                            controls_dict = controls_dict,
                            processDataFunction = self.processData,
                            msg_if = self.msg_if
                            )

        #########################################################
        ## Initiation Complete
        self.msg_if.pub_info("Initialization Complete")

        # Spin forever
        nepi_sdk.spin()
        #########################################################

    def processData(self, np_depth_map, status_dict, navpose_dict, controls_dict):
        # Called by ObstaclesIF once per accepted depth map frame. Thin
        # pass-through to the SDK module so the algorithm ships in nepi_sdk and
        # any other node can call it with the same four arguments.
        return nepi_obstacles.process_depth_map(np_depth_map,
                                                status_dict,
                                                navpose_dict,
                                                controls_dict)


#########################################
# Main
#########################################
if __name__ == '__main__':
    NepiObstaclesApp()
