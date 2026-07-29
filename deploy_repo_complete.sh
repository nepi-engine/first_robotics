#!/bin/bash
##
## Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
##
## This file is part of nepi-engine
## (see https://github.com/nepi-engine).
##
## License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
##

#######################################################################################################
# Usage: $ ./deploy_nepi_engine_complete.sh
#
# This script copies the complete nepi_engine source code to proper filesystem locations on target
# hardware in preparation for building nepi-engine from source. 
#
#
# The script requires the following environment variable be set
#    NEPI_REMOTE_SETUP: Indicates whether running from development host or directly on target 
#                      (1 = Dev. Host, 0 = From Target)
# In the case that NEPI_REMOTE_SETUP == 1, some further environment variables must be set
#    NEPI_TARGET_IP: Target IP address/hostname
     NEPI_TARGET_IP=${NEPI_IP} #/${NEPI_DEVICE_ID}
     echo "Using target IP: ${NEPI_TARGET_IP}"
#    NEPI_DEPLOY_USERNAME: Target username

     NEPI_DEPLOY_USERNAME=nepihost
     NEPI_SSH_PORT=22
#    NEPI_SSH_KEY: Private SSH key for SSH/Rsync to target (as applicable)
     NEPI_SSH_KEY=/home/${USER}/.ssh/nepi_default_ssh_key
#    NEPI_TARGET_SRC_DIR: Directory to deploy source code to
     NEPI_TARGET_SRC_DIR=/mnt/nepi_storage/nepi_src
#    NEPI_SETUP_SRC_DIR: Directory to deploy setup source to
     NEPI_SETUP_SRC_DIR=/home/nepihost
#######################################################################################################
# # Clear known hosts keys
# sudo rm /home/${USER}/.ssh/known*
########################################
# sudo -v





# Set NEPI folder variables if not configured by nepi aliases bash script
if [[ ! -v NEPI_USER ]]; then
    NEPI_USER=nepi
fi
if [[ ! -v NEPI_HOME ]]; then
    NEPI_HOME=/home/${NEPI_USER}
fi
if [[ ! -v NEPI_DOCKER ]]; then
    NEPI_DOCKER=/mnt/nepi_docker
fi
if [[ ! -v NEPI_STORAGE ]]; then
   NEPI_STORAGE=/mnt/nepi_storage
fi

if [[ ! -v NEPI_CONFIG ]]; then
    NEPI_CONFIG=/mnt/nepi_config
fi
if [[ ! -v NEPI_BASE ]]; then
    NEPI_BASE=/opt/nepi
fi
if [[ ! -v NEPI_RUI ]]; then
    NEPI_RUI=${NEPI_BASE}/nepi_rui
fi
if [[ ! -v NEPI_ENGINE ]]; then
    NEPI_ENGINE=${NEPI_BASE}/nepi_engine
fi
if [[ ! -v NEPI_ETC ]]; then
    NEPI_ETC=${NEPI_BASE}/etc
fi


if [[ -z "${NEPI_REMOTE_SETUP}" ]]; then
  echo "Must have environtment variable NEPI_REMOTE_SETUP set"
  exit 1
fi

if [ "${NEPI_REMOTE_SETUP}" == "0" ]; then
    echo "Running in Local Mode"

elif [ "${NEPI_REMOTE_SETUP}" == "1" ]; then
  echo "Running in Remote Mode"
  if [[ -z "${NEPI_TARGET_IP}" ]]; then
    echo "Remote setup requires env. variable NEPI_TARGET_IP be assigned"
    exit 1
  fi

  if [[ -z "${NEPI_DEPLOY_USERNAME}" ]]; then
    echo "Remote setup requires env. variable NEPI_DEPLOY_USERNAME be assigned"
    exit 1
  fi
  if [[ -z "${NEPI_SSH_KEY}" ]]; then
    echo "Remote setup requires env. variable NEPI_SSH_KEY be assigned"
    exit 1
  fi
fi

echo "Deploying to NEPI target IP: ${NEPI_TARGET_IP} (port ${NEPI_SSH_PORT})"
# ## Synce update remote clock if needed
# echo "Syncing remote clock if needed"
# if [ "${NEPI_REMOTE_SETUP}" == "1" ]; then
#   sshnhc
# fi
source /home/${USER}/.bashrc

reset_path=$(pwd)

###############################################
# Deploy Nepi Engine Overrides Folders and Files
###############################################
RSYNC_EXCLUDES=" --exclude .git --exclude .gitmodules --exclude empty.txt"
#echo "Excluding ${RSYNC_EXCLUDES}"


SOURCE_PATH=$(pwd)/nepi_engine_overrides
SOURCE_DEST_PATH=${NEPI_CONFIG}/system_cfg/src
# echo -e "${GREEN}Clearing __pycache__ folders"
find $SOURCE_PATH -type d -name "__pycache__" -exec rm -rf {} +
echo ""
echo "Syncing NEPI system config from ${SOURCE_PATH} to ${SOURCE_DEST_PATH}"
if [ "${NEPI_REMOTE_SETUP}" == "0" ]; then
  sudo rm -r /mnt/nepi_config/system_cfg/src 2>/dev/null
  rsync -arh  --delete ${RSYNC_EXCLUDES} ${SOURCE_PATH}/ ${SOURCE_DEST_PATH}/

elif [ "${NEPI_REMOTE_SETUP}" == "1" ]; then
  ssh -o StrictHostKeyChecking=no -p 22 -i ${NEPI_SSH_KEY} ${NEPI_DEPLOY_USERNAME}@${NEPI_TARGET_IP} \
  "sudo -S rm -r /mnt/nepi_config/system_cfg/src "
  echo 'rsync -avzhe "ssh -i '${NEPI_SSH_KEY}' -p '${NEPI_SSH_PORT}' -o StrictHostKeyChecking=no" --delete '${RSYNC_EXCLUDES}' '${SOURCE_PATH}'/ '${NEPI_DEPLOY_USERNAME}'@'${NEPI_TARGET_IP}':'${SOURCE_DEST_PATH}'/'
  rsync -azhe "ssh -i ${NEPI_SSH_KEY} -p ${NEPI_SSH_PORT} -o StrictHostKeyChecking=no" ${RSYNC_EXCLUDES} ${SOURCE_PATH}/ ${NEPI_DEPLOY_USERNAME}@${NEPI_TARGET_IP}:${SOURCE_DEST_PATH}/

fi


###############################################
# Deploy Nepi Apps
###############################################
  APPS="nepi_app_obstacles nepi_app_auto_move nepi_app_stereo_cam nepi_app_wpilib_if "

  RSYNC_EXCLUDES=" --exclude .git --exclude .gitmodules --exclude empty.txt"
  #echo "Excluding ${RSYNC_EXCLUDES}"

  DEST_PATH=${NEPI_TARGET_SRC_DIR}/nepi_engine_ws/nepi_apps
  for APP in $APPS; do
    APP_PATH="$(pwd)/${APP}"
    echo ""
    echo "Syncing repo ${APP} from ${APP_PATH} to"
    echo "${DEST_PATH}"
    sleep 1
    # Push everything but the EXCLUDES to the specified source folder on the target

    if [ "${NEPI_REMOTE_SETUP}" == "0" ]; then
      sudo rsync -arh  --chown=1000:1000 ${RSYNC_EXCLUDES} $(pwd)/${APP} ${DEST_PATH}/

    elif [ "${NEPI_REMOTE_SETUP}" == "1" ]; then
      rsync -azhe "ssh -i ${NEPI_SSH_KEY} -o StrictHostKeyChecking=no" --chown=1000:1000 ${RSYNC_EXCLUDES} $(pwd)/${APP} ${NEPI_DEPLOY_USERNAME}@${NEPI_TARGET_IP}:${DEST_PATH}/

    fi

  done


###############################################
# Deploy Nepi Test Data
###############################################

SCRIPTS_SOURCE_PATH=$(pwd)/test_data
SCRIPTS_DEST_PATH=/mnt/nepi_storage/test_data
echo ""
echo "Syncing NEPI test_data from ${SCRIPTS_SOURCE_PATH} to ${SCRIPTS_DEST_PATH}"
if [ "${NEPI_REMOTE_SETUP}" == "0" ]; then
  rsync -arh  --delete ${RSYNC_EXCLUDES} ${SCRIPTS_SOURCE_PATH}/ ${SCRIPTS_DEST_PATH}/

elif [ "${NEPI_REMOTE_SETUP}" == "1" ]; then
  rsync -azhe "ssh -i ${NEPI_SSH_KEY} -p ${NEPI_SSH_PORT} -o StrictHostKeyChecking=no" --delete ${RSYNC_EXCLUDES} ${SCRIPTS_SOURCE_PATH}/ ${NEPI_DEPLOY_USERNAME}@${NEPI_TARGET_IP}:${SCRIPTS_DEST_PATH}/

fi

###############################################
# Deploy Nepi Scripts
###############################################

# SCRIPTS_SOURCE_PATH=$(pwd)/src/nepi_scripts
# SCRIPTS_DEST_PATH=/mnt/nepi_storage/nepi_scripts
# echo ""
# echo "Syncing NEPI scripts from ${SCRIPTS_SOURCE_PATH} to ${SCRIPTS_DEST_PATH}"
# if [ "${NEPI_REMOTE_SETUP}" == "0" ]; then
#   rsync -arh  --delete ${RSYNC_EXCLUDES} ${SCRIPTS_SOURCE_PATH}/ ${SCRIPTS_DEST_PATH}/

# elif [ "${NEPI_REMOTE_SETUP}" == "1" ]; then
#   rsync -azhe "ssh -i ${NEPI_SSH_KEY} -p ${NEPI_SSH_PORT} -o StrictHostKeyChecking=no" --delete ${RSYNC_EXCLUDES} ${SCRIPTS_SOURCE_PATH}/ ${NEPI_DEPLOY_USERNAME}@${NEPI_TARGET_IP}:${SCRIPTS_DEST_PATH}/

# fi


# NEPI_ENGINE_WS_FOLDER=/home/${USER}/nepi_engine_ws
# echo ""
# echo "Running NEPI Deploy"
# run_folder=$(pwd)
# cd $NEPI_ENGINE_WS_FOLDER
# source ./deploy_nepi_complete.sh
# cd $run_folder

cd $reset_path
