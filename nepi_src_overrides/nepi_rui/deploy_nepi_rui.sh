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
# Usage: $ ./deploy_nepi_rui.sh
#
# Derived from deploy_nepi_source.sh, scoped to this single component folder.
# It copies the contents of the "nepi_rui" folder (the folder this script lives in)
# to the NEPI target's system config source overlay at:
#
#     ${NEPI_CONFIG}/system_cfg/src/nepi_rui/
#
# It can be run from a development host or directly on the target hardware as described
# in this repository's README.
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
sudo -v


REPO="nepi_rui"


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

reset_path=$(pwd)

# Resolve this script's own folder -- that folder IS the deploy source, so the
# script works no matter what directory it is invoked from.
REPO_FOLDER=$(cd -P "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)

echo "Deploying ${REPO} to NEPI target IP: ${NEPI_TARGET_IP} (port ${NEPI_SSH_PORT})"
# ## Synce update remote clock if needed
# echo "Syncing remote clock if needed"
# if [ "${NEPI_REMOTE_SETUP}" == "1" ]; then
#   sshnhc
# fi
source /home/${USER}/.bashrc

# NOTE: keep these patterns glob-free -- an unquoted "*" in this string would be
# glob-expanded by the shell into extra rsync source arguments. __pycache__ is
# handled by the explicit find below instead.
RSYNC_EXCLUDES=" --exclude .git --exclude .gitmodules --exclude empty.txt --exclude deploy_${REPO}.sh"

echo "Excluding ${RSYNC_EXCLUDES}"


# Deploy this component folder into the system config source overlay
SOURCE_PATH=${REPO_FOLDER}
SOURCE_DEST_PATH=${NEPI_CONFIG}/system_cfg/src/${REPO}
echo "Clearing __pycache__ folders"
find "${SOURCE_PATH}" -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null
echo "Syncing NEPI ${REPO} from ${SOURCE_PATH} to ${SOURCE_DEST_PATH}"
if [ "${NEPI_REMOTE_SETUP}" == "0" ]; then
  sudo rm -r ${SOURCE_DEST_PATH} 2>/dev/null
  rsync -avrh --delete ${RSYNC_EXCLUDES} ${SOURCE_PATH}/ ${SOURCE_DEST_PATH}/

elif [ "${NEPI_REMOTE_SETUP}" == "1" ]; then
  ssh -o StrictHostKeyChecking=no -p ${NEPI_SSH_PORT} -i ${NEPI_SSH_KEY} ${NEPI_DEPLOY_USERNAME}@${NEPI_TARGET_IP} \
  "sudo -S rm -r ${SOURCE_DEST_PATH}"
  echo 'rsync -avzhe "ssh -i '${NEPI_SSH_KEY}' -p '${NEPI_SSH_PORT}' -o StrictHostKeyChecking=no" --delete '${RSYNC_EXCLUDES}' '${SOURCE_PATH}'/ '${NEPI_DEPLOY_USERNAME}'@'${NEPI_TARGET_IP}':'${SOURCE_DEST_PATH}'/'
  rsync -avzhe "ssh -i ${NEPI_SSH_KEY} -p ${NEPI_SSH_PORT} -o StrictHostKeyChecking=no" --delete ${RSYNC_EXCLUDES} ${SOURCE_PATH}/ ${NEPI_DEPLOY_USERNAME}@${NEPI_TARGET_IP}:${SOURCE_DEST_PATH}/

fi

cd $reset_path
