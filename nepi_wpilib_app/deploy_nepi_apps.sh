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
# deploy_nepi_apps.sh [app_folder ...]
#
# Deploys NEPI app package folders to the correct place in the target's
# nepi_engine_ws source tree. With no arguments it deploys ALL nepi_app_*
# folders sitting next to this script; pass folder names to deploy a subset:
#     ./deploy_nepi_apps.sh nepi_app_template
#
# Unlike drivers (which are flattened into a shared folder), each app is a
# complete catkin package and is synced AS A FOLDER into:
#
#     <src>/nepi_engine_ws/src/nepi_apps/<app_folder>/
#
# The workspace build then installs its params/api/rui files to the places
# apps_mgr and the RUI expect (see APP_ARCHITECTURE.md Section on install).
#
# The script requires the following environment variable be set
#    NEPI_REMOTE_SETUP: Indicates whether running from development host or directly on target
#                      (1 = Dev. Host, 0 = From Target)
# In the case that NEPI_REMOTE_SETUP == 1, some further environment variables must be set
#    NEPI_TARGET_IP: Target IP address/hostname
     NEPI_TARGET_IP=${NEPI_IP} #/${NEPI_DEVICE_ID}
#    NEPI_TARGET_USERNAME: Target username
    nepihost=nepi
    if [[ "$NEPI_IN_CONTAINER" -eq 1 ]]; then
      nepihost=nepihost
    fi

     NEPI_TARGET_USERNAME=${nepihost}
#    NEPI_SSH_KEY: Private SSH key for SSH/Rsync to target (as applicable)
#    Honor an already-set NEPI_SSH_KEY / NEPI_SSH_KEY_PATH (the nepi env exports
#    the latter), otherwise fall back to the standard nepi key under ~/.ssh.
     NEPI_SSH_KEY=${NEPI_SSH_KEY:-${NEPI_SSH_KEY_PATH:-/home/${USER}/.ssh/nepi_default_ssh_key}}
#    NEPI_TARGET_SRC_DIR: Directory to deploy source code to
     NEPI_TARGET_SRC_DIR=/mnt/nepi_storage/nepi_src
#######################################################################################################

REPO_FOLDER=$(cd -P "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)

APPS_TARGET_PATH=${NEPI_TARGET_SRC_DIR}/nepi_engine_ws/src/nepi_apps

# App folders from args, or every nepi_app_* folder next to this script
if [ "$#" -gt 0 ]; then
  APP_FOLDERS="$@"
else
  APP_FOLDERS=$(cd "${REPO_FOLDER}" && ls -d nepi_app_*/ 2>/dev/null | tr -d '/')
fi

if [ -z "${APP_FOLDERS}" ]; then
  echo "No nepi_app_* folders found next to this script"
  exit 1
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
  if [[ -z "${NEPI_TARGET_USERNAME}" ]]; then
    echo "Remote setup requires env. variable NEPI_TARGET_USERNAME be assigned"
    exit 1
  fi
  if [[ -z "${NEPI_SSH_KEY}" ]]; then
    echo "Remote setup requires env. variable NEPI_SSH_KEY be assigned"
    exit 1
  fi
fi


## Sync update remote clock if needed
## sshnhc is a shell function from ~/.nepi_remote_aliases; it is NOT exported, so
## a child script only sees it if the caller has run `export -f sshnhc`. Guard the
## call so a missing helper doesn't abort/spam the deploy.
echo "Syncing remote clock if needed"
if [ "${NEPI_REMOTE_SETUP}" == "1" ]; then
  if command -v sshnhc >/dev/null 2>&1; then
    sshnhc
  else
    echo "  Skipping clock sync: 'sshnhc' not defined in this shell (source ~/.nepi_remote_aliases or 'export -f sshnhc' to enable)"
  fi
fi


# NOTE: filters MUST be a quoted array -- an unquoted "*" in a string variable
# gets glob-expanded by the shell into extra rsync source arguments.
RSYNC_FILTERS=(--exclude '.git' --exclude '.gitmodules' --exclude '__pycache__' --exclude '*.pyc')
echo "Using rsync filters: ${RSYNC_FILTERS[*]}"

for APP_FOLDER in ${APP_FOLDERS}; do

  APP_SOURCE_PATH=${REPO_FOLDER}/${APP_FOLDER}

  if [ ! -d "${APP_SOURCE_PATH}" ]; then
    echo "Skipping ${APP_FOLDER}: source folder not found: ${APP_SOURCE_PATH}"
    continue
  fi

  echo "Syncing app ${APP_FOLDER} from ${APP_SOURCE_PATH} to ${APPS_TARGET_PATH}/${APP_FOLDER}"

  if [ "${NEPI_REMOTE_SETUP}" == "0" ]; then
    rsync -avrh "${RSYNC_FILTERS[@]}" "${APP_SOURCE_PATH}/" "${APPS_TARGET_PATH}/${APP_FOLDER}/"

  elif [ "${NEPI_REMOTE_SETUP}" == "1" ]; then
    rsync -avzhe "ssh -i ${NEPI_SSH_KEY} -o StrictHostKeyChecking=no" "${RSYNC_FILTERS[@]}" "${APP_SOURCE_PATH}/" "${NEPI_TARGET_USERNAME}@${NEPI_TARGET_IP}:${APPS_TARGET_PATH}/${APP_FOLDER}/"

  fi

done
