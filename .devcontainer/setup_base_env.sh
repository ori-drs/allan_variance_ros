#!/bin/bash

# Exit on failures
set -e

USER_DOCKER="local"
HOME_DOCKER="/home/${USER_DOCKER}"

# fix .gitconfig causing issues when the file doesn't exist before creating devcontainer
GITCONFIG=${HOME}/.gitconfig

if [ -d "$GITCONFIG" ]; then
    # Directory exists, remove directory
    rm -rf $GITCONFIG
fi

if [ ! -f "$GITCONFIG" ]; then
    # Check if file doesn't exist, then create one
    touch $GITCONFIG
fi

# Get the workspace path without expanding symlinks
WORKSPACE_PATH=$(pwd)/$(git rev-parse --show-cdup)
WORKSPACE_BASENAME=$(basename $WORKSPACE_PATH)
WORKSPACE_DOCKER="${HOME_DOCKER}/catkin_ws/src/${WORKSPACE_BASENAME}"

DEVCONTAINER_PATH=${DEVCONTAINER_PATH:=${WORKSPACE_PATH}/.devcontainer}
USER_CONFIG_PATH=$DEVCONTAINER_PATH/user_config
DOCKERFILE_BUILD="Dockerfile.devcontainer"

# Load the user config for the container, which could have the following options:
# ADDITIONAL_BUILD_APT_DEPS - additional APT packages to install when building the container
# HOST_HOME_MOUNT_PATH - location in the container to mount the user's home folder from the host machine, defaults to /home/$USER/host_home
# DOCKERFILE_BUILD - target an alternate Dockerfile
if [ -f $USER_CONFIG_PATH ]; then
    . $USER_CONFIG_PATH
fi

# Create the .env file
ENV_TARGET=${DEVCONTAINER_PATH}/.env
rm -f $ENV_TARGET

# Add additional variables needed in the target environment
echo "DOCKERFILE_BUILD=${DOCKERFILE_BUILD}" >> $ENV_TARGET
echo "WORKSPACE=$WORKSPACE_PATH" >> $ENV_TARGET
echo "WORKSPACE_DOCKER=$WORKSPACE_DOCKER" >> $ENV_TARGET
echo "WORKSPACE_BASENAME=${WORKSPACE_BASENAME}" >> $ENV_TARGET
echo "HOST_HOME_MOUNT_PATH=${HOST_HOME_MOUNT_PATH:=/home/$USER/host_home}" >> $ENV_TARGET
echo "DEVCONTAINER_PATH=$DEVCONTAINER_PATH" >> $ENV_TARGET
echo "HOME_DOCKER=${HOME_DOCKER}" >> $ENV_TARGET
echo "USER_DOCKER=${USER_DOCKER}" >> $ENV_TARGET

# Define which nvidia drivers/libs to load into the container
# See https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/1.10.0/user-guide.html#driver-capabilities
echo "NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:=}" >> $ENV_TARGET

# Set env variable for netrc file
NETRC_FILE=${NETRC_FILE:="${HOME}/.netrc"}
NETRC_FILE_DOCKER=${NETRC_FILE_DOCKER:="${HOME_DOCKER}/.netrc"}
if [ ! -f "${NETRC_FILE}" ]; then
    NETRC_FILE=/dev/null
    NETRC_FILE_DOCKER="${HOME_DOCKER}/.netrc-null"
fi

echo "NETRC_FILE=$NETRC_FILE" >> $ENV_TARGET
echo "NETRC_FILE_DOCKER=$NETRC_FILE_DOCKER" >> $ENV_TARGET

# Set env variable for ssh file
SSH_FILE_NAME=${SSH_FILE_NAME}
SSH_FILE=${SSH_FILE:-"${HOME}/.ssh/${SSH_FILE_NAME}"}
SSH_FILE_DOCKER=${SSH_FILE_DOCKER:-"${HOME_DOCKER}/.ssh/${SSH_FILE_NAME}"}

if [ ! -f {$SSH_FILE} ]; then
  SSH_FILE=/dev/null
  SSH_FILE_DOCKER="${HOME_DOCKER}/.ssh/ssh.null"

  for key_name in "id_rsa" "id_ed25519"
  do
    TEMP_SSH_FILE="${HOME}/.ssh/${key_name}"

    if [ -f "$TEMP_SSH_FILE" ]; then
      export SSH_FILE_NAME="${key_name}"
      export SSH_FILE="${TEMP_SSH_FILE}"
      export SSH_FILE_DOCKER="${HOME_DOCKER}/.ssh/${SSH_FILE_NAME}"
    fi
  done
fi

echo "SSH_FILE=$SSH_FILE" >> $ENV_TARGET
echo "SSH_FILE_DOCKER=$SSH_FILE_DOCKER" >> $ENV_TARGET

# Set env variable for docker config dir
DOCKER_DIR=${DOCKER_DIR:="${HOME}/.docker"}
DOCKER_DIR_DOCKER=${DOCKER_DIR_DOCKER:="${HOME_DOCKER}/.docker"}
if [ ! -d "${DOCKER_DIR}" ]; then
    DOCKER_DIR=/dev/null
    DOCKER_DIR_DOCKER="${HOME_DOCKER}/.docker-null"
fi

echo "DOCKER_DIR=$DOCKER_DIR" >> $ENV_TARGET
echo "DOCKER_DIR_DOCKER=$DOCKER_DIR_DOCKER" >> $ENV_TARGET

# Add the user's ID to the environment
echo "CURRENT_UID=$(id -u):$(id -g)" >> $ENV_TARGET

# Ensure the .vscode-server folder exists
VSCODE_SERVER_PATH=${DEVCONTAINER_PATH}/.vscode-server
if [ ! -d $VSCODE_SERVER_PATH ]; then
    mkdir $VSCODE_SERVER_PATH
else
    # Remove the machine settings file to ensure the latest is used from the devcontainer.json
    rm -f $VSCODE_SERVER_PATH/data/Machine/settings.json
    rm -f $VSCODE_SERVER_PATH/data/Machine/.writeMachineSettingsMarker

    # Remove the install extensions marker file to ensure the extensions in the devcontainer.json are installed
    rm -f $VSCODE_SERVER_PATH/data/Machine/.installExtensionsMarker
fi

SSH_PORT=${SSH_PORT:-8888} # Can override in user_config
echo "SSH_PORT=${SSH_PORT}" >> $ENV_TARGET
