#!/bin/bash
set -e

sudo service ssh start

# Enable any ssh key that can ssh into machine to be able to ssh into dev container
if [ -f /home/"$USER_HOST"/.ssh/authorized_keys ]; then
  cp /home/"$USER_HOST"/.ssh/authorized_keys /home/"$USER_DOCKER"/.ssh/authorized_keys
fi

# Allow local machine to ssh into dev container with existing key
if [ -f /home/"$USER_HOST"/.ssh/id_rsa.pub ]; then
  cat /home/"$USER_HOST"/.ssh/id_rsa.pub >> /home/"$USER_DOCKER"/.ssh/authorized_keys
elif [ -f  /home/"$USER_HOST"/.ssh/id_ed25519.pub ]; then
  cat /home/"$USER_HOST"/.ssh/id_ed25519.pub >> /home/"$USER_DOCKER"/.ssh/authorized_keys
fi
