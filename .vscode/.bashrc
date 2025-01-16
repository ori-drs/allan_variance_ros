# Get the directory containing this script, assuming this script is being sourced
SCRIPT_DIR=$( dirname "${BASH_SOURCE[0]}" )

# Include .bashrc if it exists
if [ -f $HOME/.bashrc ]; then
    . $HOME/.bashrc
fi

# Include .bashrc if it exists
if [ -f $SCRIPT_DIR/user.bashrc ]; then
    . $SCRIPT_DIR/user.bashrc
fi

# Provide git/bazel/etc.. completion for everyone
# source /usr/share/bash-completion/bash_completion

source /opt/ros/noetic/setup.bash