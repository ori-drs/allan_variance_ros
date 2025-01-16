# How to use a devcontainer for development

1. Install the [Remote - Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension in VSCode
2. Install Docker on local machine (https://docs.docker.com/desktop/setup/install/linux/ubuntu/)
3. Open the allan_variance_ros repository in VSCode
4. Reopen in Container: Use the Command Palette (Ctrl+Shift+P or Cmd+Shift+P on macOS) and select "Remote-Containers: Reopen in Container". This will build the container using the Dockerfile and open your project inside the container
5. Run `catkin build allan_variance_ros` to build the package
6. Run `rosrun allan_variance_ros allan_variance /path/to/rosbag.bag /path/to/config.yaml` to run the tool

Once the repository is open in the container, you can develop as normal following the instructions in the [README.md](../README.md) file.
