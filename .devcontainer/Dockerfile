FROM ros:jazzy-perception

# Arguments for user creation
ARG USERNAME=robotics
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete existing user with same UID to avoid conflicts
RUN if id -u $USER_UID >/dev/null 2>&1; then userdel $(id -un $USER_UID); fi

# Create a non-root user with sudo access
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to the non-root user
USER $USERNAME

# Add user to video group (GUI apps, camera access) and input group (joystick/gamepad access)
RUN sudo groupadd --force input \
    && sudo usermod --append --groups video,input $USERNAME

# Set default shell
ENV SHELL=/bin/bash

# Update system and install common tools
RUN sudo apt-get update && sudo apt-get upgrade -y \
    && sudo apt-get install -y git python3-pip python3-colcon-common-extensions

# Source ROS environment automatically
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc

RUN PYTHONUSERBASE=$(ros2 pkg prefix py_venv) pip3 install --user --break-system-packages rsl-rl-lib pygame

