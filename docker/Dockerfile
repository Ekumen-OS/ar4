FROM osrf/ros:humble-desktop-full

# Arguments for building
ARG USERID
ARG USERNAME

# Install base system dependencies and development dependencies.
RUN --mount=type=cache,target=/var/cache/apt \
    apt update && apt install -y \
    apt-utils \
    bash-completion \
    clang \
    debian-archive-keyring \
    debian-keyring \
    python3-pip \
    software-properties-common \
    sudo \
    tmux \
    openssh-server

# Create a user with passwordless sudo.
RUN adduser --uid $USERID --gecos "AR4 Developer" --disabled-password $USERNAME && \
    adduser $USERNAME sudo && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Switch to your user.
USER $USERNAME

RUN --mount=type=bind,source=.,target=/tmp/ws \
    . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths /tmp/ws --ignore-src -r -y

# Configure the workspace.
WORKDIR /home/$USERNAME/workspace

CMD ["/bin/bash"]
