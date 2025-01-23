FROM osrf/ros:humble-desktop-full

# Arguments for building
ARG USERID
ARG USER

# Install base system dependencies and development dependencies.
RUN apt update && apt install -y sudo openssh-server software-properties-common \
   debian-keyring debian-archive-keyring apt-utils python3-pip bash-completion tmux

# Install clang.
RUN apt update && apt install -y clang lldb lld

# Setup environment.
ENV TERM linux
ENV DEBIAN_FRONTEND noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
RUN apt install -y locales
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
        apt-utils \
        curl \
        software-properties-common \
        wget \
    && \
    apt-get clean && \
    apt-get autoremove && \
    apt-get autoclean

# Copy installation scripts, and perform the dependency installation.
COPY docker/install.sh .
COPY docker/requirements_ros.txt .
RUN ./install.sh
RUN rm install.sh requirements_ros.txt

# Create a user with passwordless sudo.
RUN adduser --uid $USERID --gecos "AR4 Developer" --disabled-password $USER
RUN adduser $USER sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo "export QT_X11_NO_MITSHM=1" >> /home/$USER/.bashrc

RUN apt-get update

# Switch to your user.
USER $USER

# Creates the src folder of the workspace.
RUN mkdir -p /home/$USER/ws/src

# Configure the workspace.
WORKDIR /home/$USER/ws

CMD ["/bin/bash"]
