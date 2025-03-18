FROM osrf/ros:humble-desktop-full AS cacher

WORKDIR /ws/src

COPY . project/

RUN mkdir -p /tmp/ws/src \
 && find ./ -name "package.xml" | xargs cp --parents -t /tmp/ws/src \
 && find ./ -name "COLCON_IGNORE" | xargs cp --parents -t /tmp/ws/src \
 || true

FROM osrf/ros:humble-desktop-full

# Install base system dependencies and development dependencies
RUN apt update \
 && apt install -y \
    apt-utils \
    bash-completion \
    ccache \
    clang \
    debian-archive-keyring \
    debian-keyring \
    gdb \
    locate \
    mc \
    nano \
    openssh-server \
    python-is-python3 \
    python3-pip \
    screen \
    software-properties-common \
    sudo \
    tmux \
    wget \
    xterm \
 && rm -rf /var/lib/apt/lists/*

ARG USER=developer
ARG GROUP=developer

ARG USERID=1000
ARG GROUPID=1000

ENV USER $USER
ENV GROUP $GROUP

RUN addgroup --gid $GROUPID $GROUP \
 && adduser --uid $USERID --ingroup $GROUP --home /home/$USER --shell /bin/sh --disabled-password --gecos "" $USER \
 && adduser $USER sudo \
 && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

RUN curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.6.0/fixuid-0.6.0-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - \
 && chown root:root /usr/local/bin/fixuid \
 && chmod 4755 /usr/local/bin/fixuid \
 && mkdir -p /etc/fixuid \
 && printf "user: $USER\ngroup: $GROUP\n" > /etc/fixuid/config.yml


USER $USER

ENV USER_WS /home/${USER}/ws
RUN mkdir -p $USER_WS/src

WORKDIR $USER_WS

#
# install a colcon mixin that will use ccache for building C++ code
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
 && colcon mixin update default
COPY --chown=$USER:$GROUP docker/files/colcon_defaults.yaml /home/$USER/.colcon/defaults.yaml

#
# Install base system external dependencies
COPY docker/files/dependencies.repos /tmp/dependencies.repos
RUN cd src/ \
 && mkdir -p external \
 && cd external \
 && vcs import < /tmp/dependencies.repos

#
# Install Mujoco, no packages available
ENV MUJOCO_VERSION "3.2.7"
RUN sudo wget https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz \
 && sudo tar -C /opt -xzf mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz \
 && sudo rm mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz \
 && sudo apt update \
 && sudo apt install -y libglfw3-dev \
 && sudo rm -rf /var/lib/apt/lists/ \
 && pip install mujoco \
 && pip install urdf2mjcf
ENV MUJOCO_DIR /opt/mujoco-${MUJOCO_VERSION}
ENV LD_LIBRARY_PATH $MUJOCO_DIR/bin:${LD_LIBRARY_PATH}
ENV PATH $MUJOCO_DIR/bin:${PATH}

#
# Install ROS project dependencies by copying the package.xml files (but not the source code,
# to avoid rebuilding the workspace every time the source code changes) and running rosdep install
COPY --from=cacher --chown=$USER:$GROUP /tmp/ws/ $USER_WS/
RUN sudo apt update \
 && . /opt/ros/humble/setup.sh \
 && rosdep update \
 && rosdep install -i -y --from-path src \
 && sudo rm -rf /var/lib/apt/lists/*

#
# setup a bashrc so that an environment is sourced whenever the developer enters
# the container by running or attaching to it
COPY docker/files/developer_rc /home/developer/.developer_rc
RUN mkdir -p $USER_WS/persistence \
 && chmod 777 $USER_WS/persistence \
 && echo "source /home/developer/.developer_rc" >> /home/developer/.bashrc

ENV WITHIN_DEV 1

ENTRYPOINT ["fixuid", "-q", "/ros_entrypoint.sh", "bash"]
