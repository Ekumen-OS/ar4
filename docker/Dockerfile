FROM osrf/ros:humble-desktop-full

# Install base system dependencies and development dependencies
RUN --mount=type=cache,target=/var/cache/apt \
    apt update \
 && apt install -y \
    apt-utils \
    bash-completion \
    clang \
    debian-archive-keyring \
    debian-keyring \
    gdb \
    locate \
    mc \
    openssh-server \
    python3-pip \
    screen \
    software-properties-common \
    sudo \
    tmux

# Set user and fixuid configuration
ARG USER=developer
ARG GROUP=ekumen

ARG USERID=1000
ARG GROUPID=1000

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

# Install ros dependencies, updating the apt cache is necessary.
RUN --mount=type=bind,source=.,target=/tmp/ws \
    sudo apt-get update \
 && . /opt/ros/humble/setup.sh \
 && rosdep update \
 && rosdep install --from-paths /tmp/ws --ignore-src -r -y

WORKDIR /workspace
ENV SHELL /bin/bash
ENTRYPOINT ["fixuid", "-q", "/ros_entrypoint.sh", "/bin/bash"]
