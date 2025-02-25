FROM osrf/ros:humble-desktop-full

RUN apt update \
 && apt install --no-install-recommends -y \
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
    tmux \
 && rm -rf /var/lib/apt/lists/*

ARG UID=1000
ARG GID=1000
ARG USER=devel
ARG GROUP=devel

RUN addgroup --gid $GID $GROUP \
 && adduser --uid $UID --ingroup $GROUP --home /home/$USER --shell /bin/sh --disabled-password --gecos "" $USER \
 && adduser $USER sudo \
 && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

RUN curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.6.0/fixuid-0.6.0-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - \
 && chown root:root /usr/local/bin/fixuid \
 && chmod 4755 /usr/local/bin/fixuid \
 && mkdir -p /etc/fixuid \
 && printf "user: $USER\ngroup: $GROUP\n" > /etc/fixuid/config.yml

USER $USER

# install the ros dependencies of the packages in the repository
RUN --mount=type=bind,source=.,target=/tmp/ws \
    sudo apt update \
 && . /opt/ros/humble/setup.sh \
 && rosdep update \
 && rosdep install --from-paths /tmp/ws --ignore-src -y \
 && sudo rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
ENV SHELL /bin/bash
ENTRYPOINT ["fixuid", "-q", "/ros_entrypoint.sh", "/bin/bash"]
