FROM ubuntu:24.04

ARG USERID=1000
ARG GROUPID=1000

ARG USER=developer
ARG GROUP=developer

ENV USER=$USER
ENV GROUP=$GROUP

RUN apt update \
 && apt install -y \
    apt-utils \
    bash-completion \
    locate \
    mc \
    nano \
    python-is-python3 \
    python3 \
    python3-pip \
    python3-venv \
    sudo \
 && rm -rf /var/lib/apt/lists/*

RUN userdel ubuntu
RUN addgroup --gid $GROUPID $GROUP \
 && adduser --uid $USERID --ingroup $GROUP --home /home/$USER --shell /bin/sh --disabled-password --gecos "" $USER \
 && adduser $USER sudo \
 && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

ARG HOME=/home/$USER

USER ${USER}

WORKDIR $HOME

COPY docker/requirements.txt /tmp/requirements.txt
RUN python3 -m venv .venv \
 && . .venv/bin/activate \
 && pip install -r /tmp/requirements.txt

COPY docker/process_urdf.sh $HOME

COPY docker/entrypoint.sh /entrypoint.sh

CMD ["./process_urdf.sh"]

ENTRYPOINT ["/entrypoint.sh"]
