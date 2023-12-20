FROM ubuntu:18.04

#ARG DEBIAN_FRONTEND=noninteractive

ENV TZ=America/New_York
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

#Build Python 3.11
RUN apt-get update -y \
    && apt-get upgrade -y \
    && apt-get -y install build-essential \
        zlib1g-dev \
        libncurses5-dev \
        libgdbm-dev \ 
        libnss3-dev \
        libssl-dev \
        libreadline-dev \
        libffi-dev \
        libsqlite3-dev \
        libbz2-dev \
        wget \
    && export DEBIAN_FRONTEND=noninteractive \
    && apt-get purge -y imagemagick imagemagick-6-common 

RUN cd /usr/src \
    && wget https://www.python.org/ftp/python/3.11.0/Python-3.11.0.tgz \
    && tar -xzf Python-3.11.0.tgz \
    && cd Python-3.11.0 \
    && ./configure --enable-optimizations \
    && make altinstall

RUN update-alternatives --install /usr/bin/python3 python3 /usr/local/bin/python3.11 1

################### PYTHON ########################
#RUN : \
#    && apt-get update \
#    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
#        software-properties-common \
#    && add-apt-repository -y ppa:deadsnakes \
#    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
#        python3.11 \
#    && apt-get clean \
#    && rm -rf /var/lib/apt/lists/* \
#    && :

RUN python3.11 -m venv /venv
ENV PATH=/venv/bin:$PATH

# Get Python 3
RUN apt-get update && apt-get install -y \
    netcat \
    ntp \
    python2.7 \
    python3 \
    python3-dev \
    libssl-dev \
    libgeos-dev \
    python3-pip \
    python3-yaml \
    python3-setuptools \
    python3-rospkg \
    python-pip && \
    python3 -m pip install --upgrade pip && \
    python2.7 -m pip install --upgrade pip && \
    python2.7 -m pip install --upgrade paho-mqtt && \
    python3 -m pip install --upgrade Pillow


# Install Python packages
RUN apt-get update && \
    python3 -m pip install \
    gps3 \
    paho-mqtt \
    pyserial \
    pygatt \
    bokeh \
    pandas \
    matplotlib \
    pexpect \
    utm \
    geopy \
    flask \
    pandas \
    shapely \ 
    waitress

# Install Common Tools and Required Software
RUN apt-get update && apt-get install -y \
    curl \
    openjdk-11-jdk-headless \
    software-properties-common \
    iproute2 \
    net-tools \
    vim \
    nano \
    bc \
    ssh \
    socat \
    wget \
    screen \
    sudo \
    unzip \
    gpg-agent


# Setup the environment
RUN ln -snf /usr/share/zoneinfo/$CONTAINER_TIMEZONE /etc/localtime && echo $CONTAINER_TIMEZONE > /etc/timezone && \
    mkdir /opt/gradle && \
    mkdir home/mace/


# Install ROS melodic and nodes for optional ROS use
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && apt-get install -y \
    ros-melodic-move-base \
    ros-melodic-move-base-msgs \
    ros-melodic-ros-base \
    ros-melodic-rosbridge-suite \
    ros-melodic-gps-common \
    ros-melodic-tf2-sensor-msgs \
    ros-melodic-geographic-msgs && \
    python -m pip install \
    utm \
    websocket \
    websocket-client \
    paho-mqtt


################## MOSQUITTO ######################

# Install Mosquitto
RUN apt-add-repository ppa:mosquitto-dev/mosquitto-ppa && \
    apt-get update && apt-get install -y \
    mosquitto \
    mosquitto-clients && \
    apt clean


################### GRADLE ######################

# Install Gradle
RUN curl -L https\://services.gradle.org/distributions/gradle-7.1.1-bin.zip -o gradle-7.1.1-bin.zip && \
    apt-get update && apt-get upgrade -y && \
    unzip -d /opt/gradle/ gradle-7.1.1-bin.zip
ENV GRADLE_HOME=/opt/gradle/gradle-7.1.1/
ENV PATH=$PATH:$GRADLE_HOME/bin


################### MACE ########################

# Copy the MACE project to the container 
COPY /code /home/mace/code/
COPY /config /home/mace/config/
COPY /scripts /home/mace/scripts/
COPY /src /home/mace/src/
COPY README.md /home/mace/

# Copy relevant documentation
COPY /docs/development /docs/examples /docs/manuals /home/mace/docs/

# Modify the workdir to be mace
WORKDIR /home/mace/src/analytics_server/

# # Build the analytics server
RUN gradle build -x test

# Build ROS Nodes
WORKDIR /home/mace/code/stomp/ros_sim
# RUN rm -r devel_isolated build_isolated; exit 0
RUN mv /home/mace/code/stomp/ros_sim /root/rover

WORKDIR /root/rover
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make_isolated"

# Set the base workdir to be mace and update the remaining configs
WORKDIR /home/mace/
RUN cp scripts/docker/mosquitto.conf /etc/mosquitto/mosquitto.conf
RUN cp config/mosquitto/sim_mosquitto.conf /etc/mosquitto/sim_mosquitto.conf
RUN cp config/mosquitto/mace_mosquitto.conf /etc/mosquitto/mace_mosquitto.conf
#Use sim config for mosquitto
RUN cp /etc/mosquitto/sim_mosquitto.conf /etc/mosquitto/mosquitto.conf

#Setup MQTT Access Controls
RUN cp config/mosquitto/mosquitto_pass /etc/mosquitto/mosquitto_pass
RUN cp config/mosquitto/dynamic-security.json /var/lib/mosquitto/dynamic-security.json

#Setup MQTT SSL
RUN cp config/mosquitto/dynamic-security.json /var/lib/mosquitto/dynamic-security.json
RUN cp -r config/mosquitto/ssl/certs /etc/mosquitto/
RUN chown mosquitto /etc/mosquitto/certs/server.key
RUN cp -r config/mosquitto/ssl/ca_certificates /etc/mosquitto/


#Setup NTP
RUN cp config/ntp/ntp.conf /etc/ntp.conf
