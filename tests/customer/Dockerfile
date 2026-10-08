FROM ubuntu:24.04
ARG SOURCE_COMMIT
ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8
SHELL ["/bin/bash", "-ec"]
# The README links to the ROS Debian installation instructions before its commands.
RUN test ! -d /opt/ros; apt-get update; \
    apt-get install -y sudo ca-certificates curl git python3 software-properties-common; \
    add-apt-repository -y universe; \
    ros_apt_version="$(curl -fsSL --retry 3 https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')"; \
    curl -fsSL --retry 3 -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_version}/ros2-apt-source_${ros_apt_version}.noble_all.deb"; \
    dpkg -i /tmp/ros2-apt-source.deb; apt-get update; apt-get upgrade -y; \
    apt-get install -y ros-jazzy-ros-base; \
    useradd -m -u 1001 -s /bin/bash customer; \
    echo 'customer ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/customer
COPY --chown=customer:customer . /opt/lekiwi-source
USER customer
ENV USER=customer SOURCE_COMMIT=${SOURCE_COMMIT}
WORKDIR /home/customer
RUN /usr/bin/python3 /opt/lekiwi-source/tests/customer/readme_commands.py
# Only the test driver needs a virtual display and synthetic keyboard events.
RUN sudo apt-get install -y xvfb xdotool x11-utils; sudo rm -rf /var/lib/apt/lists/*
