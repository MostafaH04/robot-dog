FROM ros:jazzy-ros-base-noble@sha256:567b81bc54f44479e16ef1b75e4984d132f154b6511ea4fc851ee6bde76c30f8

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG USERNAME=ubuntu

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    ROS_DISTRO=jazzy \
    VIRTUAL_ENV=/opt/robot-dog-venv

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      git \
      make \
      python3-colcon-common-extensions \
      python3-pip \
      python3-rosdep \
      python3-scipy \
      python3-venv \
      sudo \
      ros-jazzy-foxglove-bridge \
      ros-jazzy-joint-state-publisher \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-rviz2 \
      ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

RUN echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}"

COPY requirements.txt /tmp/robot-dog-requirements.txt

RUN python3 -m venv --system-site-packages "${VIRTUAL_ENV}" \
    && "${VIRTUAL_ENV}/bin/python" -m pip install --no-cache-dir --upgrade pip==25.1.1 \
    && "${VIRTUAL_ENV}/bin/python" -m pip install --no-cache-dir -r /tmp/robot-dog-requirements.txt

ENV PATH="${VIRTUAL_ENV}/bin:${PATH}"

USER ${USERNAME}
WORKDIR /workspaces/robot-dog

RUN echo 'source /opt/ros/jazzy/setup.bash' >> "/home/${USERNAME}/.bashrc"
