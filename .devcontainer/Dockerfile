FROM mostafah04/mobile_robot:latest

RUN sudo apt install software-properties-common 
RUN sudo add-apt-repository universe

RUN sudo apt install ros-humble-xacro -y && \
  sudo apt install ros-humble-joint-state-publisher -y && \
  sudo apt install pip -y

RUN pip install pybullet && pip install numpy==1.23