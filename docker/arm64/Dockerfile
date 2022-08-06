FROM arm64v8/ros:galactic-ros-base


# Installing all the required libs
RUN apt-get update -y; \
    apt-get install -y \ 
    ros-galactic-imu-filter-madgwick \ 
    screen

RUN apt install -y python3-pip; \
    pip3 install gpiozero pigpio; \
    pip3 install pyserial


