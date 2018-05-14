FROM osrf/ros:lunar-desktop-full
RUN apt-get update && apt-get install -y gdb nano
