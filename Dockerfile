# Use the official ROS Melodic base image
FROM osrf/ros:melodic-desktop-full

# Set the working directory
WORKDIR /workspace

# Install additional dependencies if needed
# For example, you can uncomment the line below to install a package
# Install additional dependencies
RUN apt-get update \
    && apt-get -y --quiet --no-install-recommends install \
    gcc \
    python \
    python-pip\
    libeigen3-dev\
    libboost-all-dev\
    libx11-dev\
    curl \
    libboost-dev python-dev python-numpy cmake cmake-curses-gui\
    g++ cython liboctave-dev freeglut3-dev\
    python-tk

# Install bayes optimization
RUN git clone https://github.com/zhaozhongch/bayesopt.git

WORKDIR /workspace/bayesopt/build

RUN cmake ..

RUN make

RUN make install

# Install nvdia drivers
RUN curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
    
RUN apt-get update && apt-get -y --quiet --no-install-recommends install nvidia-container-toolkit

RUN pip install setuptools
RUN pip install catkin-tools 

# Copy your ROS packages into the workspace
COPY . /workspace/src/

# RUN catkin build
WORKDIR /workspace

RUN . /opt/ros/melodic/setup.sh && catkin_make

# Source the ROS setup file
RUN echo "source /workspace/devel/setup.bash" >> ~/.bashrc

# Expose ROS master port
EXPOSE 11311

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Set entry point to start ROS
CMD ["bash"]
