#!/bin/bash
apt-get update -y  && \
    apt-get install -y software-properties-common && \
    add-apt-repository ppa:deadsnakes/ppa -y && \
    apt-get install -y ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-velocity-controllers ros-melodic-ros-numpy && \
    apt-get install -y python3.7 python3-setuptools python3-pip python-dev python3-dev python3.7-dev && \
    apt-get install -y build-essential libssl-dev libffi-dev libxml2-dev libxslt1-dev zlib1g-dev libblas-dev libatlas-base-dev && \
    apt-get -y --only-upgrade install ros-* && \

# Python
unlink /usr/bin/python && \
    unlink /usr/bin/python3 && \
    unlink /usr/bin/python-config && \
    ln -s /usr/bin/python3.7 /usr/bin/python3 && \
    ln -s /usr/bin/python3.7 /usr/bin/python && \
    ln -s /usr/bin/python3.7-config /usr/bin/python-config && \
    pip3 install --upgrade pip && \

# Tensorflow API
mkdir /ROS/TensorFlow && \
    cd /ROS/TensorFlow && \
    git clone https://github.com/tensorflow/models.git && \
    cd /ROS && \

# ProtoBuf
export PATH="$PATH:/ROS/protobuf"  && \
    mkdir /ROS/protobuf && \
    cd /ROS/protobuf && \
    wget https://github.com/protocolbuffers/protobuf/releases/download/v3.14.0/protoc-3.14.0-linux-x86_64.zip && \
    unzip protoc-3.14.0-linux-x86_64.zip && \
    cd /ROS/TensorFlow/models/research && \
    protoc object_detection/protos/*.proto --python_out=. && \

# Coco API
    cd /ROS && \
    git clone https://github.com/cocodataset/cocoapi.git && \
    cd /ROS/cocoapi/PythonAPI && \
    pip3 install numpy cython && \
    make && \

# Tensorflow Object Detection API
cd /ROS/TensorFlow/models/research/ && \
    cp object_detection/packages/tf2/setup.py . && \
    python -m pip install . --use-deprecated legacy-resolver && \
    python object_detection/builders/model_builder_tf2_test.py && \

# Reinstall caktin_make for python3.7
pip3 install catkin_tools rospkg defusedxml netifaces && \

# Install python packages
cd /ROS && \
    pip install -r requirements.txt

#catkin_make
#source devel/setup.bash
