FROM ubuntu:18.04
MAINTAINER Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>

COPY . /tmp/pcg_gazebo

RUN apt-get update && \
    apt-get install \
    python3-pip \
    libspatialindex-dev \
    pybind11-dev -y

RUN pip3 install -r /tmp/pcg_gazebo/requirements.txt

WORKDIR /tmp/pcg_gazebo
RUN python3 setup.py install

RUN python3 -c "import pcg_gazebo"