FROM ubuntu:18.04
MAINTAINER Musa Morena Marcusso Manhaes <musa.marcusso@de.bosch.com>

RUN apt-get update && \
    apt-get install \
    python3-pip \
    libspatialindex-dev \
    pybind11-dev -y
    
COPY . /tmp/pcg_gazebo

WORKDIR /tmp/pcg_gazebo

RUN pip3 install -e .

RUN python3 -c "import pcg_gazebo"

RUN pip3 install -e .[test]

RUN pytest -x /tmp/pcg_gazebo/tests