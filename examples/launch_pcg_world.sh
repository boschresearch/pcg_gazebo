#/bin/bash
# Copyright (c) 2019 - The Procedural Generation for Gazebo authors
# For information on the respective copyright owner see the NOTICE file
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
set -e
BASEDIR=$(dirname $0)
CONFIG_FOLDER=${BASEDIR}/world_generator/worlds

function usage(){
    echo "usage: ./launch_pcg_world.sh WORLD_CONFIG_NAME"
    echo "Input template not found in ${CONFIG_FOLDER}"
    echo "Options are:"
    for FILENAME in $(ls ${CONFIG_FOLDER});
    do
        WORLD_NAME=$(echo "$FILENAME" | cut -f 1 -d '.')
        echo " - ${WORLD_NAME}"
    done    
}

WORLD_NAME=$1

if [ ! -f ${CONFIG_FOLDER}/${WORLD_NAME}.yml ]; then
    usage
else
    if [ ! -d ${WORLDS_FOLDER} ]; then
        mkdir -p ${WORLDS_FOLDER}
    fi
    
    if [[ -w ${HOME} ]]; then
        WORLDS_FOLDER=${HOME}/.pcg/worlds
    else
        WORLDS_FOLDER=/tmp/.pcg/worlds
    fi

    if [ ! -d ${WORLDS_FOLDER} ]; then
        mkdir -p ${WORLDS_FOLDER}
    fi

    pcg-generate-pcg-world \
        --config-file ${CONFIG_FOLDER}/${WORLD_NAME}.yml \
        --output-world-file ${WORLDS_FOLDER}/${WORLD_NAME}.world

    if [ $(which roslaunch) ]; then 
        if [ -f ${WORLDS_FOLDER}/${WORLD_NAME}.world ]; then
            echo "Running Gazebo for world ${WORLDS_FOLDER}/${WORLD_NAME}.world"
            if echo "${WORLD_NAME}" | grep -q "ode"; then
                roslaunch gazebo_ros empty_world.launch world_name:=${WORLDS_FOLDER}/${WORLD_NAME}.world verbose:=true
            else
                roslaunch gazebo_ros empty_world.launch world_name:=${WORLDS_FOLDER}/${WORLD_NAME}.world verbose:=true physics:=bullet
            fi
        else
            echo "${WORLDS_FOLDER}/${WORLD_NAME}.world COULD NOT be generated!"
        fi
    else
        echo "roslaunch not available"
    fi
fi
set +e