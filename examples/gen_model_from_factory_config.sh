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
CONFIG_FOLDER=${BASEDIR}/model_factory

function usage(){
    echo "usage: ./gen_model_from_factory_config.sh CONFIG_NAME"
    echo "Input template not found in ${CONFIG_FOLDER}"
    echo "Options are:"
    for FILENAME in $(ls ${CONFIG_FOLDER});
    do
        MODEL_NAME=$(echo "$FILENAME" | cut -f 1 -d '.')
        echo " - ${MODEL_NAME}"
    done    
}

CONFIG_NAME=$1

if [ ! -f ${CONFIG_FOLDER}/${CONFIG_NAME}.yaml ]; then
    usage
else
    if [[ -w ${HOME} ]]; then
        MODELS_FOLDER=${HOME}/.pcg/models
    else
        MODELS_FOLDER=/tmp/.pcg/models
    fi

    if [ ! -d ${MODELS_FOLDER} ]; then
        mkdir -p ${MODELS_FOLDER}
        echo "Output directory ${MODELS_FOLDER} created"
    fi

    pcg-run-model-factory \
        --config-file ${CONFIG_FOLDER}/${CONFIG_NAME}.yaml \
        --store-dir ${MODELS_FOLDER} --store-model --overwrite    
fi
set +e