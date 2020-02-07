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
BASEDIR=$(dirname $0)
TEMPLATES_FOLDER=${BASEDIR}/templates/models

function usage(){
    echo "usage: ./gen_model_from_template.sh TEMPLATE_NAME"
    echo "Input template not found in ${TEMPLATES_FOLDER}"
    echo "Options are:"
    for FILENAME in $(ls ${TEMPLATES_FOLDER});
    do
        MODEL_NAME=$(echo "$FILENAME" | cut -f 1 -d '.')
        echo " - ${MODEL_NAME}"
    done    
}

TEMPLATE_FILE=$1

if [ ! -f ${TEMPLATES_FOLDER}/${TEMPLATE_FILE}.sdf.jinja ]; then
    usage
else
    MODEL_NAME=$(echo "$TEMPLATE_FILE" | cut -f 1 -d '.')
    echo "Processing model template=${TEMPLATES_FOLDER}/${TEMPLATE_FILE}.sdf.jinja"
    echo "Model name=${MODEL_NAME}"

    if [[ -w ${HOME} ]]; then
        MODELS_FOLDER=${HOME}/.pcg/models
    else
        MODELS_FOLDER=/tmp/.pcg/models
    fi

    if [ ! -d ${MODELS_FOLDER}/${MODEL_NAME} ]; then
        mkdir -p ${MODELS_FOLDER}/${MODEL_NAME}
    fi

    pcg-process-jinja-template \
        -i '$(PCG)/model.config.jinja' \
        -o ${MODELS_FOLDER}/${MODEL_NAME}/model.config \
        -p model_name=${MODEL_NAME} \
        -p version=1.0 \
        -p sdf_version=1.6 \
        -p author_name=$(whoami) \
        -p author_email=$(whoami)@email.com \
        -p sdf_filename=model.sdf

    pcg-process-jinja-template \
        -i ${TEMPLATES_FOLDER}/${TEMPLATE_FILE}.sdf.jinja \
        -o ${MODELS_FOLDER}/${MODEL_NAME}/model.sdf

    echo "${MODEL_NAME} generated in ${MODELS_FOLDER}/${MODEL_NAME}"
fi
