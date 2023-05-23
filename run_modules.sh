#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# setup our python virtual environment
. ${SCRIPT_DIR}/venv/bin/activate

python ${SCRIPT_DIR}/run_modules.py $@

# deactivate - if we get here
deactivate
