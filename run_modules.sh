#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

${SCRPIPT_DIR}/venv/bin/python ${SCRIPT_DIR}/run_modules.py $@

