#!/bin/bash

conda activate py37

# setting the env
export CODE_FOLDER=/home/kin/workspace/carla-expert
export CARLA_ROOT=/home/kin/CARLA_0.9.10.1
export SCENARIO_RUNNER_ROOT=${CODE_FOLDER}/scenario_runner
export LEADERBOARD_ROOT=${CODE_FOLDER}/leaderboard
export PYTHONPATH="/home/kin/CARLA_0.9.10.1/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${LEADERBOARD_ROOT}":"${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg":"${CODE_FOLDER}/team_code":${PYTHONPATH}

data_collect () {
  python data_collect.py carla_sh_path=${CARLA_ROOT}/CarlaUE4.sh \
  absolute_path=${CODE_FOLDER} \
}

# resume benchmark in case carla is crashed.
RED=$'\e[0;31m'
NC=$'\e[0m'
PYTHON_RETURN=1
until [ $PYTHON_RETURN == 0 ]; do
  python data_collect.py carla_sh_path=${CARLA_ROOT}/CarlaUE4.sh
  PYTHON_RETURN=$?
  echo "${RED} PYTHON_RETURN=${PYTHON_RETURN}!!! Start Over!!!${NC}" >&2
  sleep 2
done

killall -9 -r CarlaUE4-Linux
echo "Bash script done."