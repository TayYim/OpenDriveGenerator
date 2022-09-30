#! /bin/bash

# ==============================================================================
# -- Validate topology of the xodr map -----------------------------------------------------------
# ==============================================================================

ESMINI_PATH="/home/tay/Applications/esmini"
OPENDRIVE_PATH=$ESMINI_PATH"/output.xodr"
PythonAPI_PATH=$CARLA_ROOT"/PythonAPI/util"
SCRIPTS_PATH="/home/tay/Workspace/sbst-carla/leaderboard/scripts"

# generate xodr file
/usr/bin/python3 generate_parampoly3_road.py

# import to carla
/usr/bin/python3 ${PythonAPI_PATH}/config.py -x ${OPENDRIVE_PATH}

# test topology
/usr/bin/python3 ${SCRIPTS_PATH}/get_topology.py