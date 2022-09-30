#! /bin/bash

# ==============================================================================
# -- Make map, convert, import and package -------------------------------------
# ==============================================================================

ESMINI_PATH="/home/tay/Applications/esmini"
OPENDRIVE_PATH=$ESMINI_PATH"/output.xodr"
PythonAPI_PATH=$CARLA_ROOT"/PythonAPI/util"
SCRIPTS_PATH="/home/tay/Workspace/sbst-carla/leaderboard/scripts"
CARLA_SB_PATH="/home/tay/Applications/CARLA_sb/carla_ril"
CARLA_DIST_PATH="/home/tay/Applications/CARLA_LB"

PACKAGE_NAME="Sep29"

start_time=$(date +%s)

# generate xodr file
/usr/bin/python3 generate_parampoly3_road.py

# generate osgb
## TODO: close window
${ESMINI_PATH}/bin/odrviewer --window 60 60 1024 576 --odr output.xodr --road_features --density 0 --ground_plane  --save_generated_model

OSGB_FILE=generated_road.osgb
if test -f "$OSGB_FILE"; then
    echo "Generate $OSGB_FILE success."
else
    echo "Generate $OSGB_FILE failed."
    exit
fi



# generate fbx
osgconv generated_road.osgb ${PACKAGE_NAME}.fbx -o -90-1,0,0 -s 100,100,100 --use-world-frame



## change name of the xodr file
mv output.xodr ${PACKAGE_NAME}.xodr

# move files to Import folder
##clear it first
rm -rf ${CARLA_SB_PATH}/Import/*
mv ${PACKAGE_NAME}.fbx ${CARLA_SB_PATH}/Import
mv ${PACKAGE_NAME}.xodr ${CARLA_SB_PATH}/Import

# clear content files of SB Carla
rm -rf ${CARLA_SB_PATH}/Unreal/CarlaUE4/Content/${PACKAGE_NAME}
## also Dist files
rm -rf ${CARLA_SB_PATH}/Dist/*

# make import
(cd ${CARLA_SB_PATH} ; make import ARGS="--package=${PACKAGE_NAME} --clean-intermediate")

# make package
(cd ${CARLA_SB_PATH} ; make package ARGS="--packages=${PACKAGE_NAME}" )

# delete files in Import folder of the dist Carla 
rm -rf ${CARLA_DIST_PATH}/Import/*

# move files to Import folder of the dist Carla
mv ${CARLA_SB_PATH}/Dist/*.tar.gz ${CARLA_DIST_PATH}/Import


# import assests
(cd ${CARLA_DIST_PATH} ;  /usr/bin/bash ImportAssets.sh)

end_time=$(date +%s)
cost_time=$[ $end_time-$start_time ]
echo "Total time: $(($cost_time/60))min $(($cost_time%60))s"