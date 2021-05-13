#!/bin/bash

BUILD_PX4="false"

#echo -e "\e[1;33m Do you want to build PX4 v1.10.1? (y) for simulation (n) if setting this up on on-barod computer \e[0m"
#read var
#if [ "$var" != "y" ] && [ "$var" != "Y" ] ; then
#    echo -e "\e[1;33m Skipping PX4 v1.10.1 \e[0m"
#    BUILD_PX4="false"
#    sleep 1
#else
#    echo -e "\e[1;33m PX4 v1.10.1 will be built \e[0m"
#    BUILD_PX4="true"
#    sleep 1
#fi

CATKIN_WS=${HOME}/HIVE/catkin_ws_test
CATKIN_SRC=${HOME}/HIVE/catkin_ws_test/src

if [ ! -d "$CATKIN_WS" ]; then
	echo "Creating $CATKIN_WS ... "
	mkdir -p $CATKIN_SRC
fi

if [ ! -d "$CATKIN_SRC" ]; then
	echo "Creating $CATKIN_SRC ..."
fi

# Configure catkin_Ws
cd $CATKIN_WS
catkin init
catkin config --merge-devel
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

grep -xF 'source '${HOME}'/HIVE/catkin_ws_test/devel/setup.bash' ${HOME}/.bashrc || echo "source $HOME/HIVE/catkin_ws_test/devel/setup.bash" >> $HOME/.bashrc
####################################### Setup PX4 v1.10.1 #######################################

#Copying this to  .bashrc file
grep -xF 'source ~/HIVE/PX4-Autopilot/Tools/setup_gazebo.bash ~/HIVE/PX4-Autopilot ~/HIVE/PX4-Autopilot/build/px4_sitl_default' ${HOME}/.bashrc || echo "source ~/HIVE/PX4-Autopilot/Tools/setup_gazebo.bash ~/HIVE/PX4-Autopilot ~/HIVE/PX4-Autopilot/build/px4_sitl_default" >> ${HOME}/.bashrc
grep -xF 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/HIVE/PX4-Autopilot' ${HOME}/.bashrc || echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/HIVE/PX4-Autopilot" >> ${HOME}/.bashrc
grep -xF 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/HIVE/PX4-Autopilot/Tools/sitl_gazebo' ${HOME}/.bashrc || echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/HIVE/PX4-Autopilot/Tools/sitl_gazebo" >> ${HOME}/.bashrc
grep -xF 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-9/plugins' ${HOME}/.bashrc || echo "export GAZEBO_PLUGIN_PATH=\$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-9/plugins" >> ${HOME}/.bashrc
grep -xF 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:'${HOME}'/HIVE/catkin_ws_test/src/px4_fast_planner/models' ${HOME}/.bashrc || echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:${HOME}/HIVE/catkin_ws_test/src/px4_fast_planner/models" >> ${HOME}/.bashrc

# Copy PX4 SITL param file
cp $CATKIN_SRC/px4_fast_planner/config/10017_iris_depth_camera ${HOME}/HIVE/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/

source ${HOME}/.bashrc
# Install MAVROS
sudo apt install ros-melodic-mavros ros-melodic-mavros-extras -y
####################################### mavros_controllers setup #######################################
echo -e "\e[1;33m Adding mavros_controllers \e[0m"
#Adding mavros_controllers
if [ ! -d "$CATKIN_SRC/mavros_controllers" ]; then
    echo "Cloning the mavros_controllers repo ..."
    cd $CATKIN_SRC
    git clone https://github.com/Jaeyoung-Lim/mavros_controllers.git
    cd ../
else
    echo "mavros_controllers already exists. Just pulling ..."
    cd $CATKIN_SRC/mavros_controllers
    git pull
    cd ../ 
fi

#Adding catkin_simple
if [ ! -d "$CATKIN_SRC/catkin_simple" ]; then
    echo "Cloning the catkin_simple repo ..."
    cd $CATKIN_SRC
    git clone https://github.com/catkin/catkin_simple
    cd ../
else
    echo "catkin_simple already exists. Just pulling ..."
    cd $CATKIN_SRC/catkin_simple
    git pull
    cd ../ 
fi

#Adding eigen_catkin
if [ ! -d "$CATKIN_SRC/eigen_catkin" ]; then
    echo "Cloning the eigen_catkin repo ..."
    cd $CATKIN_SRC
    git clone https://github.com/ethz-asl/eigen_catkin
    cd ../
else
    echo "eigen_catkin already exists. Just pulling ..."
    cd $CATKIN_SRC/eigen_catkin
    git pull
    cd ../ 
fi

#Adding eigen_catkin
if [ ! -d "$CATKIN_SRC/mav_comm" ]; then
    echo "Cloning the mav_comm repo ..."
    cd $CATKIN_SRC
    git clone https://github.com/ethz-asl/mav_comm
    cd ../
else
    echo "mav_comm already exists. Just pulling ..."
    cd $CATKIN_SRC/mav_comm
    git pull
    cd ../ 
fi


####################################### Fast-planner setup #######################################
echo -e "\e[1;33m Adding Fast-Planner \e[0m"
# Required for Fast-Planner
sudo apt install ros-melodic-nlopt libarmadillo-dev -y

#Adding Fast-Planner
if [ ! -d "$CATKIN_SRC/Fast-Planner" ]; then
    echo "Cloning the Fast-Planner repo ..."
    cd $CATKIN_SRC
    git clone https://github.com/mzahana/Fast-Planner.git
    cd ../
else
    echo "Fast-Planner already exists. Just pulling ..."
    cd $CATKIN_SRC/Fast-Planner
    git pull
    cd ../ 
fi

# Checkout ROS Mellodic branch 
cd $CATKIN_SRC/Fast-Planner
git checkout changes_for_ros_melodic

####################################### Building catkin_ws #######################################
cd $CATKIN_WS
catkin build multi_map_server
catkin build
source $CATKIN_WS/devel/setup.bash