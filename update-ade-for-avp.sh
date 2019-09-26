# run as 'sudo su'

export ROS_DISTRO=melodic

apt update
apt install wget
echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list
wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
apt update

apt install -y gazebo9 
apt install -y ros-$ROS_DISTRO-gazebo-plugins \
               ros-$ROS_DISTRO-gazebo-ros-pkgs \
               ros-$ROS_DISTRO-gazebo-ros-control 

apt install -y ros-$ROS_DISTRO-autoware-can-msgs \
               ros-$ROS_DISTRO-autoware-config-msgs \
               ros-$ROS_DISTRO-autoware-external-msgs \
               ros-$ROS_DISTRO-autoware-map-msgs \
               ros-$ROS_DISTRO-autoware-msgs \
               ros-$ROS_DISTRO-autoware-system-msgs

apt install -y ros-$ROS_DISTRO-pcl-ros

apt install -y ros-$ROS_DISTRO-tf2-geometry-msgs

apt install -y ros-$ROS_DISTRO-jsk-rviz-plugins

#apt install -y ros-$ROS_DISTRO-map-server
#apt install -y ros-$ROS_DISTRO-fake-localization
apt install -y \
        ros-$ROS_DISTRO-robot \
        ros-$ROS_DISTRO-map-server \
        ros-$ROS_DISTRO-ackermann-msgs \
        ros-$ROS_DISTRO-fake-localization \
        ros-$ROS_DISTRO-robot-localization \
        ros-$ROS_DISTRO-joy \
        ros-$ROS_DISTRO-teleop-twist-joy \
        ros-$ROS_DISTRO-teleop-twist-keyboard \
        ros-$ROS_DISTRO-pid	\
        ros-$ROS_DISTRO-geographic-info \
        ros-$ROS_DISTRO-unique-identifier \
        ros-$ROS_DISTRO-ros-canopen

apt install -y ros-$ROS_DISTRO-vector-map-msgs
#apt install -y ros-$ROS_DISTRO-nmea-msgs


