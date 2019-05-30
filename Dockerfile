FROM ros:indigo

RUN apt-get update && apt-get install -y \
	python-catkin-pkg python-rosdep python-wstool \
	python-catkin-tools ros-indigo-catkin \
	&& rm -rf /var/lib/apt/lists

ENV CATKIN_WS=/root/catkin_ws
RUN rm /bin/sh \
	&& ln -s /bin/bash /bin/sh
	
RUN source /ros_entrypoint.sh \
	&& mkdir -p $CATKIN_WS/src \
	&& cd $CATKIN_WS \
	&& catkin init \
	&& cd $CATKIN_WS/src \
	&& git clone https://github.com/ICRA2017/costar_stack.git \
	&& git clone https://github.com/SalvoVirga/iiwa_stack.git \
	&& git clone https://github.com/ros-industrial/robotiq.git \
	&& git clone https://github.com/jbohren/rqt_dot.git \
	&& git clone https://github.com/sniekum/ar_track_alvar.git \
	&& git clone https://github.com/sniekum/ar_track_alvar_msgs.git \
	&& git clone https://github.com/gt-ros-pkg/hrl-kdl.git \
	&& git clone https://github.com/xqms/ur_modern_driver.git --branch thread_safety \
	&& git clone https://github.com/jhu-lcsr/ObjRecRANSAC.git

RUN apt-get update && apt-get install -y \
	software-properties-common \
	&& rm -rf /var/lib/apt/lists

RUN add-apt-repository --yes ppa:v-launchpad-jochen-sprickerhof-de/pcl  \
	&& apt-get update && apt-get install -y libpcl-all \
	&& rm -rf /var/lib/apt/lists

RUN add-apt-repository --yes ppa:xqms/opencv-nonfree \
	&& apt-get update && apt-get install -y libopencv-nonfree-dev \
	&& rm -rf /var/lib/apt/lists

RUN source /ros_entrypoint.sh \
	&& apt-get update \
	&& cd $CATKIN_WS/src \
	&& rosdep install -y --from-paths ./ --ignore-src --rosdistro indigo

RUN source /ros_entrypoint.sh \
	&& cd $CATKIN_WS \
	&& catkin build
	