SOURCECMDS=source /opt/ros/humble/setup.bash && source install/setup.bash

deps:
	sudo apt update
	sudo apt-get install ros-humble-rttest ros-humble-rclcpp-action ros-humble-gazebo-dev ros-humble-gazebo-msgs ros-humble-gazebo-plugins ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs ros-humble-joint-state-publisher-gui ros-humble-xacro

build: FORCE
	cd ../
	bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

rviz: build
	cd ../
	bash -c "$(SOURCECMDS) && ros2 launch marvin_simulation display.launch.py"

gazebo: build
	cd ../
	bash -c "$(SOURCECMDS) && ros2 launch marvin_simulation simulation.launch.py"
	
FORCE: ;
