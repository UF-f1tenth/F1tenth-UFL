# CMake generated Testfile for 
# Source directory: /home/f1tenth2/f110_ws/src/f110-fall2018-skeletons/system/urg_node
# Build directory: /home/f1tenth2/f110_ws/build/f110-fall2018-skeletons/system/urg_node
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_urg_node_roslint_package "/home/f1tenth2/f110_ws/build/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/f1tenth2/f110_ws/build/test_results/urg_node/roslint-urg_node.xml" "--working-dir" "/home/f1tenth2/f110_ws/build/f110-fall2018-skeletons/system/urg_node" "--return-code" "/opt/ros/melodic/share/roslint/cmake/../../../lib/roslint/test_wrapper /home/f1tenth2/f110_ws/build/test_results/urg_node/roslint-urg_node.xml make roslint_urg_node")
add_test(_ctest_urg_node_roslaunch-check_launch_urg_lidar.launch "/home/f1tenth2/f110_ws/build/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/f1tenth2/f110_ws/build/test_results/urg_node/roslaunch-check_launch_urg_lidar.launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/f1tenth2/f110_ws/build/test_results/urg_node" "/opt/ros/melodic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/f1tenth2/f110_ws/build/test_results/urg_node/roslaunch-check_launch_urg_lidar.launch.xml\" \"/home/f1tenth2/f110_ws/src/f110-fall2018-skeletons/system/urg_node/launch/urg_lidar.launch\" ")
