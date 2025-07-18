# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ubuntu/pd_ros/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/pd_ros/build

# Utility rule file for plutodrone_generate_messages_cpp.

# Include the progress variables for this target.
include plutodrone/CMakeFiles/plutodrone_generate_messages_cpp.dir/progress.make

plutodrone/CMakeFiles/plutodrone_generate_messages_cpp: /home/ubuntu/pd_ros/devel/include/plutodrone/PlutoMsg.h
plutodrone/CMakeFiles/plutodrone_generate_messages_cpp: /home/ubuntu/pd_ros/devel/include/plutodrone/PlutoMsgAP.h
plutodrone/CMakeFiles/plutodrone_generate_messages_cpp: /home/ubuntu/pd_ros/devel/include/plutodrone/Drone_stats.h
plutodrone/CMakeFiles/plutodrone_generate_messages_cpp: /home/ubuntu/pd_ros/devel/include/plutodrone/PlutoPilot.h
plutodrone/CMakeFiles/plutodrone_generate_messages_cpp: /home/ubuntu/pd_ros/devel/include/plutodrone/aruco.h
plutodrone/CMakeFiles/plutodrone_generate_messages_cpp: /home/ubuntu/pd_ros/devel/include/plutodrone/SetPos.h


/home/ubuntu/pd_ros/devel/include/plutodrone/PlutoMsg.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/ubuntu/pd_ros/devel/include/plutodrone/PlutoMsg.h: /home/ubuntu/pd_ros/src/plutodrone/msg/PlutoMsg.msg
/home/ubuntu/pd_ros/devel/include/plutodrone/PlutoMsg.h: /opt/ros/noetic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ubuntu/pd_ros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating C++ code from plutodrone/PlutoMsg.msg"
	cd /home/ubuntu/pd_ros/src/plutodrone && /home/ubuntu/pd_ros/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/ubuntu/pd_ros/src/plutodrone/msg/PlutoMsg.msg -Iplutodrone:/home/ubuntu/pd_ros/src/plutodrone/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p plutodrone -o /home/ubuntu/pd_ros/devel/include/plutodrone -e /opt/ros/noetic/share/gencpp/cmake/..

/home/ubuntu/pd_ros/devel/include/plutodrone/PlutoMsgAP.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/ubuntu/pd_ros/devel/include/plutodrone/PlutoMsgAP.h: /home/ubuntu/pd_ros/src/plutodrone/msg/PlutoMsgAP.msg
/home/ubuntu/pd_ros/devel/include/plutodrone/PlutoMsgAP.h: /opt/ros/noetic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ubuntu/pd_ros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating C++ code from plutodrone/PlutoMsgAP.msg"
	cd /home/ubuntu/pd_ros/src/plutodrone && /home/ubuntu/pd_ros/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/ubuntu/pd_ros/src/plutodrone/msg/PlutoMsgAP.msg -Iplutodrone:/home/ubuntu/pd_ros/src/plutodrone/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p plutodrone -o /home/ubuntu/pd_ros/devel/include/plutodrone -e /opt/ros/noetic/share/gencpp/cmake/..

/home/ubuntu/pd_ros/devel/include/plutodrone/Drone_stats.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/ubuntu/pd_ros/devel/include/plutodrone/Drone_stats.h: /home/ubuntu/pd_ros/src/plutodrone/msg/Drone_stats.msg
/home/ubuntu/pd_ros/devel/include/plutodrone/Drone_stats.h: /opt/ros/noetic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ubuntu/pd_ros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating C++ code from plutodrone/Drone_stats.msg"
	cd /home/ubuntu/pd_ros/src/plutodrone && /home/ubuntu/pd_ros/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/ubuntu/pd_ros/src/plutodrone/msg/Drone_stats.msg -Iplutodrone:/home/ubuntu/pd_ros/src/plutodrone/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p plutodrone -o /home/ubuntu/pd_ros/devel/include/plutodrone -e /opt/ros/noetic/share/gencpp/cmake/..

/home/ubuntu/pd_ros/devel/include/plutodrone/PlutoPilot.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/ubuntu/pd_ros/devel/include/plutodrone/PlutoPilot.h: /home/ubuntu/pd_ros/src/plutodrone/srv/PlutoPilot.srv
/home/ubuntu/pd_ros/devel/include/plutodrone/PlutoPilot.h: /opt/ros/noetic/share/gencpp/msg.h.template
/home/ubuntu/pd_ros/devel/include/plutodrone/PlutoPilot.h: /opt/ros/noetic/share/gencpp/srv.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ubuntu/pd_ros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Generating C++ code from plutodrone/PlutoPilot.srv"
	cd /home/ubuntu/pd_ros/src/plutodrone && /home/ubuntu/pd_ros/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/ubuntu/pd_ros/src/plutodrone/srv/PlutoPilot.srv -Iplutodrone:/home/ubuntu/pd_ros/src/plutodrone/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p plutodrone -o /home/ubuntu/pd_ros/devel/include/plutodrone -e /opt/ros/noetic/share/gencpp/cmake/..

/home/ubuntu/pd_ros/devel/include/plutodrone/aruco.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/ubuntu/pd_ros/devel/include/plutodrone/aruco.h: /home/ubuntu/pd_ros/src/plutodrone/srv/aruco.srv
/home/ubuntu/pd_ros/devel/include/plutodrone/aruco.h: /opt/ros/noetic/share/gencpp/msg.h.template
/home/ubuntu/pd_ros/devel/include/plutodrone/aruco.h: /opt/ros/noetic/share/gencpp/srv.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ubuntu/pd_ros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Generating C++ code from plutodrone/aruco.srv"
	cd /home/ubuntu/pd_ros/src/plutodrone && /home/ubuntu/pd_ros/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/ubuntu/pd_ros/src/plutodrone/srv/aruco.srv -Iplutodrone:/home/ubuntu/pd_ros/src/plutodrone/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p plutodrone -o /home/ubuntu/pd_ros/devel/include/plutodrone -e /opt/ros/noetic/share/gencpp/cmake/..

/home/ubuntu/pd_ros/devel/include/plutodrone/SetPos.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/ubuntu/pd_ros/devel/include/plutodrone/SetPos.h: /home/ubuntu/pd_ros/src/plutodrone/srv/SetPos.srv
/home/ubuntu/pd_ros/devel/include/plutodrone/SetPos.h: /opt/ros/noetic/share/gencpp/msg.h.template
/home/ubuntu/pd_ros/devel/include/plutodrone/SetPos.h: /opt/ros/noetic/share/gencpp/srv.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ubuntu/pd_ros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Generating C++ code from plutodrone/SetPos.srv"
	cd /home/ubuntu/pd_ros/src/plutodrone && /home/ubuntu/pd_ros/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/ubuntu/pd_ros/src/plutodrone/srv/SetPos.srv -Iplutodrone:/home/ubuntu/pd_ros/src/plutodrone/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p plutodrone -o /home/ubuntu/pd_ros/devel/include/plutodrone -e /opt/ros/noetic/share/gencpp/cmake/..

plutodrone_generate_messages_cpp: plutodrone/CMakeFiles/plutodrone_generate_messages_cpp
plutodrone_generate_messages_cpp: /home/ubuntu/pd_ros/devel/include/plutodrone/PlutoMsg.h
plutodrone_generate_messages_cpp: /home/ubuntu/pd_ros/devel/include/plutodrone/PlutoMsgAP.h
plutodrone_generate_messages_cpp: /home/ubuntu/pd_ros/devel/include/plutodrone/Drone_stats.h
plutodrone_generate_messages_cpp: /home/ubuntu/pd_ros/devel/include/plutodrone/PlutoPilot.h
plutodrone_generate_messages_cpp: /home/ubuntu/pd_ros/devel/include/plutodrone/aruco.h
plutodrone_generate_messages_cpp: /home/ubuntu/pd_ros/devel/include/plutodrone/SetPos.h
plutodrone_generate_messages_cpp: plutodrone/CMakeFiles/plutodrone_generate_messages_cpp.dir/build.make

.PHONY : plutodrone_generate_messages_cpp

# Rule to build all files generated by this target.
plutodrone/CMakeFiles/plutodrone_generate_messages_cpp.dir/build: plutodrone_generate_messages_cpp

.PHONY : plutodrone/CMakeFiles/plutodrone_generate_messages_cpp.dir/build

plutodrone/CMakeFiles/plutodrone_generate_messages_cpp.dir/clean:
	cd /home/ubuntu/pd_ros/build/plutodrone && $(CMAKE_COMMAND) -P CMakeFiles/plutodrone_generate_messages_cpp.dir/cmake_clean.cmake
.PHONY : plutodrone/CMakeFiles/plutodrone_generate_messages_cpp.dir/clean

plutodrone/CMakeFiles/plutodrone_generate_messages_cpp.dir/depend:
	cd /home/ubuntu/pd_ros/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/pd_ros/src /home/ubuntu/pd_ros/src/plutodrone /home/ubuntu/pd_ros/build /home/ubuntu/pd_ros/build/plutodrone /home/ubuntu/pd_ros/build/plutodrone/CMakeFiles/plutodrone_generate_messages_cpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : plutodrone/CMakeFiles/plutodrone_generate_messages_cpp.dir/depend

