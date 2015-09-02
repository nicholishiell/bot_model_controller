cmake_minimum_required(VERSION 2.8.3)
project(bot_model_controller)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs image_transport vrep_common)

include_directories(${catkin_INCLUDE_DIRS})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(botModelController src/botModelController.cpp src/FSM/FSM.cpp 
		src/FSM/StateSearchPuck.cpp src/FSM/StateTrackPuck.cpp src/FSM/StateCapturePuck.cpp 
		src/FSM/StateSearchGoal.cpp src/FSM/StateTrackGoal.cpp src/FSM/StateReleasePuck.cpp
		src/FSM/StateDepositPuck.cpp src/FSM/StateEvade.cpp) 
target_link_libraries(botModelController ${catkin_LIBRARIES})
add_dependencies(botModelController vrep_common_generate_messages_cpp)


