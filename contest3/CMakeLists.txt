cmake_minimum_required(VERSION 2.8)
project(mie443_contest3)

find_package(OpenCV 3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
		cv_bridge 
		image_transport 
		sensor_msgs
		std_msgs
		kobuki_msgs)

catkin_package()

include_directories(include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# add the publisher example
add_executable(publisher src/my_publisher_video.cpp)
target_link_libraries(publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add the subscriber example
add_executable(contest3 src/contest3.cpp
			src/imageTransporter.cpp)
target_link_libraries(contest3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Webcam Publisher Executable
add_executable(webcam_publisher3 
    src/webcam_publisher.cpp)
target_link_libraries(webcam_publisher3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Contest 3 Executable
add_executable(contest3img
    src/Image.cpp)
target_link_libraries(contest3img ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
