cmake_minimum_required(VERSION 3.5)
project(ros_gz_example_application)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)  # Add Eigen3
find_package(Curses REQUIRED)



#IGNPUBSUB
add_executable(ign_pubsub src/ign_pubsub.cpp src/ButterworthFilter.cpp)
ament_target_dependencies(ign_pubsub
 rclcpp 
 std_msgs 
 sensor_msgs 
 ros_gz_interfaces 
 geometry_msgs 
 tf2 
 tf2_ros 
 visualization_msgs)

install(TARGETS
  ign_pubsub
  DESTINATION lib/${PROJECT_NAME})


 target_include_directories(ign_pubsub PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
 ${EIGEN3_INCLUDE_DIR}  # Add Eigen include directory 
)

target_link_libraries(ign_pubsub ButterworthFilter)


#SEDAS_RVIZ
add_executable(sedas_rviz src/sedas_rviz.cpp)
ament_target_dependencies(sedas_rviz
 rclcpp 
 std_msgs 
 sensor_msgs 
 ros_gz_interfaces 
 geometry_msgs 
 tf2 
 tf2_ros 
 visualization_msgs)

install(TARGETS
sedas_rviz
  DESTINATION lib/${PROJECT_NAME})


 target_include_directories(sedas_rviz PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
 ${EIGEN3_INCLUDE_DIR}  # Add Eigen include directory 
)

target_link_libraries(sedas_rviz)


#PINOCCHIOTRY
add_executable(pinocchio_try src/pinocchio_try.cpp)
ament_target_dependencies(pinocchio_try
 rclcpp 
 std_msgs 
 sensor_msgs 
 geometry_msgs 
 tf2 
 tf2_ros 
)
 
target_link_libraries(pinocchio_try pinocchio::pinocchio)

target_include_directories(pinocchio_try PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
${EIGEN3_INCLUDE_DIR}  # Add Eigen include directory 
)

install(TARGETS
  pinocchio_try
  DESTINATION lib/${PROJECT_NAME})



#SEDAS_KEY_PUBLISHER
add_executable(sedas_key_publisher src/sedas_key_publisher.cpp)
target_link_libraries(sedas_key_publisher ${CURSES_LIBRARIES})
ament_target_dependencies(sedas_key_publisher
 rclcpp
 std_msgs
 geometry_msgs
 )

install(TARGETS
  sedas_key_publisher
  DESTINATION lib/${PROJECT_NAME}
)


#SEDAS_TRAJ
add_executable(sedas_traj src/sedas_traj.cpp)
target_link_libraries(sedas_traj ButterworthFilter ${CURSES_LIBRARIES})
ament_target_dependencies(sedas_traj
 rclcpp
 std_msgs
 geometry_msgs
 )

 target_include_directories(sedas_traj PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
 ${EIGEN3_INCLUDE_DIR}  # Add Eigen include directory 
 )
 

install(TARGETS
  sedas_traj
  DESTINATION lib/${PROJECT_NAME}
)

#CSV_SAVER
add_executable(csv_saver src/csv_saver.cpp)
target_link_libraries(csv_saver ${CURSES_LIBRARIES})
ament_target_dependencies(csv_saver
 rclcpp
 std_msgs
 geometry_msgs
 )

 target_include_directories(csv_saver PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
 ${EIGEN3_INCLUDE_DIR}  # Add Eigen include directory 
 )
 

install(TARGETS
  csv_saver
  DESTINATION lib/${PROJECT_NAME}
)


#butterworth filter
add_library(ButterworthFilter src/ButterworthFilter.cpp)
target_include_directories(ButterworthFilter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}  # Eigen의 include 경로 추가
)


ament_package()
