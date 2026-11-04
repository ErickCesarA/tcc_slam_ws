cmake_minimum_required(VERSION 3.16)
project(tcc_slam_ground_truth)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# Dependências ROS 2
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Protobuf REQUIRED)

# Caminhos das bibliotecas Gazebo instaladas pelo ROS 2
set(GZ_TRANSPORT_PREFIX "/opt/ros/lyrical/opt/gz_transport_vendor")
set(GZ_MSGS_PREFIX "/opt/ros/lyrical/opt/gz_msgs_vendor")

# Localizar bibliotecas Gazebo
find_library(GZ_TRANSPORT_LIBRARY
    NAMES gz-transport
    PATHS "${GZ_TRANSPORT_PREFIX}/lib"
    NO_DEFAULT_PATH
    REQUIRED
)

find_library(GZ_MSGS_LIBRARY
    NAMES gz-msgs
    PATHS "${GZ_MSGS_PREFIX}/lib"
    NO_DEFAULT_PATH
    REQUIRED
)

# Executável
add_executable(ground_truth_odom
    src/ground_truth_odom.cpp
)

# Diretórios de cabeçalhos
target_include_directories(ground_truth_odom PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${Protobuf_INCLUDE_DIRS}

    "${GZ_TRANSPORT_PREFIX}/include/gz/transport15"
    "${GZ_MSGS_PREFIX}/include/gz/msgs12"

    "/opt/ros/lyrical/opt/gz_utils_vendor/include/gz/utils4"
)

# Vincular bibliotecas ROS 2 e Gazebo sem a macro ament_target_dependencies
target_link_libraries(ground_truth_odom
    ${rclcpp_LIBRARIES}
    ${nav_msgs_LIBRARIES}
    "${GZ_TRANSPORT_LIBRARY}"
    "${GZ_MSGS_LIBRARY}"
    ${Protobuf_LIBRARIES}
)

# Instalação
install(
    TARGETS ground_truth_odom
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
