cmake_minimum_required(VERSION 3.10.0)
project(hmi_ui VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(QT_QML_GENERATE_QMLLS_INI ON)

find_package(ament_cmake REQUIRED)

find_package(Qt5 COMPONENTS Core Gui Quick Qml QuickControls2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

qt5_add_resources(QML_RESOURCES qml.qrc)

include_directories(include)

add_executable(qml_ros2_app
    src/main.cpp
    include/hmi_moc/RobotData.hpp
    include/hmi_moc/DeviceStatus.hpp
    src/objects/RobotData.cpp
    src/objects/DeviceStatus.cpp
    ${QML_RESOURCES}
)
target_link_libraries(qml_ros2_app
    Qt5::Core
    Qt5::Gui
    Qt5::Quick
    Qt5::Qml
    Qt5::QuickControls2
)
ament_target_dependencies(qml_ros2_app rclcpp std_msgs)

ament_package()