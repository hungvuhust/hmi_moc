cmake_minimum_required(VERSION 3.10.0)
project(rtcrobot_hmi_denso VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(QT_QML_GENERATE_QMLLS_INI ON)

find_package(ament_cmake REQUIRED)

find_package(Qt5 COMPONENTS Core Gui Quick Qml QuickControls2 REQUIRED)
set(THIS_PACKAGE_DEPENDENCIES 
    rclcpp
    std_msgs

    vda5050_msgs
    rtc_nodecore
)

foreach(DEPENDENCY ${THIS_PACKAGE_DEPENDENCIES})
    find_package(${DEPENDENCY} REQUIRED)
endforeach()


qt5_add_resources(QML_RESOURCES qml.qrc)

include_directories(include)


add_executable(qml_ros2_app
    src/main.cpp
    include/rtcrobot_hmi_denso/RobotData.hpp
    include/rtcrobot_hmi_denso/DeviceStatus.hpp
    include/rtcrobot_hmi_denso/RosClient.hpp
    src/objects/RobotData.cpp
    src/objects/DeviceStatus.cpp
    src/objects/RosClient.cpp
    ${QML_RESOURCES}
)
target_link_libraries(qml_ros2_app
    Qt5::Core
    Qt5::Gui
    Qt5::Quick
    Qt5::Qml
    Qt5::QuickControls2
)

install(DIRECTORY qml DESTINATION share/rtcrobot_hmi_denso)
install(DIRECTORY include DESTINATION include)


install(TARGETS 
    qml_ros2_app 
    DESTINATION lib/rtcrobot_hmi_denso
    COMPONENT runtime
)


ament_target_dependencies(qml_ros2_app ${THIS_PACKAGE_DEPENDENCIES})

ament_package()