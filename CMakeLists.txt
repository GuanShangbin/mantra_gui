##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(mantra_gui)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
##############################################################################
# Catkin
##############################################################################
# qt_build provides the qt cmake glue, roscpp the comms for a default talker

set(CMAKE_INCLUDE_CURRENT_DIR ON)#added
find_package(Qt5 REQUIRED Core Widgets) #added
set(QT_LIBRARIES Qt5::Widgets) #added

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    rs_perception
    darknet_ros_msgs
    rviz
    cv_bridge
    image_transport
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    )
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package(
    )

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
# rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/mantra_gui/*.hpp *.h)

#QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
#QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
#QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})


##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(mantra_gui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(mantra_gui ${QT_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS mantra_gui RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
