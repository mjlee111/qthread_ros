
cmake_minimum_required(VERSION 2.8.0)
project(turtle_master)


find_package(catkin REQUIRED 
    COMPONENTS 
    qt_build 
    roscpp 
    cv_bridge
    rospy
    cv_bridge
    image_transport
    std_msgs
    robit_msgs
    roslib
    )

set(OpenCV_INCLUDE_DIRS
  /usr/local/include/opencv4
  /usr/local/include/opencv4/opencv2
  #/home/robit/opencv/opencv_contrib/modules/xfeatures2d/include
  #/home/robit/opencv/opencv_contrib/modules/xfeatures2d/include/opencv2
)

set(OpenCV_LIB_DIR
  /usr/local/lib
)

set(OpenCV_LIBS
  opencv_core
  opencv_highgui
  opencv_imgcodecs
  opencv_videoio
  opencv_imgproc
  opencv_features2d
  opencv_flann
  opencv_calib3d
)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES turtle_master
    CATKIN_DEPENDS image_transport roscpp roslib rospy std_msgs robit_msgs
    DEPENDS)


rosbuild_prepare_qt4(QtCore QtGui) 



file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/turtle_master/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})



file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)



add_executable(turtle_master ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
add_dependencies(turtle_master ${robit_msgs_EXPORTED_TARGETS})
target_link_libraries(turtle_master ${QT_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS})
install(TARGETS turtle_master RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

