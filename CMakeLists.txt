cmake_minimum_required(VERSION 2.8.3)
project(pocketsphinx)

find_package(catkin REQUIRED)
catkin_package(DEPENDS)

install(DIRECTORY demo
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS nodes/voice_control_updated.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS nodes/kws_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS nodes/sendAudioAsTopic.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})