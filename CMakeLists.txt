cmake_minimum_required(VERSION 2.8.3)
project(pocketsphinx)

find_package(catkin REQUIRED)
catkin_package(DEPENDS)

# catkin_package()


install(DIRECTORY demo
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# install(PROGRAMS nodes/voice_control_updated.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install(PROGRAMS nodes/kws_test.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install(PROGRAMS nodes/sendAudioAsTopic.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install(PROGRAMS nodes/jsgf_test.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# add_executable(audio_control nodes/sendAudioAsTopic.py)
# target_link_libraries(audio_control ${catkin_LIBRARIES})

# add_executable(kws_control nodes/kws_test.py)
# target_link_libraries(kws_control ${catkin_LIBRARIES})

# add_executable(jsgf_control nodes/jsgf_test.py)
# target_link_libraries(jsgf_control ${catkin_LIBRARIES})

# add_executable(audio_topic nodes/sendAudioAsTopic.py)
# target_link_libraries(audio_topic ${catkin_LIBRARIES})