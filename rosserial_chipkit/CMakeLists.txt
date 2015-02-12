cmake_minimum_required(VERSION 2.8.3)
project(rosserial_chipkit)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_message_files(FILES
                  Adc.msg
                 )

add_service_files(FILES
                  Test.srv
                 )

generate_messages()

install(DIRECTORY src/ros_lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)

install(DIRECTORY chipkit-cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS src/rosserial_chipkit/make_libraries.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



