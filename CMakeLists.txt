cmake_minimum_required(VERSION 2.8.3)
project(map_store)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  nav_msgs
  rosconsole
  roscpp
  rostest
  warehouse_ros
  warehouse_ros_mongo
  std_msgs
)

add_message_files(
  FILES
  MapListEntry.msg
)

add_service_files(
  FILES
  DeleteMap.srv
  ListMaps.srv
  PublishMap.srv
  RenameMap.srv
  SaveMap.srv
  SetOrigin.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(map_saver src/map_saver.cpp)
add_dependencies(map_saver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
<<<<<<< HEAD
target_link_libraries(map_saver uuid ${catkin_LIBRARIES})
=======
target_link_libraries(map_saver uuid ${catkin_LIBRARIES} )
>>>>>>> 91d3a850875bd97a4aa5694673d4de325e70c05a

add_executable(map_manager src/map_manager.cpp)
add_dependencies(map_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_manager uuid ${catkin_LIBRARIES})

# add_rostest(test/map_store.test)

# install(TARGETS map_saver
# 	        map_manager
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install(PROGRAMS scripts/insert_map_from_file.py
#         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install(DIRECTORY launch
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
