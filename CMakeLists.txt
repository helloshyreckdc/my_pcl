cmake_minimum_required(VERSION 2.8.3)
project(my_pcl)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
	find_package(catkin REQUIRED COMPONENTS
			pcl_conversions
			pcl_ros
			roscpp
			sensor_msgs
			)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_pcl
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
		)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
	include_directories(
			${PCL_INCLUDE_DIRS}
			${catkin_INCLUDE_DIRS}
			)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
## Declare a C++ library
# add_library(my_pcl
#   src/${PROJECT_NAME}/my_pcl.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(my_pcl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(my_pcl_node src/my_pcl_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(my_pcl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(my_pcl_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS my_pcl my_pcl_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_my_pcl.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

#add_executable(example src/example.cpp)
#target_link_libraries(example ${catkin_LIBRARIES})
#
#
#add_executable(pcd_write src/pcd_write.cpp)
#target_link_libraries(pcd_write ${catkin_LIBRARIES})
#
#
#add_executable(openni_viewer_simple src/openni_viewer_simple.cpp)
#target_link_libraries(openni_viewer_simple ${catkin_LIBRARIES})
#
#add_executable(openni_grabber src/openni_grabber.cpp)
#target_link_libraries(openni_grabber ${catkin_LIBRARIES})

add_executable(passthrough src/passthrough.cpp)
target_link_libraries(passthrough ${catkin_LIBRARIES})

	add_executable(voxel_grid src/voxel_grid.cpp)
target_link_libraries(voxel_grid ${catkin_LIBRARIES})

	add_executable(statistical_removal src/statistical_removal.cpp)
target_link_libraries(statistical_removal ${catkin_LIBRARIES})

add_executable(project_inliers src/project_inliers.cpp)
target_link_libraries(project_inliers ${catkin_LIBRARIES})

add_executable(planar_segmentation src/planar_segmentation.cpp)
target_link_libraries(planar_segmentation ${catkin_LIBRARIES})

add_executable(cylinder_segmentation src/cylinder_segmentation.cpp)
target_link_libraries(cylinder_segmentation ${catkin_LIBRARIES})

add_executable(kdtree_search src/kdtree_search.cpp)
target_link_libraries(kdtree_search ${catkin_LIBRARIES})

add_executable(cluster_extraction src/cluster_extraction.cpp)
target_link_libraries(cluster_extraction ${catkin_LIBRARIES})

	add_executable(extract_indices src/extract_indices.cpp)
target_link_libraries(extract_indices ${catkin_LIBRARIES})
target_link_libraries(extract_indices ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so libpthread.so libvtkGraphics.so)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

	add_executable(iterative_closest_point src/iterative_closest_point.cpp)
target_link_libraries(iterative_closest_point ${catkin_LIBRARIES})
target_link_libraries(iterative_closest_point ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so libpthread.so libvtkGraphics.so)
	add_executable(correspondence_grouping src/correspondence_grouping.cpp)
target_link_libraries(correspondence_grouping ${catkin_LIBRARIES})
target_link_libraries(correspondence_grouping ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so )

	add_executable(normal_estimation_using_integral_images src/normal_estimation_using_integral_images.cpp)
target_link_libraries(normal_estimation_using_integral_images ${catkin_LIBRARIES})
target_link_libraries(normal_estimation_using_integral_images ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so )

#add_executable(moment_of_inertia src/moment_of_inertia.cpp)
#target_link_libraries(moment_of_inertia ${catkin_LIBRARIES})	
#target_link_libraries(moment_of_inertia ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)

	add_executable(ply2pcd src/ply2pcd.cpp)
target_link_libraries(ply2pcd ${catkin_LIBRARIES})
target_link_libraries(ply2pcd ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)
	add_executable(template_alignment src/template_alignment.cpp)
target_link_libraries(template_alignment ${catkin_LIBRARIES})
target_link_libraries(template_alignment ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)


add_executable(matrix_transform src/matrix_transform.cpp)
target_link_libraries(matrix_transform ${catkin_LIBRARIES})
target_link_libraries(matrix_transform ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)

add_executable(general_matrix_transform src/general_matrix_transform.cpp)
target_link_libraries(general_matrix_transform ${catkin_LIBRARIES})
target_link_libraries(general_matrix_transform ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)

add_executable(publish_transform_matrix_from_scene src/publish_transform_matrix_from_scene.cpp)
target_link_libraries(publish_transform_matrix_from_scene ${catkin_LIBRARIES})
target_link_libraries(publish_transform_matrix_from_scene ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)
#list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
#
#	find_package(HDF5)
#if(HDF5_FOUND)
#
#	find_package(FLANN)
#	include_directories(${FLANN_INCLUDE_DIRS})
#
#	include_directories(${HDF5_INCLUDE_DIR})
#
#	add_executable(build_tree build_tree.cpp)
#	target_link_libraries(build_tree ${PCL_LIBRARIES} ${Boost_LIBRARIES}
#			${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY})
#
#	add_executable(nearest_neighbors nearest_neighbors.cpp)
#	target_link_libraries(nearest_neighbors ${PCL_LIBRARIES} 
#			${Boost_LIBRARIES} ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY} 
#			)
#endif(HDF5_FOUND)
	add_executable(scan_scene_to_pcd src/scan_scene_to_pcd.cpp)
target_link_libraries(scan_scene_to_pcd ${catkin_LIBRARIES})
target_link_libraries(scan_scene_to_pcd ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)

	add_executable(alignment_prerejective src/alignment_prerejective.cpp)
target_link_libraries(alignment_prerejective ${catkin_LIBRARIES})
target_link_libraries(alignment_prerejective ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)
