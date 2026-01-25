cmake_minimum_required(VERSION 3.16)

project(axonv_task LANGUAGES C CXX)

add_executable(axonv
  src/main.cpp
  src/App.cpp
  src/Surface.cpp
  src/ConvexHull.cpp
)

target_compile_features(axonv PRIVATE cxx_std_17)
#
#find_package(PCL CONFIG REQUIRED COMPONENTS
#  common
#  io
#  visualization
#  surface
#  search
#  kdtree
#)
#
#target_compile_definitions(axonv PRIVATE PCL_NO_PRECOMPILE)
#
## Prefer imported targets if the PCL package provides them (common with vcpkg builds)
#if(TARGET PCL::common)
#  target_link_libraries(axonv PRIVATE
#    PCL::common
#    PCL::io
#    PCL::visualization
#    PCL::surface
#    PCL::search
#    PCL::kdtree
#  )
#else()
#  # Fallback for older/non-target-based PCL packages
#  target_include_directories(axonv PRIVATE ${PCL_INCLUDE_DIRS})
#  target_link_libraries(axonv PRIVATE ${PCL_LIBRARIES})
#endif()
#
#add_custom_command(
#  TARGET axonv POST_BUILD
#  COMMAND ${CMAKE_COMMAND} -E copy_directory
#          "${CMAKE_SOURCE_DIR}/data"
#          "$<TARGET_FILE_DIR:axonv>/data"
#)


add_custom_command(
  TARGET axonv POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
          "${CMAKE_SOURCE_DIR}/data"
          "$<TARGET_FILE_DIR:axonv>/data"
)




find_package(PCL CONFIG REQUIRED COMPONENTS
  common
  io
  visualization
  surface
  search
  kdtree
)

target_compile_definitions(axonv PRIVATE PCL_NO_PRECOMPILE)

# vcpkg PCL should provide imported targets; use them when present.
if(TARGET PCL::visualization)
  target_link_libraries(axonv PRIVATE
    PCL::common
    PCL::io
    PCL::visualization
    PCL::surface
    PCL::search
    PCL::kdtree
  )
else()
  # Fallback: make sure includes/defs/libs are actually applied
  if(PCL_INCLUDE_DIRS)
    target_include_directories(axonv PRIVATE ${PCL_INCLUDE_DIRS})
  endif()

  if(PCL_DEFINITIONS)
    # PCL_DEFINITIONS sometimes includes -D; handle both forms
    foreach(def ${PCL_DEFINITIONS})
      string(REGEX REPLACE "^-D" "" def_stripped "${def}")
      target_compile_definitions(axonv PRIVATE "${def_stripped}")
    endforeach()
  endif()

  target_link_libraries(axonv PRIVATE ${PCL_LIBRARIES})
endif()

