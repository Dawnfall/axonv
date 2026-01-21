cmake_minimum_required(VERSION 3.2)
project(axonv_task LANGUAGES C CXX)

add_executable(axonv 
"src/main.cpp"
"src/App.cpp"
"src/Surface.cpp"
"src/ConvexHull.cpp"
)

#add_executable(axonv ${AXONV_SOURCES})
target_compile_features(axonv PRIVATE cxx_std_17)
find_package(PCL CONFIG REQUIRED COMPONENTS 
common 
io
visualization 
surface 
search 
kdtree
)

#target_include_directories(axonv PRIVATE ${PCL_INCLUDE_DIRS})

target_compile_definitions(axonv PRIVATE PCL_NO_PRECOMPILE)
target_link_libraries(axonv PRIVATE ${PCL_LIBRARIES})
#target_compile_definitions(axonv PRIVATE ${PCL_DEFINITIONS})