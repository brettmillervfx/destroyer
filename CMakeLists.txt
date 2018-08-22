cmake_minimum_required(VERSION 3.6)
project(destroyer)

set(CMAKE_CXX_STANDARD 11)

set( HFS "/Applications/Houdini/Houdini16.5.323/Frameworks/Houdini.framework/Versions/Current/Resources/")
list( APPEND CMAKE_PREFIX_PATH "${HFS}/toolkit/cmake" )
find_package( Houdini REQUIRED )

add_subdirectory(src)
add_subdirectory(test)
