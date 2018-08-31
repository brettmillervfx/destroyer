include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(../../include)
include_directories(${_houdini_include_dir})

add_executable( runTetMeshChecks
        TetNode_check.cpp
        TetEdge_check.cpp
        #Tetrahedron_check.cpp
        runTetMeshTests.cpp
        TetMesh_test.cpp
        TetFace_check.cpp
        #RefinementTetMesh_check.cpp
        )

target_link_libraries( runTetMeshChecks gtest gtest_main )
target_link_libraries( runTetMeshChecks destroyer )


get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
    message(STATUS "dir='${dir}'")
endforeach()
