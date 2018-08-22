list( APPEND CMAKE_PREFIX_PATH "/usr/local/share/eigen3/cmake" )
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(../include)

set( library_name destroyer )

add_library( ${library_name} SHARED
        TetMesh/TetMesh.cpp
        TetMesh/TetNode.cpp
        TetMesh/Tetrahedron.cpp
        TetMesh/VDBSampler.cpp
        TetMesh/DetailGenerator.cpp
        TetMesh/BCCLatticeGenerator.cpp
        TetMesh/TetMeshToHoudiniDetail.cpp
        TetMesh/RefinementTetMesh.cpp
        #TetMesh/CompressionTetMesh.cpp
        TetMesh/TetFace.cpp
        TetMesh/TetEdge.cpp
        Houdini/Registration.cpp
        #Houdini/SOP_AnalyzeTetMesh.cpp
        #Houdini/SOP_CleanupTetMesh.cpp
        #Houdini/SOP_CompressTetMesh.cpp
        Houdini/SOP_RefineTetrahedra.cpp
        Houdini/SOP_TetrahedralizeVDB.cpp
        )

# Link against the Houdini libraries, and add required include directories and compile definitions.
target_link_libraries( ${library_name} Houdini Eigen3::Eigen)

# Configure several common target properties, such as its output directory.
houdini_configure_target( ${library_name} )