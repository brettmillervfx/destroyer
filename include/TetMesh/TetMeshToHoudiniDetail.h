//
// Created by Brett Miller on 8/15/18.
//

#pragma once

#include <GU/GU_Detail.h>
#include <OP/OP_Error.h>

#include "Types.h"


namespace destroyer {

/*************************************************************************************

 TetMeshToHoudiniDetail

 Once a TetMesh has been fully formed, this class can be used to inject the geometry into
 a Houdini detail.

*************************************************************************************/

class TetMeshToHoudiniDetail {
public:
    TetMeshToHoudiniDetail(TetMeshPtr tet_mesh, GU_Detail* gdp);
    ~TetMeshToHoudiniDetail() = default;

    // Converts the contained TetMesh into Houdini detail and writes into the contained gdp.
    void Convert();

    // Converts only the boundary faces to a manifold mesh in the gdp.
    void ConvertBoundary();


private:
    void ConvertPoints(bool boundary_only=false);
    void ConvertTetrahedra();
    void ConvertBoundaryFaces();

private:
    TetMeshPtr tet_mesh_;
    GU_Detail* gdp_;
};


}; // namespace destroyer
