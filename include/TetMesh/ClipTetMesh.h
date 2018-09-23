//
// Created by Brett Miller on 9/23/18.
//

#pragma once

#include "TetMesh.h"

namespace destroyer {

class ClipTetMesh : public TetMesh
{
public:

    using TetMesh::TetMesh;

    // Uses the base class constructor.
    ~ClipTetMesh() = default;

    void Clip(Real quality_threshold, Real distance_threshold);

private:
    void FlagInteriorNodes(Real distance_threshold);

    void ShiftNodesToBoundary(Real distance_threshold, Real quality_threshold);

    void SplitBoundaryTets(Real distance_threshold);

    void PrepSplitBoundaryTet(TetrahedronRef tet, int in_count, int on_count, int out_count);
    void ProcessSplitBoundaryTet(TetrahedronRef tet);

    void ProcessCase1Split(TetrahedronRef tet);
    void ProcessCase2Split(TetrahedronRef tet);
    void ProcessCase3Split(TetrahedronRef tet);
    void ProcessCase4Split(TetrahedronRef tet);
    void ProcessCase5Split(TetrahedronRef tet);
    void ProcessCase6Split(TetrahedronRef tet);

};


}; // namespace destroyer
