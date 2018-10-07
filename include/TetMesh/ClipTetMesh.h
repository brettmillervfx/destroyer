//
// Created by Brett Miller on 9/23/18.
//

#pragma once

#include "TetMesh.h"

namespace destroyer {

/*************************************************************************************

 ClipTetMesh class

 ClipTetMesh is a specialization of TetMesh used to clip a tetmesh to a level set.

 The technique used is described in "Tetrahedral mesh generation based on space indicator functions",
 Friess et al.

 The algorithm reaches boundary conformity in two successive passes. Pass 1 shifts selected nodes to the
 boundary surface; it largely eliminates configurations where splitting would produce sliver elements.
 Pass 2 splits boundary-crossing elements left over from Pass 1.

 Before Pass 1, all nodes are assigned a classification: IN, ON, or OUT (referring to the relation with
 the level set). If an edge crosses the level set, the closer of the two nodes forming the endpoints is
 moved to the level set, unless the quality of an incident tetrahedron is degraded below a prescribed threshold.

 During Pass 2, tets that cross the boundary are assigned one of 6 categories (please refer to Figure 7 in the
 paper for a useful illustration). The category determines which edges are split and how the tet is to be subdivided.

*************************************************************************************/


    class ClipTetMesh : public TetMesh
{
public:

    using TetMesh::TetMesh;

    // Uses the base class constructor.
    ~ClipTetMesh() = default;

    // A node with distance from the level set is below distance_threshold is considered "ON" the level set and will
    // not be shifted during the initial Pass. If a shift diminishes an incidental tet's quality below quality_threshold,
    // the node is not moved. Note that setting this value too high may result in difficult to optimize "slivers" and setting
    // it too low may result in flattened tets that are likewise difficult to optimize.
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
