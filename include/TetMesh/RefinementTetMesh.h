//
// Created by Brett Miller on 8/15/18.
//

#pragma once

#include "TetMesh/TetMesh.h"

#include <set>


namespace destroyer {

class RefinementTetMesh : public TetMesh {
public:

    using TetMesh::TetMesh;

    ~RefinementTetMesh() = default;

    // Apply Red Green Hierarchical Refinement to tets in an ID group.
    void RefineIdGroup(const std::set<Index> id_group);

    // Ensure that topology will be wel behaved in an FEM solve.
    // void Cleanup(int culldepth=2);

private:
    void PropagateRefinement();

    // void SplitInteriorEdge(TetrahedronRef tet);

    void PrepTetForGreenRefinement(TetrahedronRef tet);

    void PrepTetForRedRefinement(TetrahedronRef tet);

    void SubdivideRefinedTets();

    void SubdivideTetrahedron(TetrahedronRef tet);

    void IrregularSubdivideTetrahedronOne(TetrahedronRef tet);

    void IrregularSubdivideTetrahedronTwo(TetrahedronRef tet);

    void IrregularSubdivideTetrahedronThree(TetrahedronRef tet);

    //void RepairNonManifoldBoundary();

private:
    std::set<TetrahedronRef> refine_list_;

};

}; // namespace destroyer
