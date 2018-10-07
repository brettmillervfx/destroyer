//
// Created by Brett Miller on 9/23/18.
//

#pragma once

#include "TetMesh.h"

namespace destroyer {

/*************************************************************************************

 CollapseTetMesh class

 CollapseTetMesh is a specialization of TetMesh used for finding tets of poor quality and
 collapsing those tets along the shortest edge.

*************************************************************************************/


class CollapseTetMesh : public TetMesh
{
public:

    using TetMesh::TetMesh;

    // Uses the base class constructor.
    ~CollapseTetMesh() = default;

    // Collapse all tetrahedra with quality below the prescribed threshold. Because collapsing
    // tetrahedra this way can degrade the quality of adjacent tets, this method runds iteratively
    // until all no tet of low quality are discovered, or max_iters is reached.
    bool Cleanup(Real quality_threshold, int max_iter);

private:


};


}; // namespace destroyer
