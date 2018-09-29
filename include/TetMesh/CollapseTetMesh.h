//
// Created by Brett Miller on 9/23/18.
//

#pragma once

#include "TetMesh.h"

namespace destroyer {

class CollapseTetMesh : public TetMesh
{
public:

    using TetMesh::TetMesh;

    // Uses the base class constructor.
    ~CollapseTetMesh() = default;

    bool Cleanup(Real quality_threshold, int max_iter);

private:


};


}; // namespace destroyer
