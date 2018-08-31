//
// Created by Brett Miller on 8/15/18.
//

#pragma once

#include "TetMesh.h"
#include <queue>


namespace destroyer {

class CompressionTetMesh : public TetMesh
{
public:

    using TetMesh::TetMesh;
    ~CompressionTetMesh() = default;

    // Sort the nodelist by depth from the surface.
    void SortNodesByDepth();

    Real QualityMetric(TetNodeRef node) const;

private:
    Real InteriorQualityMetric(TetNodeRef node) const;


};


}; // namespace destroyer
