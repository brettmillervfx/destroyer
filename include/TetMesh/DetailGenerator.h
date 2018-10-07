//
// Created by Brett Miller on 8/15/18.
//

#pragma once

#include <unordered_map>
#include <GU/GU_Detail.h>

#include "Types.h"

namespace destroyer {

/*************************************************************************************

 DetailGenerator class

 Fills an empty TetMesh with the tetrahedral mesh found in a Houdini detail.

*************************************************************************************/


class DetailGenerator
{
public:

    // Supply the constructor with an instatiated, empty TetMesh and the Houdini detail.
    // Tetrahedral primitives in the detail will be transformed into the TetMesh data structure,
    // as well as all relevant connectivity and topology. However, note that any attributes attached to
    // the detail will not be carried through adn will be lost.
    DetailGenerator(TetMeshPtr tet_mesh, GU_Detail* gdp);

    ~DetailGenerator() = default;

    // Tesselate the bounding box domain.
    void FillTetMesh();

private:

private:
    TetMeshPtr tet_mesh_;
    GU_Detail* gdp_;
    std::unordered_map<Index, TetNodeRef> node_refs_;

};


}; // namespace destroyer
