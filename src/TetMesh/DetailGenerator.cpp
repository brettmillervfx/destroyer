//
// Created by Brett Miller on 8/15/18.
//

#include "TetMesh/DetailGenerator.h"

#include <array>
#include <GEO/GEO_PrimTetrahedron.h>

#include "TetMesh/TetMesh.h"
#include "TetMesh/TetNode.h"


namespace destroyer {


DetailGenerator::DetailGenerator(TetMeshPtr tet_mesh, GU_Detail *gdp) {

    tet_mesh_ = tet_mesh;
    gdp_ = gdp;

    if (!tet_mesh_->IsEmpty())
        tet_mesh_->TearDown();

}

void DetailGenerator::FillTetMesh() {

    // Copy all points from the detail.
    UT_Vector3 p;

    GA_Offset start, end;
    for (GA_Iterator it(gdp_->getPointRange()); it.blockAdvance(start, end);) {
        for (GA_Offset ptoff = start; ptoff < end; ++ptoff) {
            p = gdp_->getPos3(ptoff);
            TetNodeRef node_ref = tet_mesh_->AddNode(p[0], p[1], p[2]);
            node_ref->SetId(ptoff);
            node_refs_[ptoff] = node_ref;
        }
    }

    // Copy only tetrahedra from the detail.
    const GEO_Primitive *prim;
    std::array<TetNodeRef,4> vertices;
    GA_FOR_ALL_PRIMITIVES(gdp_, prim) {
        if (prim->getTypeId() == GEO_PRIMTETRAHEDRON) {
            auto tet = dynamic_cast<const GEO_PrimTetrahedron *>(prim);
            for (auto i=0; i<4; i++){
                auto offset = tet->getPointOffset(i);
                vertices[i] = node_refs_[offset];
            }
            tet_mesh_->AddTetrahedron(vertices[0], vertices[1], vertices[2], vertices[3], tet->getMapOffset());
        }
    }

    // Clean up
    tet_mesh_->DeleteUnusedTopology();

}


}; // namespace destroyer