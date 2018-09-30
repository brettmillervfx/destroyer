//
// Created by Brett Miller on 8/15/18.
//

#include "TetMesh/TetMeshToHoudiniDetail.h"

#include <GEO/GEO_PrimTetrahedron.h>
#include <GEO/GEO_PrimPoly.h>

#include "TetMesh/Tetrahedron.h"
#include "TetMesh/TetMesh.h"
#include "TetMesh/TetNode.h"
#include "TetMesh/TetFace.h"


namespace destroyer {


TetMeshToHoudiniDetail::TetMeshToHoudiniDetail(TetMeshPtr tet_mesh, GU_Detail *gdp) {

    tet_mesh_ = tet_mesh;
    gdp_ = gdp;

}

void TetMeshToHoudiniDetail::Convert() {

    // Clear out the detail.
    gdp_->clearAndDestroy();

    // Write points into detail and register generated indices.
    ConvertPoints();

    // Write tetrahedral primitives into detail.
    ConvertTetrahedra();

}

void TetMeshToHoudiniDetail::ConvertBoundary() {

    // Clear out the detail.
    gdp_->clearAndDestroy();

    // Write boundary points into detail and register generated indices.
    ConvertPoints(true);

    // Write boundary face primitives into detail.
    ConvertBoundaryFaces();

}

void TetMeshToHoudiniDetail::ConvertPoints(bool boundary_only) {

    // Put all of the TetMesh's nodes into the detail as points.
    tet_mesh_->ResetNodeIterator();
    auto node = tet_mesh_->NextNode();
    while(node != nullptr) {

        if (!boundary_only | node->IsBoundary()) {
            auto index = gdp_->appendPoint();
            Vec3 position = node->Position();
            gdp_->setPos3(index, position[0], position[1], position[2]);

            // Register the point offset with the TetNode.
            node->SetId(index);
        }

        node = tet_mesh_->NextNode();
    }

}

void TetMeshToHoudiniDetail::ConvertTetrahedra() {

    // The nodes should already be points in the detail. Use these points as
    // vertices for the newly created tetrahedron primitives.
    tet_mesh_->ResetTetIterator();
    auto tet = tet_mesh_->NextTet();
    while(tet != nullptr) {
        auto offset0 = tet->GetNodeRef(0)->Id();
        auto offset1 = tet->GetNodeRef(1)->Id();
        auto offset2 = tet->GetNodeRef(2)->Id();
        auto offset3 = tet->GetNodeRef(3)->Id();

        auto hou_tet = GEO_PrimTetrahedron::build(gdp_,false);
        hou_tet->setVertexPoint(0, offset0);
        hou_tet->setVertexPoint(1, offset1);
        hou_tet->setVertexPoint(2, offset2);
        hou_tet->setVertexPoint(3, offset3);

        tet = tet_mesh_->NextTet();
    }

}

void TetMeshToHoudiniDetail::ConvertBoundaryFaces() {

    // The nodes should already be points in the detail. Use these points as
    // vertices for the newly created polygon primitives.
    tet_mesh_->ResetFaceIterator();
    auto face = tet_mesh_->NextFace();
    while(face != nullptr) {
        if (face->IsBoundary()) {

            face->CorrectWinding();

            auto prim = (GEO_PrimPoly *)gdp_->appendPrimitive(GA_PRIMPOLY);
            prim->setSize(0);
            for (uint j=0; j<3; j++)
                prim->appendVertex(face->GetNodeRef(j)->Id());
            prim->close();

        }

        face = tet_mesh_->NextFace();
    }

}


}; // namespace destroyer