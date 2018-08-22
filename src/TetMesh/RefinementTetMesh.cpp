//
// Created by Brett Miller on 8/15/18.
//


#include "TetMesh/RefinementTetMesh.h"

#include <iostream>

#include "TetMesh/Tetrahedron.h"
#include "TetMesh/TetNode.h"
#include "TetMesh/TetEdge.h"

namespace destroyer {


void RefinementTetMesh::RefineIdGroup(const std::set<Index> id_group) {

    // Add references to all tets in the id group to the refine list.
    // While doing so, split edges and add adjacent tets.
    refine_list_.clear();
    for (auto& tet: tets_) {
        if (id_group.find(tet.second->Id()) != id_group.end())
            PrepTetForRedRefinement(tet.first);
    }

    PropagateRefinement();
    SubdivideRefinedTets();

}

/*
void RefinementTetMesh::Cleanup(int culldepth) {

    // The TetMesh must abide the following rules:
    //      * the boundary must be a manifold
    //      * no tetrahedron may have all four nodes on the boundary
    //      * and no interior edge may connect two boundary nodes.

    refine_list_.clear();
    FlagAllBoundaryNodes();

    for (auto& tet: tet_list_) {
        auto boundary_node_count = tet.second->BoundaryNodeCount();

        if (boundary_node_count == 2) {

            SplitInteriorEdge(tet.first);

        } else if (boundary_node_count == 4) {

            tet.second->ClassifySplitEdgeConfiguration();
            PrepTetForRedRefinement(tet.first);

        }

    }

    PropagateRefinement();

    SubdivideRefinedTets();

    CullOutsideTets(culldepth);

    RepairNonManifoldBoundary();
}
*/
void RefinementTetMesh::PropagateRefinement() {

    // Find all potential Red tets in the refinement list. Split their edges and add adjacent tets.
    // Continue to do this until we find no potential Reds.
    bool found_potential_reds = false;
    do {
        found_potential_reds = false;
        for (auto& tet: refine_list_) {
            tet->ClassifySplitEdgeConfiguration();
            if (tet->RequiresGreenPrep()) {
                found_potential_reds = true;
                PrepTetForGreenRefinement(tet);
            }
            if (tet->IsPotentialRed()) {
                found_potential_reds = true;
                PrepTetForRedRefinement(tet);
            }
        }
    } while (found_potential_reds);

}

/*
void RefinementTetMesh::SplitInteriorEdge(TetrahedronRef tet) {

    // Test if the edge is interior. If so, it will need to be split.
    // If the edge is not interior, there will be two incident tets that contains
    // both the boundary nodes and a third boundary node.

    // Get the two nodes comprising the interior edge.
    TetNodeRef ref0 = nullptr;
    TetNodeRef ref1 = nullptr;
    for (Index i = 0; i<4; i++) {
        auto node = tet->GetNodeRef(i);
        if (node->IsBoundary()) {
            if (ref0 == nullptr)
                ref0 = node;
            else
                ref1 = node;
        }
    }

    // For any tet that contains both nodes, are there three boundary nodes?
    bool is_interior = true;
    ref0->ResetConnectedTetsIterator();
    auto connected_tet = ref0->NextConnectedTet();
    while (connected_tet != nullptr) {
        if (connected_tet->HasNode(ref1)){
            if (connected_tet->BoundaryNodeCount() > 2)
                is_interior = false;
        }
        connected_tet = ref0->NextConnectedTet();
    }

    if (is_interior) {
        SplitEdge(ref0, ref1);
        ref0->ResetConnectedTetsIterator();
        auto connected_tet = ref0->NextConnectedTet();
        while (connected_tet != nullptr) {
            connected_tet->ClassifySplitEdgeConfiguration();
            refine_list_.insert(connected_tet);
            connected_tet = ref0->NextConnectedTet();
        }
    }
}
*/
void RefinementTetMesh::SubdivideTetrahedron(TetrahedronRef tet) {

    // For convenience, capture all the tet's original nodes and all the edge split midpoints.
    std::array<TetNodeRef,4> original_nodes;
    for (Index i = 0; i < 4; i++) {
        original_nodes[i] = tet->GetNodeRef(i);
    }
    std::map<Index, TetNodeRef> midpoint;
    midpoint[01] = midpoint[10] = tet->GetEdgeRef(EDGE_0_1)->Midpoint();
    midpoint[02] = midpoint[20] = tet->GetEdgeRef(EDGE_0_2)->Midpoint();
    midpoint[03] = midpoint[30] = tet->GetEdgeRef(EDGE_0_3)->Midpoint();
    midpoint[12] = midpoint[21] = tet->GetEdgeRef(EDGE_1_2)->Midpoint();
    midpoint[13] = midpoint[31] = tet->GetEdgeRef(EDGE_1_3)->Midpoint();
    midpoint[23] = midpoint[32] = tet->GetEdgeRef(EDGE_2_3)->Midpoint();

    // Produce the 4 corner tetrahedra.
    for (Index i = 0; i < 4; i++) {
        auto index0 = (i*10) + ((i+1)%4);
        auto index1 = (i*10) + ((i+2)%4);
        auto index2 = (i*10) + ((i+3)%4);
        AddTetrahedron(original_nodes[i], midpoint[index0], midpoint[index1], midpoint[index2], tet->Id());
    }

    // TODO first calculate the interior diagonals and use the shortest to determine the interior topology.
    // Produce the 4 interior tetrahedra.
    AddTetrahedron(midpoint[03], midpoint[23], midpoint[01], midpoint[02], tet->Id());
    AddTetrahedron(midpoint[03], midpoint[23], midpoint[13], midpoint[01], tet->Id());
    AddTetrahedron(midpoint[12], midpoint[13], midpoint[01], midpoint[23], tet->Id());
    AddTetrahedron(midpoint[12], midpoint[01], midpoint[02], midpoint[23], tet->Id());

    // Find and delete the original.
    DeleteTetrahedron(tet);

}

void RefinementTetMesh::PrepTetForGreenRefinement(TetrahedronRef tet) {

    // Precondition RefinementTetMesh for red/green hierarchy identification.
    // Generally this means turning some two-splits into three-splits to avoid regular subdivision.

    // We know we have two split edges that are adjacent. We need to find the third edge that
    // completes the face and split it.
    auto third_edge = tet->ThirdEdgeToCompleteSplitFace();

    if (third_edge != NO_EDGE) {
        tet->SplitEdge(third_edge);
        tet->ClassifySplitEdgeConfiguration();

        auto incident_tets = tet->GetEdgeRef(third_edge)->GetIncidentTets();
        for (auto& tet: incident_tets)
            refine_list_.insert(tet);
    }

}

void RefinementTetMesh::PrepTetForRedRefinement(TetrahedronRef tet) {

    // Split all edges of the tetrahedron.
    // Add all neighbor tets to refinement list, as they may potentially require red or green refinement.
    // This tet is connected to the edge, so it will be picked up implicitly.
    for (auto index: {EDGE_0_1, EDGE_0_2, EDGE_0_3, EDGE_1_2, EDGE_2_3, EDGE_1_3}) {

        tet->SplitEdge(index);

        auto incident_tets = tet->GetEdgeRef(index)->GetIncidentTets();
        for (auto& tet: incident_tets)
            refine_list_.insert(tet);
    }

}

void RefinementTetMesh::SubdivideRefinedTets() {

    for (auto& tet: refine_list_) {

        tet->ClassifySplitEdgeConfiguration();

        switch (tet->GetSplitEdgeCount()) {

            // No split edges? Don't do anything.
            case 0:
                break;

            case 1:
                IrregularSubdivideTetrahedronOne(tet);
                break;

            case 2:
                IrregularSubdivideTetrahedronTwo(tet);
                break;

            case 3:
                IrregularSubdivideTetrahedronThree(tet);
                break;

            default:
                SubdivideTetrahedron(tet);
                break;

        }

    }

}

void RefinementTetMesh::IrregularSubdivideTetrahedronOne(TetrahedronRef tet) {

    Index node0, node1, node2, node3;

    if (tet->IsEdgeSplit(EDGE_0_1)) {
        node0 = 0; node1 = 1; node2 = 2; node3 = 3;
    } else if (tet->IsEdgeSplit(EDGE_0_2)) {
        node0 = 0; node1 = 2; node2 = 3; node3 = 1;
    } else if (tet->IsEdgeSplit(EDGE_0_3)) {
        node0 = 0; node1 = 3; node2 = 1; node3 = 2;
    } else if (tet->IsEdgeSplit(EDGE_1_2)) {
        node0 = 1; node1 = 2; node2 = 0; node3 = 3;
    } else if (tet->IsEdgeSplit(EDGE_2_3)) {
        node0 = 2; node1 = 3; node2 = 0; node3 = 1;
    } else if (tet->IsEdgeSplit(EDGE_1_3)) {
        node0 = 3; node1 = 1; node2 = 0; node3 = 2;
    } else {
        node0 = 0; node1 = 1; node2 = 2; node3 = 3;
    }

    // Build tets on either side of the split.
    auto midpt0 = tet->SplitEdge(node0, node1);

    AddTetrahedron(tet->GetNodeRef(node0), midpt0, tet->GetNodeRef(node2), tet->GetNodeRef(node3), tet->Id());
    AddTetrahedron(tet->GetNodeRef(node1), midpt0, tet->GetNodeRef(node3), tet->GetNodeRef(node2), tet->Id());

    // Find and delete the original.
    DeleteTetrahedron(tet);

}

void RefinementTetMesh::IrregularSubdivideTetrahedronTwo(TetrahedronRef tet) {

    // TODO there is no doubt a way to get better results if we expand to 6 options, ie. we consider which direction the split edge goes.
    // Subdivide for tets with two broken edges, opposite one another. This method assumes that
    // the input is prefiltered and that the edges are indeed opposite.
    Index node0, node1, node2, node3;

    if (tet->IsEdgeSplit(EDGE_0_1)) {
        node0 = 0;
        node1 = 1;
        node2 = 2;
        node3 = 3;
    } else if (tet->IsEdgeSplit(EDGE_0_2)) {
        node0 = 0;
        node1 = 2;
        node2 = 3;
        node3 = 1;
    } else  { // EDGE_0_3
        node0 = 0;
        node1 = 3;
        node2 = 1;
        node3 = 2;
    }

    // Build the 4 tets that quarter the tetrahedron.

    auto midpt0 = tet->SplitEdge(node0, node1);
    auto midpt1 = tet->SplitEdge(node2, node3);

    AddTetrahedron(tet->GetNodeRef(node0), midpt0, tet->GetNodeRef(node2), midpt1, tet->Id());
    AddTetrahedron(tet->GetNodeRef(node0), midpt1, tet->GetNodeRef(node3), midpt0, tet->Id());
    AddTetrahedron(tet->GetNodeRef(node1), midpt0, tet->GetNodeRef(node3), midpt1, tet->Id());
    AddTetrahedron(tet->GetNodeRef(node1), midpt0, midpt1, tet->GetNodeRef(node2), tet->Id());

    // Find and delete the original.
    DeleteTetrahedron(tet);
}

void RefinementTetMesh::IrregularSubdivideTetrahedronThree(TetrahedronRef tet) {

    // Subdivide tets with three broken edges, all on the same face.
    Index apex, node0, node1, node2;

    if (tet->IsEdgeSplit(EDGE_0_1) & tet->IsEdgeSplit(EDGE_1_2) & tet->IsEdgeSplit(EDGE_0_2)) {
        apex = 3;
        node0 = 0;
        node1 = 1;
        node2 = 2;
    } else if (tet->IsEdgeSplit(EDGE_0_2) & tet->IsEdgeSplit(EDGE_0_3) & tet->IsEdgeSplit(EDGE_2_3)) {
        apex = 1;
        node0 = 0;
        node1 = 2;
        node2 = 3;
    } else if (tet->IsEdgeSplit(EDGE_0_1) & tet->IsEdgeSplit(EDGE_0_3) & tet->IsEdgeSplit(EDGE_1_3)) {
        apex = 2;
        node0 = 0;
        node1 = 1;
        node2 = 3;
    } else if (tet->IsEdgeSplit(EDGE_1_3) & tet->IsEdgeSplit(EDGE_1_2) & tet->IsEdgeSplit(EDGE_2_3)) {
        apex = 0;
        node0 = 1;
        node1 = 2;
        node2 = 3;
    } else return;

    // Build the 4 tets that quarter the tetrahedron.
    auto midpt0 = tet->SplitEdge(node0, node1);
    auto midpt1 = tet->SplitEdge(node1, node2);
    auto midpt2 = tet->SplitEdge(node2, node0);

    AddTetrahedron(tet->GetNodeRef(apex), tet->GetNodeRef(node0), midpt0, midpt2, tet->Id());
    AddTetrahedron(tet->GetNodeRef(apex), tet->GetNodeRef(node1), midpt1, midpt0, tet->Id());
    AddTetrahedron(tet->GetNodeRef(apex), tet->GetNodeRef(node2), midpt2, midpt1, tet->Id());
    AddTetrahedron(tet->GetNodeRef(apex), midpt0, midpt1, midpt2, tet->Id());

    // Find and delete the original.
    DeleteTetrahedron(tet);

}
/*
void RefinementTetMesh::RepairNonManifoldBoundary() {

    // NonManifold boundaries occur when a boundary node is surrounded by
    // two or more distinct vertex rings.
    FlagAllBoundaryNodes();
    for (auto& node: node_list_) {

    }

}
*/

}; // namespace destroyer