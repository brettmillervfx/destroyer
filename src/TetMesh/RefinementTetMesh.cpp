//
// Created by Brett Miller on 8/15/18.
//


#include "TetMesh/RefinementTetMesh.h"

#include <iostream>
#include <queue>

#include "TetMesh/Tetrahedron.h"
#include "TetMesh/TetNode.h"
#include "TetMesh/TetEdge.h"
#include "TetMesh/TetFace.h"

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

bool RefinementTetMesh::Cleanup(int max_iterations) {

    auto iter = max_iterations;
    bool done = false;
    while ((iter > 0) & !done) {

        done = true;

        auto lone_count = RefineLoneTets();
        done &= (lone_count==0);

        auto weak_ext_count = RemoveWeakExteriorTets();
        done &= (weak_ext_count==0);

        auto weak_int_count = SplitWeakInteriorEdges();
        done &= (weak_int_count==0);

        auto nm_edge_count = RefineNonManifoldEdges();
        done &= (nm_edge_count==0);

        auto nm_node_count = RefineNonManifoldNodes();
        done &= (nm_node_count==0);

        iter--;
    }

    DeleteUnusedTopology();

    return done;

}

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

void RefinementTetMesh::SplitInteriorEdge(TetrahedronRef tet) {

    // This tet has two boundary face. The edge shared by the remaining faces is interior
    // although both nodes are on the boundary. This configuration is weak and should be
    // split into two elements for simulation.
    if (tet->BoundaryFaceCount()==2) {

        refine_list_.insert(tet);

        for (auto index: {EDGE_0_1, EDGE_0_2, EDGE_0_3, EDGE_1_2, EDGE_2_3, EDGE_1_3}) {

            auto incident_faces = tet->GetFacesIncidentTo(index);
            bool face0_is_interior = !incident_faces[0]->IsBoundary();
            bool face1_is_interior = !incident_faces[1]->IsBoundary();
            if (face0_is_interior & face1_is_interior) {
                tet->SplitEdge(index);
                auto incident_tets = tet->GetEdgeRef(index)->GetIncidentTets();
                for (auto& other_tet: incident_tets)
                    refine_list_.insert(other_tet);
                break;
            }
        }

    }
}

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

int RefinementTetMesh::RefineLoneTets() {

    // Any detached tets should be subdivided to ensure that they are stable enough
    // for FEM simulation.
    int lone_count = 0;

    refine_list_.clear();
    for (auto& tet: tets_) {
        if (tet.first->BoundaryFaceCount() == 4) {
            PrepTetForRedRefinement(tet.first);
            lone_count++;
        }
    }

    SubdivideRefinedTets();

    return lone_count;

}

int RefinementTetMesh::RemoveWeakExteriorTets() {

    // We define a weak exterior tet as one that is only connected to the contiguous mesh
    // on one face. Such Tets are eliminated.
    int weak_ext_count = 0;

    for (auto& tet: tets_) {
        if (tet.first->BoundaryFaceCount() == 3) {
            DeleteTetrahedron(tet.first);
            weak_ext_count++;
        }
    }

    return weak_ext_count;

}

int RefinementTetMesh::SplitWeakInteriorEdges() {

    int weak_int_count = 0;

    refine_list_.clear();
    for (auto& tet: tets_) {
        if (tet.first->BoundaryFaceCount() == 2) {
            SplitInteriorEdge(tet.first);
            weak_int_count++;
        }
    }

    PropagateRefinement();
    SubdivideRefinedTets();

    return weak_int_count;

}

int RefinementTetMesh::RefineNonManifoldEdges() {

    refine_list_.clear();

    // Iterate boundary edges and store refs for any non-manifolds we encounter.
    std::queue<TetEdgeRef> nonmanifold_edges;
    for (auto& edge: edges_)
        if (edge.second->IsNonManifold())
            nonmanifold_edges.push(edge.first);

    if (nonmanifold_edges.empty())
        return false;

    int correction_count = 0;

    // Duplicate non-manifold edges and apply new edge to half the incident tets.
    // Both the old tet and new tet are split.
    while (!nonmanifold_edges.empty()) {
        auto edge = nonmanifold_edges.front();
        nonmanifold_edges.pop();

        // All incident tets will require refinement.
        for (auto tet: edge->GetIncidentTets())
            refine_list_.insert(tet);

        // Create a duplicate edge.
        auto node0 = edge->GetFirstNode();
        auto node1 = edge->GetOtherNode(node0);
        auto duplicate_edge = AddEdge(node0, node1);

        // Get the first boundary face and its single connected tet.
        auto current_face = edge->GetIncidentBoundaryFaces()[0];
        auto current_tet = current_face->GetFirstTet();

        while(true) {

            // Get the face that shares an edge with the face on this tet.
            auto edge_index = current_tet->GetEdgeIndex(edge);
            auto incident_faces = current_tet->GetFacesIncidentTo(edge_index);
            auto next_face = (incident_faces[0] == current_face) ? incident_faces[1] : incident_faces[0];

            // Replace the edge on this tet.
            current_tet->ReplaceEdge(edge_index, duplicate_edge);

            // If the next face is boundary, we're done.
            if (next_face->IsBoundary())
                break;

            // Otherwise, get the next tet attached to that face.
            current_tet = next_face->GetOtherTet(current_tet);
            current_face = next_face;

        }

        // Split both the old edge and the new duplicate.
        auto mid = edge->MidpointPosition();
        edge->SetMidpoint(AddNode(mid[0], mid[1], mid[2]));
        duplicate_edge->SetMidpoint(AddNode(mid[0], mid[1], mid[2]));

        correction_count++;

    }

    // We can directly refine the incident tets without bothering to propagate since all
    // subdivided tets are "green".
    SubdivideRefinedTets();

    return correction_count;

}

int RefinementTetMesh::RefineNonManifoldNodes() {

    // Iterate nodes and store refs for any non-manifolds we encounter.
    std::queue<TetNodeRef> nonmanifold_nodes;
    for (auto& node: nodes_) {
        if (node->IsNonManifold())
            nonmanifold_nodes.push(node.get());
    }

    if (nonmanifold_nodes.empty()) {
        return false;
    }

    int correction_count = 0;

    // Replace non-manifold node with a duplicate on one of the edge rings.
    while (!nonmanifold_nodes.empty()) {

        auto node = nonmanifold_nodes.front();
        nonmanifold_nodes.pop();

        // Collect the tets attached to the ring edge
        auto edge_ring = node->GetFirstEdgeRing();
        std::vector<TetrahedronRef> ring_tets;
        for (auto tet: node->GetIncidentTets()) {
            for (auto& edge: edge_ring) {
                if (tet->GetEdgeIndex(edge) != NO_EDGE) {
                    ring_tets.push_back(tet);
                    break;
                }
            }
        }

        std::vector<TetrahedronRef> expanded_search;
        for (auto& tet: node->GetIncidentTets()) {

            // If tet is not already selected...
            if (std::find(ring_tets.begin(), ring_tets.end(), tet) == ring_tets.end()) {
                // ...and if shares and edge or face with a ring tet...
                for (auto& ring_tet: ring_tets) {
                    if (tet->SharesFaceWith(ring_tet) | tet->SharesEdgeWith(ring_tet)) {
                        expanded_search.push_back(tet);
                        break;
                    }
                }
            }
        }
        ring_tets.insert( ring_tets.end(), expanded_search.begin(), expanded_search.end() );

        if (ring_tets.size() == node->GetIncidentTets().size()) {

            // Infrequent case: the node is a pinch point between two surfaces.
            // We assume that this is actually a hole that hasn't been captured due to resolution.
            // So we delete all tets that use the node.
            for (auto& tet: node->GetIncidentTets())
                DeleteTetrahedron(tet);

        } else {

            // Generally, we eliminate all the tets on one side of the non-manifold node.
            for (auto& tet: ring_tets)
                DeleteTetrahedron(tet);

        }

        correction_count++;
    }

    return correction_count;

}

}; // namespace destroyer