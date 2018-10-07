//
// Created by Brett Miller on 9/23/18.
//

#include "TetMesh/ClipTetMesh.h"

#include "TetMesh/Tetrahedron.h"
#include "TetMesh/TetNode.h"
#include "TetMesh/TetEdge.h"
#include "TetMesh/VDBSampler.h"


namespace destroyer {

void ClipTetMesh::Clip(Real quality_threshold, Real distance_threshold) {

    ShiftNodesToBoundary(distance_threshold, quality_threshold);

    SplitBoundaryTets(distance_threshold);

}

void ClipTetMesh::FlagInteriorNodes(Real distance_threshold) {

    // Set the SDFFlag for all nodes. Nodes that closer to the level set than
    // distance_threshold are considered "ON" the level set.

    for (auto& node: nodes_) {

        auto pos = node->Position();
        auto distance = sampler_->Sample(pos[0], pos[1], pos[2]);
        node->SetSdf(distance);

        if (abs(distance)<distance_threshold) {
            node->SetSDFFlag(ON_SDF);
        } else if (distance < 0.0) {
            node->SetSDFFlag(IN_SDF);
        } else
            node->SetSDFFlag(OUT_SDF);

    }
}

void ClipTetMesh::ShiftNodesToBoundary(Real distance_threshold, Real quality_threshold) {

    FlagInteriorNodes(distance_threshold);

    // Sort nodes in ascending distance order.
    nodes_.sort([](const TetNodePtr & a, const TetNodePtr & b) { return abs(a->Sdf()) < abs(b->Sdf()); });

    for (auto& node: nodes_) {

        if (node->SdfFlag() != ON_SDF) {

            // A node is a move candidate if it is connected to another node with the opposite IN/OUT flag.
            auto opposite = (node->SdfFlag() == IN_SDF) ? OUT_SDF : IN_SDF;
            if (node->ConnectedToSdfFlag(opposite)) {

                // Move node to Level Set boundary.
                auto current_pos = node->Position();
                auto sdf_gradient = sampler_->SampleGradient(current_pos[0], current_pos[1], current_pos[2]);
                if(sdf_gradient.length2() > 0.0) {
                    auto boundary_pos = current_pos - (sdf_gradient / sdf_gradient.length()) * node->Sdf();
                    node->SetPosition(boundary_pos);

                    // If the quality has degraded substantially, move it back.
                    if (node->CalculateLocalQuality() < quality_threshold)
                        node->SetPosition(current_pos);
                    else
                        node->SetSDFFlag(ON_SDF);
                }
            }
        }

    }

}

void ClipTetMesh::SplitBoundaryTets(Real distance_threshold) {

    std::vector<TetrahedronRef> delete_list;
    std::vector<TetrahedronRef> split_list;

    for (auto& tet: tets_) {

        auto in_count = tet.first->CountInNodes();
        auto on_count = tet.first->CountOnNodes();
        auto out_count = tet.first->CountOutNodes();

        if (in_count == 0) {
            delete_list.push_back(tet.first);
        } else {

            if (in_count < 4) {
                if (out_count > 0) {
                    PrepSplitBoundaryTet(tet.first, in_count, on_count, out_count);
                    split_list.push_back(tet.first);
                }
            }
        }

    }

    for (auto& tet: delete_list) {
        DeleteTetrahedron(tet);
    }

    for (auto& tet: split_list) {
        ProcessSplitBoundaryTet(tet);
    }

}

void ClipTetMesh::PrepSplitBoundaryTet(TetrahedronRef tet, int in_count, int on_count, int out_count) {

    if ( in_count==1 & out_count==1 & on_count==2) {
        // Case 1
        tet->SetBoundarySplitCase(1);
    } else if ( in_count==1 & out_count==2 & on_count==1) {
        // Case 2
        tet->SetBoundarySplitCase(2);
    } else if ( in_count==2 & out_count==1 & on_count==1) {
        // Case 3
        tet->SetBoundarySplitCase(3);
    } else if ( in_count==1 & out_count==3 & on_count==0) {
        // Case 4
        tet->SetBoundarySplitCase(4);
    } else if ( in_count==3 & out_count==1 & on_count==0) {
        // Case 5
        tet->SetBoundarySplitCase(5);
    } else if ( in_count==2 & out_count==2 & on_count==0) {
        // Case 6
        tet->SetBoundarySplitCase(6);
    }

    tet->SplitBoundaryCrossingEdges();

}

void ClipTetMesh::ProcessSplitBoundaryTet(TetrahedronRef tet) {

    switch (tet->BoundarySplitCase()) {

        case 0:
            break;

        case 1:
            ProcessCase1Split(tet);
            break;

        case 2:
            ProcessCase2Split(tet);
            break;

        case 3:
            ProcessCase3Split(tet);
            break;

        case 4:
            ProcessCase4Split(tet);
            break;

        case 5:
            ProcessCase5Split(tet);
            break;

        case 6:
            ProcessCase6Split(tet);
            break;

    }
}

void ClipTetMesh::ProcessCase1Split(TetrahedronRef tet) {

    auto in_node = tet->InNodes()[0];
    auto out_node = tet->OutNodes()[0];
    auto on_nodes = tet->OnNodes();

    auto edge = in_node->GetEdgeTo(out_node);
    auto mid_node = edge->Midpoint();

    AddTetrahedron(in_node, mid_node, on_nodes[0], on_nodes[1]);
    DeleteTetrahedron(tet);

}

void ClipTetMesh::ProcessCase2Split(TetrahedronRef tet) {

    auto in_node = tet->InNodes()[0];
    auto out_nodes = tet->OutNodes();
    auto on_node = tet->OnNodes()[0];

    std::vector<TetNodeRef> mid_nodes;
    for (auto& out_node: out_nodes) {
        auto edge = in_node->GetEdgeTo(out_node);
        mid_nodes.push_back(edge->Midpoint());
    }

    AddTetrahedron(in_node, on_node, mid_nodes[0], mid_nodes[1]);
    DeleteTetrahedron(tet);

}

void ClipTetMesh::ProcessCase3Split(TetrahedronRef tet) {

    auto in_nodes = tet->InNodes();
    auto out_node = tet->OutNodes()[0];
    auto on_node = tet->OnNodes()[0];

    std::vector<TetNodeRef> mid_nodes;
    for (auto& in_node: in_nodes) {
        auto edge = in_node->GetEdgeTo(out_node);
        mid_nodes.push_back(edge->Midpoint());
    }

    // This configuration produces a quadrilateral face, which we sill split along the shortest diagonal.
    auto diagonal0 = (mid_nodes[0]->Position() - in_nodes[1]->Position()).length2();
    auto diagonal1 = (mid_nodes[1]->Position() - in_nodes[0]->Position()).length2();
    if (diagonal0<diagonal1) {
        AddTetrahedron(mid_nodes[0], in_nodes[1], mid_nodes[1], on_node);
        AddTetrahedron(mid_nodes[0], in_nodes[1], in_nodes[0], on_node);
    } else {
        AddTetrahedron(mid_nodes[0], in_nodes[0], mid_nodes[1], on_node);
        AddTetrahedron(mid_nodes[1], in_nodes[1], in_nodes[0], on_node);
    }

    DeleteTetrahedron(tet);

}

void ClipTetMesh::ProcessCase4Split(TetrahedronRef tet) {

    auto in_node = tet->InNodes()[0];
    auto out_nodes = tet->OutNodes();

    std::vector<TetNodeRef> mid_nodes;
    for (auto& out_node: out_nodes) {
        auto edge = in_node->GetEdgeTo(out_node);
        mid_nodes.push_back(edge->Midpoint());
    }

    AddTetrahedron(in_node, mid_nodes[0], mid_nodes[1], mid_nodes[2]);
    DeleteTetrahedron(tet);

}

void ClipTetMesh::ProcessCase5Split(TetrahedronRef tet) {

    auto in_nodes = tet->InNodes();
    auto out_node = tet->OutNodes()[0];

    std::vector<TetNodeRef> mid_nodes;
    for (auto& in_node: in_nodes) {
        auto edge = in_node->GetEdgeTo(out_node);
        mid_nodes.push_back(edge->Midpoint());
    }

    // This configuration produces three quadrilateral faces, which we will split along the shortest diagonals.
    auto diagonal_0_1 = (mid_nodes[0]->Position() - in_nodes[1]->Position()).length2();
    auto diagonal_0_2 = (mid_nodes[0]->Position() - in_nodes[2]->Position()).length2();
    auto diagonal_1_0 = (mid_nodes[1]->Position() - in_nodes[0]->Position()).length2();
    auto diagonal_1_2 = (mid_nodes[1]->Position() - in_nodes[2]->Position()).length2();
    auto diagonal_2_0 = (mid_nodes[1]->Position() - in_nodes[0]->Position()).length2();
    auto diagonal_2_1 = (mid_nodes[2]->Position() - in_nodes[1]->Position()).length2();

    if ((diagonal_0_1 < diagonal_1_0) & (diagonal_0_2 < diagonal_2_0)) {

        AddTetrahedron(in_nodes[0], in_nodes[1], in_nodes[2], mid_nodes[0]);
        if (diagonal_1_2 < diagonal_2_1) {
            AddTetrahedron(in_nodes[2], mid_nodes[0], mid_nodes[1], mid_nodes[2]);
            AddTetrahedron(in_nodes[1], mid_nodes[1], in_nodes[2], mid_nodes[0]);
        } else {
            AddTetrahedron(in_nodes[1], mid_nodes[1], mid_nodes[2], mid_nodes[0]);
            AddTetrahedron(in_nodes[2], in_nodes[1], mid_nodes[2],  mid_nodes[0]);
        }

    } else if ((diagonal_1_0 < diagonal_0_1) & (diagonal_1_2 < diagonal_2_1)) {

        AddTetrahedron(in_nodes[0], in_nodes[1], in_nodes[2], mid_nodes[1]);
        if (diagonal_0_2 < diagonal_2_0) {
            AddTetrahedron(in_nodes[2], mid_nodes[1], mid_nodes[0], mid_nodes[2]);
            AddTetrahedron(in_nodes[0], mid_nodes[0], in_nodes[2], mid_nodes[1]);
        } else {
            AddTetrahedron(in_nodes[0], mid_nodes[1], mid_nodes[2], mid_nodes[0]);
            AddTetrahedron(in_nodes[2], mid_nodes[2], in_nodes[0], mid_nodes[1]);
        }

    } else {

        AddTetrahedron(in_nodes[0], in_nodes[1], in_nodes[2], mid_nodes[2]);
        if (diagonal_1_0 < diagonal_0_1) {
            AddTetrahedron(in_nodes[0], mid_nodes[2], mid_nodes[1], mid_nodes[0]);
            AddTetrahedron(in_nodes[1], mid_nodes[1], in_nodes[0], mid_nodes[2]);
        } else {
            AddTetrahedron(in_nodes[1], mid_nodes[2], mid_nodes[0], mid_nodes[1]);
            AddTetrahedron(in_nodes[0], mid_nodes[0], in_nodes[1], mid_nodes[2]);
        }

    }


    DeleteTetrahedron(tet);

}

void ClipTetMesh::ProcessCase6Split(TetrahedronRef tet) {

    auto in_nodes = tet->InNodes();
    auto out_nodes = tet->OutNodes();

    auto edge_0_0 = in_nodes[0]->GetEdgeTo(out_nodes[0]);
    auto mid_node_0_0 = edge_0_0->Midpoint();
    auto edge_0_1 = in_nodes[0]->GetEdgeTo(out_nodes[1]);
    auto mid_node_0_1 = edge_0_1->Midpoint();
    auto edge_1_0 = in_nodes[1]->GetEdgeTo(out_nodes[0]);
    auto mid_node_1_0 = edge_1_0->Midpoint();
    auto edge_1_1 = in_nodes[1]->GetEdgeTo(out_nodes[1]);
    auto mid_node_1_1 = edge_1_1->Midpoint();

    auto center_position = (mid_node_0_0->Position() +
                            mid_node_0_1->Position() +
                            mid_node_1_0->Position() +
                            mid_node_1_1->Position()) / 4.0;
    auto center_node = AddNode(center_position[0], center_position[1], center_position[2]);

    AddTetrahedron(in_nodes[0], mid_node_0_0, mid_node_0_1, center_node);
    AddTetrahedron(in_nodes[1], mid_node_1_0, mid_node_1_1, center_node);

    auto diagonal_0_10 = (mid_node_1_0->Position() - in_nodes[0]->Position()).length2();
    auto diagonal_1_00 = (mid_node_0_0->Position() - in_nodes[1]->Position()).length2();
    if (diagonal_0_10 < diagonal_1_00) {
        AddTetrahedron(in_nodes[0], mid_node_1_0, mid_node_0_0, center_node);
        AddTetrahedron(in_nodes[1], mid_node_1_0, in_nodes[0], center_node);
    } else {
        AddTetrahedron(in_nodes[0], in_nodes[1], mid_node_0_0, center_node);
        AddTetrahedron(in_nodes[1], mid_node_0_0, mid_node_1_0, center_node);
    }

    auto diagonal_0_11 = (mid_node_1_1->Position() - in_nodes[0]->Position()).length2();
    auto diagonal_1_01 = (mid_node_0_1->Position() - in_nodes[1]->Position()).length2();
    if (diagonal_0_11 < diagonal_1_01) {
        AddTetrahedron(in_nodes[0], mid_node_1_1, mid_node_0_1, center_node);
        AddTetrahedron(in_nodes[1], mid_node_1_1, in_nodes[0], center_node);
    } else {
        AddTetrahedron(in_nodes[0], in_nodes[1], mid_node_0_1, center_node);
        AddTetrahedron(in_nodes[1], mid_node_0_1, mid_node_1_1, center_node);
    }

    DeleteTetrahedron(tet);

}

}; // namespace destroyer