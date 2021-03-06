//
// Created by Brett Miller on 9/23/18.
//

#include "TetMesh/CollapseTetMesh.h"

#include "TetMesh/Tetrahedron.h"
#include "TetMesh/TetNode.h"
#include "TetMesh/TetEdge.h"
#include "TetMesh/TetFace.h"

#include <list>
#include <set>
#include <iostream>


namespace destroyer {

bool CollapseTetMesh::Cleanup(Real quality_threshold, int max_iter) {

    // Doe will be true when no low quality tets are discoveres.
    bool done = false;

    // We set iter to the maximum interation count then count down to 0.
    auto iter = max_iter;
    while ((iter > 0) & !done) {

        done = false;

        // We use the edge length ratio as a quality hueristic. Collect all tets that currently
        // exhibit low quality according to this measure.
        std::list<TetrahedronRef> tet_list;
        for (auto& tet: tets_) {
            if (tet.first->EdgeLengthRatio() < quality_threshold)
                tet_list.push_back(tet.first);
        }

        if (tet_list.empty())
            done = true;
        else {

            std::cout << "collapsing " << tet_list.size() << std::endl;

            // Sort tets by ascending quality
            tet_list.sort([](const TetrahedronRef &a, const TetrahedronRef &b) {
                return a->EdgeLengthRatio() < b->EdgeLengthRatio();
            });

            // Identify the shortest edge for collapse.
            std::vector<TetEdgeRef> collapse_edges;
            for (auto &tet: tet_list)
                collapse_edges.push_back(tet->ShortestEdge());

            // Collapse edges, eliminating all associated tetrahedra.
            std::set<TetrahedronRef> delete_tets;
            for (auto& edge: collapse_edges) {

                // Delete all tets that share this edge
                auto incident_tets = edge->GetIncidentTets();
                for (auto& incident_tet: incident_tets) {
                    delete_tets.insert(incident_tet);
                }

                // Create a midpoint node.
                auto node0 = edge->GetFirstNode();
                auto node1 = edge->GetOtherNode(node0);
                auto mid_pos = (node0->Position() + node1->Position()) / 2.0;
                node0->SetPosition(mid_pos);

                // Replace the node in all topology objects.
                for (auto& incident_tet: node1->GetIncidentTets()) {
                    if (std::find(delete_tets.begin(), delete_tets.end(), incident_tet) == delete_tets.end())
                        incident_tet->ReplaceNode(node1, node0);
                }

                for (auto& incident_face: node1->GetIncidentFaces()) {
                    incident_face->ReplaceNode(node1, node0);
                }

                for (auto& incident_edge: node1->GetIncidentEdges()) {
                    incident_edge->ReplaceNode(node1, node0);
                }

            }

            // Remove collapsed nodes.
            for (auto& tet: delete_tets)
                DeleteTetrahedron(tet);

        }

        iter--;

        DeleteUnusedTopology();
    }

    return done;

}

}; // namespace destroyer