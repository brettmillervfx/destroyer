//
// Created by Brett Miller on 8/15/18.
//

#include "TetMesh/CompressionTetMesh.h"
#include "TetMesh/Tetrahedron.h"
#include "TetMesh/TetNode.h"

namespace destroyer {


    void CompressionTetMesh::SortNodesByDepth() {

        // Processing queue fro handling breadth first travesal of nodes.
        std::queue<TetNodeRef> node_depth_processer;

        // Set maximum depth as initial conditions for all nodes.
        for (auto& node: node_list_) {
            node->SetDepth(std::numeric_limits<int>::max());
        }

        // Flag all surface nodes and add them to the processing queue.
        FlagAllBoundaryNodes();

        // As they are added, set depth to 0 to indicate surface.
        for (auto& node: node_list_) {
            if(node->IsBoundary()) {
                node->SetDepth(0);
                node_depth_processer.push(node.get());
            }
        }

        // Perform a breadth first scan of all the nodes, beginning with surface.
        // When the queue is empty, the nodes all have optimal depth assigned.
        while (!node_depth_processer.empty()) {

            auto node = node_depth_processer.front();
            node_depth_processer.pop();

            // Gather all connected nodes, ie. all nodes on all connected tets.
            node->ResetConnectedTetsIterator();
            auto tet = node->NextConnectedTet();
            while(tet != nullptr) {
                // If a connected node has a depth higher than this node's depth + 1, set it and push it into the queue.
                for (Index i=0; i<4; i++) {
                    auto connected_node = tet->GetNodeRef(i);
                    if (connected_node->Depth() > (node->Depth()+1)) {
                        connected_node->SetDepth(node->Depth()+1);
                        node_depth_processer.push(connected_node);
                    }
                }
                tet = node->NextConnectedTet();
            }
        }

        // Sort the node list by depth.
        node_list_.sort([](const TetNodePtr & a, const TetNodePtr & b) { return a->Depth() < b->Depth(); });

    }

    Real CompressionTetMesh::InteriorQualityMetric(TetNodeRef node) const {

        // From the Bridson/Fedkiw 2003 paper.
        // We define the quality metric for interior nodes as the minimum value of
        // a/L + 1/4 * cos( theta_m)
        // calculated on each incident tetrahedron.
        // Where
        //      a = minimum altitude
        //      L = maximum edge length
        //      theta_m = maximum dihedral angle

        auto min_quality = std::numeric_limits<Real>::max();

        node->ResetConnectedTetsIterator();
        auto tet = node->NextConnectedTet();
        while (tet != nullptr) {
            auto a = tet->GetMinAltitude();

            MinMaxReal edge_lengths = tet->GetMinMaxEdgeLengths();
            auto L = edge_lengths[1];

            MinMaxReal dihedral_angles = tet->GetMinMaxDihedralAngles();
            auto theta_m = dihedral_angles[1];

            auto quality = (a/L) + (cos(theta_m) / 4.0);
            if (quality < min_quality)
                min_quality = quality;
            tet = node->NextConnectedTet();
        }

        return min_quality;

    }

    Real CompressionTetMesh::SurfaceQualityMetric(TetNodeRef node) const {

        // From the Bridson/Fedkiw 2003 paper.
        // We define the quality metric for surface nodes as the interior quality metric plus
        // the minimum value of
        // a/L + 1/psi_m
        // calculated on each incident surface triangle.
        // Where
        //      a = minimum triangle altitude
        //      L = maximum triangle edge length
        //      psi_m = maximum triangle angle

        auto min_quality = std::numeric_limits<Real>::max();

        node->ResetConnectedTetsIterator();
        auto tet = node->NextConnectedTet();
        while (tet != nullptr) {

            // Are there other boundary nodes?

            // Get the two other boundary nodes.
            // TODO this isn't done obviously
            auto a = 1.0;
            auto L = 1.0;
            auto psi_m = 1.0;

            auto quality = (a/L) + (1.0/psi_m);
            if (quality < min_quality)
                min_quality = quality;
            tet = node->NextConnectedTet();
        }

        return InteriorQualityMetric(node) + min_quality;

    }

}; // namespace destroyer