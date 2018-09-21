//
// Created by Brett Miller on 8/15/18.
//

#include <iostream>

#include "TetMesh/CompressionTetMesh.h"
#include "TetMesh/Tetrahedron.h"
#include "TetMesh/TetNode.h"
#include "TetMesh/TetFace.h"
#include "TetMesh/VDBSampler.h"

namespace destroyer {


void CompressionTetMesh::Compress(int soft_sweeps, int hard_sweeps, Real quality_threshold) {

    SortNodesByDepth();

    GenerateSearchTemplates();

    for (int i=0; i<soft_sweeps; ++i) {
        std::cout << "soft " << i << " of " << soft_sweeps << std::endl;
        FindLowestQuality();
        std::cout << "lowest quality " << lowest_quality_ << std::endl;
        CompressBoundaryNodes(SOFT_BOUNDARY_COMPRESS, quality_threshold);
        OptimizeNodes(SOFT_STEP_SIZE, MAX_STRIKES, quality_threshold);
    }

    for (int i=0; i<hard_sweeps; ++i) {
        std::cout << "hard " << i << " of " << hard_sweeps << std::endl;
        FindLowestQuality();
        std::cout << "lowest quality " << lowest_quality_ << std::endl;
        CompressBoundaryNodes(HARD_BOUNDARY_COMPRESS, quality_threshold);
        OptimizeNodes(HARD_STEP_SIZE, MAX_STRIKES, quality_threshold);
    }


}


void CompressionTetMesh::SortNodesByDepth() {

    // Processing queue from handling breadth first travesal of nodes.
    std::queue<TetNodeRef> node_depth_processer;

    // Set maximum depth as initial conditions for all nodes.
    for (auto& node: nodes_) {
        node->SetDepth(std::numeric_limits<uint>::max());
    }

    // As they are added, set depth to 0 to indicate surface.
    for (auto& node: nodes_) {
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
        for (auto& tet: node->GetIncidentTets()) {

            // If a connected node has a depth higher than this node's depth + 1, set it and push it into the queue.
            for (int i=0; i<4; i++) {
                auto connected_node = tet->GetNodeRef(i);
                if (connected_node->Depth() > (node->Depth()+1)) {
                    connected_node->SetDepth(node->Depth()+1);
                    node_depth_processer.push(connected_node);
                }
            }

        }
    }

    // Sort the node list by depth.
    nodes_.sort([](const TetNodePtr & a, const TetNodePtr & b) { return a->Depth() < b->Depth(); });

}

void CompressionTetMesh::GenerateSearchTemplates() {

    // Boundary search patter: evenly distributed points on unit circle
    Real d_theta = (2.0 * M_PI) / Real(BOUNDARY_SEARCH_PATTERN_SIZE);
    for (int i=1; i<BOUNDARY_SEARCH_PATTERN_SIZE; i++) {
        auto x = sin(i*d_theta);
        auto z = cos(i*d_theta);
        Vec3 search_dir(x,0.0,z);
        boundary_search_pattern_.push_back(search_dir);
    }

    // Interior search pattern: evenly distributed points on unit spehere.
    // (Using golden spiral technique).
    for (int i=0; i<INTERIOR_SEARCH_PATTERN_SIZE; i++) {
        Real index = Real(i) + 0.5;
        auto phi = acos(1.0 - 2.0*index/Real(INTERIOR_SEARCH_PATTERN_SIZE));
        auto theta = M_PI * (1.0 + pow(5.0,0.5)) * index;
        auto x = cos(theta) * sin(phi);
        auto y = sin(theta) * sin(phi);
        auto z = cos(phi);
        Vec3 search_dir(x,y,z);
        interior_search_pattern_.push_back(search_dir);
    }

}

void CompressionTetMesh::CompressBoundaryNodes(Real compression_amount, Real quality_threshold) {

    for (auto& node: nodes_) {

        if (!node->IsBoundary())
            break;

        auto pos = node->Position();
        auto sdf = sampler_->Sample(pos[0], pos[1], pos[2]);
        auto normal = node->Normal();

        //auto min_length = node->GetMinEdgeLength();
        auto new_pos = pos - sdf * normal * compression_amount; // * std::min(sdf, min_length);

        node->SetPosition(new_pos);

    }

}

void CompressionTetMesh::OptimizeNodes(Real initial_step, int max_strikes, Real quality_threshold) {

    Sweep(initial_step, max_strikes, quality_threshold, false);
    //Sweep(initial_step, max_strikes, quality_threshold, true);

}

void CompressionTetMesh::Sweep(Real initial_step, int max_strikes, Real quality_threshold, bool reverse) {

    // Set up a random number gnerator.
    std::random_device rd;
    RandomGenerator gen(rd());

    auto node_cnt = nodes_.size();
    int updated = 0;

    if (reverse) {
        for (auto rit = nodes_.rbegin(); rit != nodes_.rend(); ++rit) {
            if (rit->get()->GetLocalQuality() > lowest_quality_*quality_threshold) {
                updated++;
                AdjustNode(rit->get(), gen, initial_step, max_strikes);
            }
        }
    } else {
        for (auto& node: nodes_) {
            if (node->GetLocalQuality() > (lowest_quality_*quality_threshold)) {
                updated++;
                AdjustNode(node.get(), gen, initial_step, max_strikes);
            }
        }
    }

    std::cout << " updated " << Real(updated)/Real(node_cnt) << std::endl;

}

void CompressionTetMesh::AdjustNode(TetNodeRef node, RandomGenerator generator, Real initial_step, int max_strikes) {



    auto current_quality = QualityMetric(node);
    auto current_pos = node->Position();
    auto best_pos = current_pos;
    auto current_step = initial_step;
    //auto min_altitude = node->GetMinAltitude();
    auto min_length = node->GetMinEdgeLength();

    for (int strike = 0; strike<max_strikes; strike++ ) {

        auto search_pattern = ConstructRandomPatternBasis(node, generator);

        bool improved = false;

        // Move the point to each position in the search pattern and determine quality.
        for (auto& search_vector: search_pattern) {
            auto test_pos = current_pos + search_vector * current_step * min_length;
            //auto test_pos = current_pos + search_vector * std::min(current_step, min_altitude);
            //auto test_pos = current_pos + search_vector * current_step;
            node->SetPosition(test_pos);

            // If the movement of this node has caused an inversion, skip it.
            if (!node->IsInverted()) {

                auto quality = QualityMetric(node);

                // If the quality is better that current, keep the new position.
                if (quality < current_quality) {
                    improved = true;
                    best_pos = test_pos;
                    current_quality = quality;
                }

            }
        }

        // If the quality is not better, cut the step in half and try again
        // (until max strikes is reached).
        if (improved)
            break;
        else;
            current_step *= 0.5;

    }

    node->SetQuality(current_quality);
    node->SetNextMove(best_pos-current_pos);
    node->SetPosition(best_pos);

}

std::vector<Vec3> CompressionTetMesh::ConstructRandomPatternBasis(TetNodeRef node, RandomGenerator generator) const {

    std::vector<Vec3> pattern;

    std::uniform_real_distribution<> dist(-1.0, 1.0);

    if (node->IsBoundary()) {

        // If the node is boundary, the search pattern is transformed into the tangent plane.
        auto y_basis = node->Normal();

        // Generate a random vector projected onto the plane.
        Vec3 x_basis( dist(generator), dist(generator), dist(generator));
        x_basis -= (x_basis.dot(y_basis) * y_basis);
        x_basis.normalize();

        Vec3 z_basis = cross(x_basis, y_basis);

        // Transform the pre-computed template
        for (auto& dir: boundary_search_pattern_) {
            pattern.push_back(dir[0]*x_basis + dir[1]*y_basis + dir[2]*z_basis);
        }

    } else {

        // Interior nodes get eight evenly distributed vectors.

        // Generate random 3d basis
        Vec3 x_basis( dist(generator), dist(generator), dist(generator));
        x_basis.normalize();
        Vec3 y_basis( dist(generator), dist(generator), dist(generator));
        y_basis -= (y_basis.dot(x_basis) * x_basis);
        y_basis.normalize();
        Vec3 z_basis = cross(x_basis, y_basis);

        // Transform the pre-computed template
        for (auto& dir: interior_search_pattern_) {
            pattern.push_back(dir[0]*x_basis + dir[1]*y_basis + dir[2]*z_basis);
        }

    }

    return pattern;

}
/*
Real CompressionTetMesh::InteriorQualityMetric(TetNodeRef node) const {

    // From the Bridson/Fedkiw 2003 paper.
    // We define the quality metric for interior nodes as the minimum value of
    // a/L + sin( theta_m)
    // calculated on each incident tetrahedron.
    // Where
    //      a = minimum altitude
    //      L = maximum edge length
    //      theta_m = maximum dihedral angle
    //
    // Note that this quality heuristic favors high values, ie. the worst return quality is 0.0.

    auto min_quality = std::numeric_limits<Real>::max();

    for (auto& tet: node->GetIncidentTets()) {

        auto a = tet->GetMinAltitude();

        MinMaxReal edge_lengths = tet->GetMinMaxEdgeLengths();
        auto L = edge_lengths[1];

        MinMaxReal dihedral_angles = tet->GetMinMaxDihedralAngles();
        auto theta_m = dihedral_angles[1];

        auto quality = (a/L) + sin(theta_m);

        auto aspect = tet->CalculateAspectRatio();
        //quality += std::max(0.0,(3.0 - aspect)) / 3.0;

        //quality /= 3.0;

        auto quality = tet->EdgeLengthRatio() + std::max(0.0,(3.0 - aspect)) / 3.0;;

        if (quality < min_quality)
            min_quality = quality;
    }

    return min_quality;

}
*/

/*
Real CompressionTetMesh::QualityMetric(TetNodeRef node) const {

    // From the Bridson/Fedkiw 2003 paper.
    // We define the quality metric for surface nodes as the interior quality metric plus
    // the minimum value of
    // a/L + 1/psi_m
    // calculated on each incident surface triangle.
    // Where
    //      a = minimum triangle altitude
    //      L = maximum triangle edge length
    //      psi_m = maximum triangle angle

    auto surface_quality = 0.0;

    if (node->IsBoundary()) {
        surface_quality = std::numeric_limits<Real>::max();

        for (auto& face: node->GetIncidentFaces()) {
            if (face->IsBoundary()) {
                auto a = face->MinNodeAltitude();
                auto L = face->MaxEdgeLength();
                auto psi_m = face->MaxAngle();

                //auto quality = (a / (L*0.8165)) + (1.0472 / psi_m);
                auto quality = (a/(L*0.8165)) + face->EdgeLengthRatio();
                //quality /= 2.0;
                //auto quality = face->EdgeLengthRatio();
                if (quality < surface_quality)
                    surface_quality = quality;
            }

        }

        return surface_quality;

    }

    return (InteriorQualityMetric(node) + surface_quality);

}
*/

Real CompressionTetMesh::QualityMetric(TetNodeRef node) const {

        auto total_quality = 0.0;

        for (auto& tet: node->GetIncidentTets()) {
            auto quality = tet->QualityMeasure();
            total_quality += 1.0 / (quality*quality*quality);
        }

        return total_quality;

}

void CompressionTetMesh::FindLowestQuality() {

    lowest_quality_ = 0.0;

    for (auto& tet: tets_) {
        auto quality = tet.first->QualityMeasure();
        if (quality>lowest_quality_)
            lowest_quality_ = quality;
    }

}

}; // namespace destroyer