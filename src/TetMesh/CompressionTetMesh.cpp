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


void CompressionTetMesh::Compress(int sweeps, Real boundary_step, Real interior_step, Real quality_threshold) {

    GenerateSearchTemplates();

    for (int i=0; i<sweeps; ++i) {
        std::cout << "soft " << i << " of " << sweeps << std::endl;
        FindLowestQuality();
        std::cout << "lowest quality " << lowest_quality_ << std::endl;
        CompressBoundaryNodes(boundary_step);
        OptimizeNodes(interior_step, quality_threshold);
    }

}

void CompressionTetMesh::SortNodesByQuality() {

    for (auto& node: nodes_) {
        node->CacheLocalQuality();
    }

    // Sort the node list by quality.
    nodes_.sort([](const TetNodePtr & a, const TetNodePtr & b) { return a->LocalQuality() < b->LocalQuality(); });

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

void CompressionTetMesh::CompressBoundaryNodes(Real compression_amount) {

    for (auto& node: nodes_) {

        if (node->IsBoundary()) {

            auto pos = node->Position();
            auto sdf = sampler_->Sample(pos[0], pos[1], pos[2]);
            auto normal = node->Normal();

            auto new_pos = pos - sdf * normal * compression_amount;

            node->SetPosition(new_pos);

        }

    }

}

void CompressionTetMesh::OptimizeNodes(Real initial_step, Real quality_threshold) {

    SortNodesByQuality();
    Sweep(initial_step, quality_threshold);

}

void CompressionTetMesh::Sweep(Real initial_step, Real quality_threshold) {

    // Set up a random number gnerator.
    std::random_device rd;
    RandomGenerator gen(rd());

    for (auto& node: nodes_) {
        if ( (node->LocalQuality()-lowest_quality_)/(1.0-lowest_quality_) < quality_threshold)
            AdjustNode(node.get(), gen, initial_step);
    }

}

void CompressionTetMesh::AdjustNode(TetNodeRef node, RandomGenerator generator, Real initial_step) {

    auto current_quality = QualityMetric(node);
    auto current_pos = node->Position();
    auto best_pos = current_pos;
    auto current_step = initial_step;
    auto min_length = node->GetMinEdgeLength();

    for (int strike = 0; strike<MAX_STRIKES; strike++ ) {

        auto search_pattern = ConstructRandomPatternBasis(node, generator);

        bool improved = false;

        // Move the point to each position in the search pattern and determine quality.
        for (auto& search_vector: search_pattern) {

            auto test_pos = current_pos + (search_vector * current_step * min_length);
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

Real CompressionTetMesh::QualityMetric(TetNodeRef node) const {

    auto total_quality = 0.0;

    for (auto& tet: node->GetIncidentTets()) {
        auto quality = tet->QualityMeasure();
        total_quality += 1.0 / (quality*quality*quality);
    }

    return total_quality;

}

void CompressionTetMesh::FindLowestQuality() {

    lowest_quality_ = std::numeric_limits<Real>::max();

    for (auto& tet: tets_) {
        auto quality = tet.first->QualityMeasure();
        if (quality<lowest_quality_)
            lowest_quality_ = quality;
    }

}

}; // namespace destroyer