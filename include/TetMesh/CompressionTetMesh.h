//
// Created by Brett Miller on 8/15/18.
//

#pragma once

#include "TetMesh.h"

#include <cmath>
#include <queue>
#include <random>
#include <vector>


namespace destroyer {

/*

 CompressionTetMesh class

 Iteratively moves the nodes of the contained TetMesh so that the quality of the internal tetrahedra are
 optimized for FEM solves and the boundary nodes conform to the surface described by the supplied VDB level set.

 // TODO complete the documentation once we have a working solution

*/

// Marseinne Twister used for random number generation.
using RandomGenerator = typename std::mt19937;

#define BOUNDARY_SEARCH_PATTERN_SIZE 5
#define INTERIOR_SEARCH_PATTERN_SIZE 7
#define MAX_STRIKES 4


class CompressionTetMesh : public TetMesh
{
public:

    using TetMesh::TetMesh;

    // Uses the base class constructor.
    ~CompressionTetMesh() = default;

    // Compress the nodes of the TetMesh.
    void Compress(int sweeps, Real boundary_step, Real interior_step, Real quality_threshold);

private:
    void SortNodesByQuality();
    void GenerateSearchTemplates();
    void CompressBoundaryNodes(Real compression_amount);
    void OptimizeNodes(Real initial_step, Real quality_threshold);
    void Sweep(Real initial_step, Real quality_threshold);
    void AdjustNode(TetNodeRef node, RandomGenerator generator, Real initial_step);
    std::vector<Vec3> ConstructRandomPatternBasis(TetNodeRef node, RandomGenerator generator) const;
    Real QualityMetric(TetNodeRef node) const;
    void FindLowestQuality();

private:
    std::vector<Vec3> boundary_search_pattern_;
    std::vector<Vec3> interior_search_pattern_;
    Real lowest_quality_;

};


}; // namespace destroyer
