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


#define BOUNDARY_SEARCH_PATTERN_SIZE 5
#define INTERIOR_SEARCH_PATTERN_SIZE 9
#define SOFT_BOUNDARY_COMPRESS 0.1
#define SOFT_STEP_SIZE 0.5
#define HARD_BOUNDARY_COMPRESS 0.5
#define HARD_STEP_SIZE 0.15
#define MAX_STRIKES 4



class CompressionTetMesh : public TetMesh
{
public:

    using TetMesh::TetMesh;
    ~CompressionTetMesh() = default;

    void Compress(int soft_sweeps, int hard_sweeps, Real quality_threshold);


private:
    void SortNodesByDepth();
    void GenerateSearchTemplates();
    void CompressBoundaryNodes(Real compression_amount, Real quality_threshold);
    void OptimizeNodes(Real initial_step, int max_strikes, Real quality_threshold);

    void Sweep(Real initial_step, int max_strikes, Real quality_threshold, bool reverse);
    void AdjustNode(TetNodeRef node, std::mt19937 generator, Real initial_step, int max_strikes);
    std::vector<Vec3> ConstructRandomPatternBasis(TetNodeRef node, std::mt19937 generator) const;

    Real QualityMetric(TetNodeRef node) const;
    Real InteriorQualityMetric(TetNodeRef node) const;

    void FindLowestQuality();


private:
    std::vector<Vec3> boundary_search_pattern_;
    std::vector<Vec3> interior_search_pattern_;

    Real lowest_quality_;

};


}; // namespace destroyer
