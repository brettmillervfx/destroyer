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

/*************************************************************************************

 CompressionTetMesh class

 CompressionTetMesh is a specialization of TetMesh used to simultaneously conform the boundary of a
 TetMesh to a level set surface and optimize the quality of the tetrahedra to ensure best results from
 FEM solves.

 The algorithm employed works best if the TetMesh has been culled to the level set and preferably clipped
 to the level set prior to compression.

 The algorithm employed is iterative. It is highly unlikely to converge, so the user is encouraged to set
 aa high a number of iterations as tolerable. The quality of the worst tet is reported to the shell at each
 iteration. It's likely that the algorithm will eventually find a local minimum and begin oscillating, so there
 is usually a "sweet spot" for iteration count.

 In each iteration, boundary nodes are moved a fraction closer to the level set. Then each node is moved to the
 position that maximized local quality: defined as the lowest quality of all tetrahedra incident to the node. The
 optimization is found using a pattern search algorithm:
 https://en.wikipedia.org/wiki/Pattern_search_(optimization)
 During pattern search, boundary nodes search locations are constrained to the mesh boundary tangent plane.

 To avoid optimizing the nodes to local quality maximuma, it is recommended that the user set small boundary and interior
 steps and allow more interations. This is slower but tends to produce better results.

*************************************************************************************/

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
    // sweeps is the number of optimization iterations to allow
    // boundary_step is the fraction of distance to close when moving boundary nodes each iteration,
    //      ie. 0.05 will move a boundary node 5% closer to the level set each iteration.
    // interior_step is the maximum distance to move an interior node at each iteration, relative to
    //      the shortest edge, ie. 0.1 will move a node no further that 10% the length of the shortest edge
    //      incident to the node.
    // quality_threshold is a speed optimization: at each iteration, only nodes incident to a tet of quality in
    //      percentile prescribed are considered for a move. For instance, if the lowest quality in the mesh is
    //      currently 0.2 (highest quality is always considered 1.0), and the quality_threshold is 0.5, only nodes
    //      with "local quality" lower that 0.6 will be moved. (0.6 is halfway between lowest and highest observed quality).
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
