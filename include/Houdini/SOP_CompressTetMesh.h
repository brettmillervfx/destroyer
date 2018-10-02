//
// Created by Brett Miller on 8/15/18.
//

#pragma once

#include <SOP/SOP_Node.h>


namespace destroyer {


/*

 SOP_CompressTetMesh

 Compresses and optimizes a tetrahedral mesh to conform to the boundaries of the supplied level set and generally
 improve the quality of the tetrahedra.
 This is implimented as a loop of "sweeps". During each sweep, boundary nodes are first moved towards the level set.
 Nodes are then moved to optimize a quality heuristic, which is defined as the ratio of the inscribed sphere of the
 tetrahedron over the circumscibed sphere, normalized to the range of 0 to 1. (1 is a regular "ideal" tetrahedron; 0 is
 a completely degraded tet with all nodes on a plane.)

 To avoid getting stuck in a local minimum, it is recommended that the user set small step sizes and increase the number
 of sweeps. This will increase solve time but usually leads to better results. Output to the shell reports the lowest
 quality found in the tet mesh at the beginning of each sweep.

 As a speed optimization, the user may elect to only adjust nodes that are adjacent to a tet with quality that falls
 below a specified threshold. This can often speed up the simulation dramatically without affecting overall results.

 Inputs:
    Input0:
        Tetrahedron primitives. (All other primitives are discarded.)

    Input1:
        A VDB SDF grid describing the level set surface of the geometry to be inscribed.

    Parameters:
        Sweeps
            Number of iterative sweeps to perform for node adjustment. To avoid local minima, higher number of sweeps
            with lower step values is recommended (although this is slower.)

        Boundary Step Size
            For each sweep, move boundary nodes towards the level set by this fraction. High values will cause rapid
            degradation of boundary tets which are difficult for the solver to repair.

        Interior Step Size
            For each sweep, move candidate interior nodes no more than this fraction of their shortest altitude (to
            prevent tet collapse). Smaller values avoid local minima.

        Quality Threshold
            As an optimization, each sweep only moves nodes incident to tets with lower quality than the threshold.

    Output:
        The tetrahedral mesh with boundary nodes conforming to the supplied level set surface. Tets are
        optimized to conform as close as possible to ideal regular tetrahedra. (Although practically speaking,
        this ideal will rarely be fully realized.)

*/

class SOP_CompressTetMesh : public SOP_Node
{
public:
    SOP_CompressTetMesh(OP_Network *net, const char *name, OP_Operator *op);
    ~SOP_CompressTetMesh() override = default;

    static PRM_Template myTemplateList[];
    static OP_Node *myConstructor(OP_Network*, const char *, OP_Operator *);

protected:

    const char *inputLabel(unsigned idx) const override;

    OP_ERROR cookMySop(OP_Context &context) override;


};

}; // namespace destroyer
