//
// Created by Brett Miller on 8/15/18.
//

#pragma once

#include <SOP/SOP_Node.h>


namespace destroyer {

/*

 SOP_CleanupTetMesh

 Refines or removes non-manifold tetrahedra (ie. when two or more tetrahedra share a node but do
 not share an edge, or when an edge is shares by tetrahedra that do not share a face).

 Also ensures that all tetrahedra in the mesh obey the following rules:

    * No tetrahedron may have all four nodes on the boundary.
    * No interior edge may connect two boundary nodes.

 Inputs:
    Input0:
        Tetrahedron primitives. (All other primitives are discarded.)

 Parameters:
    Max Iterations
        Removal of problematic tetrahedra is an iterative process. Max Ierations provides a failsafe to exit
        iteration if a long loop is entered. A warning is issued if the SOP failed to remove all problem tets
        when it exited. The user is recommended to increment max iterations until the warning is no longer issued.

    Refine Lone Tets
        All disconnected tets are "regular" subdivided, ie. 8:1.

    Remove Weak Exterior Tets
        Tets with all four nodes on the boundary are deleted.

    Split Weak Interior Edges
        Any non-boundary edge with both nodes on the boundary is split and the adjacent tets subdivided.

    Refine Non-Manifold Edges
        A non-manifold edge is defined as an edge with more than 3 incident boundary faces. incident tets are deleted
        until the edge becomes manifold.

    Refine Non-Manifold Nodes
        Non-manifold nodes are those with more than one incident edge ring, ie. the node is shared by two or more
        manifold surfaces. Tets are removed until the node becomes manifold.

 Output:
        The tetrahedral mesh with tetrahedra corrected.


*/

class SOP_CleanupTetMesh : public SOP_Node
{
public:
    SOP_CleanupTetMesh(OP_Network *net, const char *name, OP_Operator *op);
    ~SOP_CleanupTetMesh() override = default;

    static PRM_Template myTemplateList[];
    static OP_Node *myConstructor(OP_Network*, const char *, OP_Operator *);

protected:

    const char *inputLabel(unsigned idx) const override;

    OP_ERROR cookMySop(OP_Context &context) override;


};

}; // namespace destroyer
