//
// Created by Brett Miller on 8/15/18.
//

#pragma once

#include <SOP/SOP_Node.h>

namespace destroyer {


/*

 SOP_RefineTetrahedra

 Performs 1:8 refinement on all tetrahedra in the supplied primitive group. This
 is executed by bisecting each of the tetrahedron's edges and subdividing accordingly.

 Following this operation, it is important that the user pass the TetMesh to a
 SOP_CleanTetMesh, otherwise topology will possibly be unusable for FEM simulation.

 Inputs:
    Input0:
        Tetrahedron primitives. (All other primitives are discarded.)
        A prim group is also expected.

    Input1:
        A VDB SDF grid describing the level set surface of the geometry to be inscribed.

    Parameters:
        Subdiv Group
            A prim group containing all tetrahedrons to be subdivided.

    Output:
        The tetrahedrons with specified tetrahedrons subdivided. Note that the
        output disobeys the "2:1 rule", ie. untouched tetrahedra adjacent to subdivided
        tetrahedra will not share topology with their neighbors. SOP_CleanTetMesh will
        perform the "green" subdivisions to complete the operation.
*/

class SOP_RefineTetrahedra : public SOP_Node
{
public:
    SOP_RefineTetrahedra(OP_Network *net, const char *name, OP_Operator *op);
    ~SOP_RefineTetrahedra() override = default;

    static PRM_Template myTemplateList[];
    static OP_Node *myConstructor(OP_Network*, const char *, OP_Operator *);

protected:

    const char *inputLabel(unsigned idx) const override;

    OP_ERROR cookMySop(OP_Context &context) override;


};

}; // namespace destroyer
