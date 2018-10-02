//
// Created by Brett Miller on 9/30/18.
//

#pragma once

#include <SOP/SOP_Node.h>


namespace destroyer {

/*

 SOP_TetMeshToCutter

 The boundary of a TetMesh is converted to a polymesh, suitable for boolean operations.

 Inputs:
    Input0:
        Tetrahedron primitives. (All other primitives are discarded.)

    Parameters:

    Output:
        A triangulated polymesh comprising the closed boundary surfaces of the Input0 TetMesh.

*/

class SOP_TetMeshToCutter : public SOP_Node
{
public:
    SOP_TetMeshToCutter(OP_Network *net, const char *name, OP_Operator *op);
    ~SOP_TetMeshToCutter() override = default;

    static PRM_Template myTemplateList[];
    static OP_Node *myConstructor(OP_Network*, const char *, OP_Operator *);

protected:

    const char *inputLabel(unsigned idx) const override;

    OP_ERROR cookMySop(OP_Context &context) override;


};

}; // namespace destroyer
