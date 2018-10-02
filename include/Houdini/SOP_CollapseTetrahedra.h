//
// Created by Brett Miller on 9/23/18.
//

#pragma once

#include <SOP/SOP_Node.h>


namespace destroyer {


/*


 SOP_ClipTetMesh

    Conforms tetrahedra mesh to Level Set boundary by first projecting nodes near to the level set onto the level set,
    then clipping the resulting mesh by the level set. Uses the method described in "Tetrahedral mesh generation based
    on space indicator functions", Friess, et al.


 Inputs:
    Input0:
        Tetrahedron primitives. (All other primitives are discarded.)

    Parameters:
        Quality Threshold
            A node will not be projected onto the level set if it results in a tet with quality lower than this threshold.

        Distance Threshold
            Nodes that are less than this distance from the level set are considered to be on the level set and will not
            be projected.

 Output:
    The tetrahedral mesh with tetrahedra corrected and clipped.


*/


    class SOP_CollapseTetrahedra : public SOP_Node
{
public:
    SOP_CollapseTetrahedra(OP_Network *net, const char *name, OP_Operator *op);
    ~SOP_CollapseTetrahedra() override = default;

    static PRM_Template myTemplateList[];
    static OP_Node *myConstructor(OP_Network*, const char *, OP_Operator *);

protected:

    const char *inputLabel(unsigned idx) const override;

    OP_ERROR cookMySop(OP_Context &context) override;


};

}; // namespace destroyer