//
// Created by Brett Miller on 9/22/18.
//

#pragma once

#include <SOP/SOP_Node.h>


namespace destroyer {

/*

 SOP_ClipTetMesh

 Conforms tetrahedra mesh to Level Set boundary.

 Uses the method described in "Tetrahedral mesh generation based on space indicator functions", Friess, et al.

 Inputs:
    Input0:
        Tetrahedron primitives. (All other primitives are discarded.)

    Input1:
        A VDB SDF grid describing the level set surface of the geometry to be inscribed.

 Parameters:
    Quality Threshold


 Output:
        The tetrahedral mesh with tetrahedra corrected and clipped.



*/

    class SOP_ClipTetMesh : public SOP_Node
    {
    public:
        SOP_ClipTetMesh(OP_Network *net, const char *name, OP_Operator *op);
        ~SOP_ClipTetMesh() override = default;

        static PRM_Template myTemplateList[];
        static OP_Node *myConstructor(OP_Network*, const char *, OP_Operator *);

    protected:

        const char *inputLabel(unsigned idx) const override;

        OP_ERROR cookMySop(OP_Context &context) override;


    };

}; // namespace destroyer
