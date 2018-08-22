//
// Created by Brett Miller on 8/15/18.
//

#pragma once

#include <SOP/SOP_Node.h>


namespace destroyer {


/*

 SOP_CompressTetMesh

 Compresses and optimizes a tetrahedral mesh to conform to the boundaries of the supplied level set.

 Inputs:
    Input0:
        Tetrahedron primitives. (All other primitives are discarded.)

    Input1:
        A VDB SDF grid describing the level set surface of the geometry to be inscribed.

    Parameters:


    Output:
        The tetrahedral mesh with boundary nodes conforming to the supplied level set surface. Tets are
        optimized according to the user supplied weights.
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
