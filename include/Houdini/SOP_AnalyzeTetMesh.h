//
// Created by Brett Miller on 8/15/18.
//

#pragma once

#include <SOP/SOP_Node.h>


namespace destroyer {


/*

 SOP_AnalyzeTetMesh

 Analyzes a Tetrahedral Mesh for quality with respect to it's use in FEM solves.

 Inputs:
    Input0:
        Tetrahedron primitives. (All other primitives are discarded.)

 Output:
        The tetrahedral mesh with primitive attributes attached:
            longest_edge
            shortest_edge
                min/max edge length of the tetrahedron
            shape_quality
                insphere to circumsphere ratio (1.0 describes equilateral tetrahedron)
            aspect_ratio
                tetrahedron’s maximum edge length divided by its minimum altitude
            min_dihedral_angle
            max_dihedral_angle
                min/max angles between tetrahedral faces.
        And with point attributes:
            boundary
                1 if the point is on the tetrahedral mesh boundary.

*/

class SOP_AnalyzeTetMesh : public SOP_Node
{
public:
    SOP_AnalyzeTetMesh(OP_Network *net, const char *name, OP_Operator *op);
    ~SOP_AnalyzeTetMesh() override = default;

    static PRM_Template myTemplateList[];
    static OP_Node *myConstructor(OP_Network*, const char *, OP_Operator *);

protected:

    const char *inputLabel(unsigned idx) const override;

    OP_ERROR cookMySop(OP_Context &context) override;

};


}; // destroyer
