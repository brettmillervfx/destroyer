//
// Created by Brett Miller on 8/15/18.
//

#pragma once

// This is only here because of a macro B2 that has the same name in Eigen as OpenVDB.
// If Eigen is included first, the conflict resolves.
//#include <Eigen/Dense>

#include <SOP/SOP_Node.h>

namespace destroyer {

/*

 SOP_TetrahedralizeVDB

 Implements a Houdini SOP node that converts a VDB signed distance field to
 a tetrahedral mesh, suitable for FEM solver application.

 The algorithm is largely that described in Bridson, et al.'s 2003 publication
 Adaptive Physics Based Tetrahedral Mesh Generation Using Level Sets.

 Inputs:
    Input0:
        A VDB SDF grid describing the level set surface of the geometry to be inscribed.
        All other geometry found in the input will be discarded. Only the first encountered SDF
        is considered.

 Parameters
    edgeLength:
        The desired edge length of the initial base tetrahedral mesh. Note that it is unlikely that
        this edge length will be maintained to final construction: Adaptive subdivision and mesh compression
        will almost certainly produce a broad range of lengths. Nonetheless, the parameter provides a reasonably
        good target.
        EdgeLength is expected to be greater than zero.

 Output:
        The tetrahedral mesh.

*/

class SOP_TetrahedralizeVDB : public SOP_Node
{
public:
    SOP_TetrahedralizeVDB(OP_Network *net, const char *name, OP_Operator *op);
    ~SOP_TetrahedralizeVDB() override = default;

    static PRM_Template myTemplateList[];
    static OP_Node *myConstructor(OP_Network*, const char *, OP_Operator *);

protected:

    const char *inputLabel(unsigned idx) const override;

    OP_ERROR cookMySop(OP_Context &context) override;

};


}; // namespace destroyer