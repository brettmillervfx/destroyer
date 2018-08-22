//
// Created by Brett Miller on 8/15/18.
//

#include "Houdini/SOP_AnalyzeTetMesh.h"

#include <memory>
#include <OP/OP_AutoLockInputs.h>

#include "TetMesh/Tetrahedron.h"
#include "TetMesh/TetMesh.h"
#include "TetMesh/TetNode.h"
#include "TetMesh/DetailGenerator.h"


namespace destroyer {


PRM_Template
        SOP_AnalyzeTetMesh::myTemplateList[] = {
        PRM_Template(),
};


//////////////////////////////////////
//////////////////////////////////////


OP_Node *
SOP_AnalyzeTetMesh::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_AnalyzeTetMesh(net, name, op);
}


SOP_AnalyzeTetMesh::SOP_AnalyzeTetMesh(OP_Network *net, const char *name, OP_Operator *op) : SOP_Node(net, name, op)
{
}


OP_ERROR
SOP_AnalyzeTetMesh::cookMySop(OP_Context &context)
{
    OP_AutoLockInputs inputs(this);
    if (inputs.lock(context) >= UT_ERROR_ABORT)
        return error();

    duplicateSource(0, context);

    // Instantiate an empty TetMesh.
    auto tet_mesh = std::make_shared<TetMesh>();

    // Fill the TetMesh using detail geometry.
    auto detail_gen = new DetailGenerator(tet_mesh, gdp);
    detail_gen->FillTetMesh();
    delete detail_gen;

    // Set up analysis primitive attributes.
    auto shortest_edge_handle = GA_RWHandleR(gdp->addFloatTuple(GA_ATTRIB_PRIMITIVE, "shortest_edge", 1));
    auto longest_edge_handle = GA_RWHandleR(gdp->addFloatTuple(GA_ATTRIB_PRIMITIVE, "longest_edge", 1));
    //auto shape_quality_handle = GA_RWHandleR(gdp->addFloatTuple(GA_ATTRIB_PRIMITIVE, "shape_quality", 1));
    //auto aspect_ratio_handle = GA_RWHandleR(gdp->addFloatTuple(GA_ATTRIB_PRIMITIVE, "aspect_ratio", 1));
    auto min_angle_handle = GA_RWHandleR(gdp->addFloatTuple(GA_ATTRIB_PRIMITIVE, "min_dihedral_angle", 1));
    auto max_angle_handle = GA_RWHandleR(gdp->addFloatTuple(GA_ATTRIB_PRIMITIVE, "max_dihedral_angle", 1));

    // Set up analysis point attributes
    auto boundary_handle = GA_RWHandleI(gdp->addIntTuple(GA_ATTRIB_POINT, "boundary", 1));

    // Calculate attributes and assign values.
    tet_mesh->ResetTetIterator();
    auto tet = tet_mesh->NextTet();
    while(tet != nullptr) {

        MinMaxReal edge_lengths = tet->GetMinMaxEdgeLengths();
        shortest_edge_handle.set(tet->Id(), edge_lengths[0]);
        longest_edge_handle.set(tet->Id(), edge_lengths[1]);

        MinMaxReal dihedral_angles = tet->GetMinMaxDihedralAngles();
        min_angle_handle.set(tet->Id(), (dihedral_angles[0] * 180.0 / 3.141592653589793));
        max_angle_handle.set(tet->Id(), (dihedral_angles[1] * 180.0 / 3.141592653589793));

        tet = tet_mesh->NextTet();
    }

    tet_mesh->FlagAllBoundaryNodes();
    tet_mesh->ResetNodeIterator();
    auto node = tet_mesh->NextNode();
    while(node != nullptr) {
        boundary_handle.set(node->Id(), (node->IsBoundary() ? 1 : 0) );
        node = tet_mesh->NextNode();
    }

    // Tear down TetMesh.
    tet_mesh->TearDown();

    return error();
}


const char *
SOP_AnalyzeTetMesh::inputLabel(unsigned idx) const
{
    switch(idx)
    {
        case 0:
            return "Tet Mesh Geo";
        default:
            return "default";
    }
}


}; // namespace destroyer