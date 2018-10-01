//
// Created by Brett Miller on 8/15/18.
//

#include "Houdini/SOP_RefineTetrahedra.h"

#include <OP/OP_AutoLockInputs.h>
#include <UT/UT_String.h>
#include <PRM/PRM_Range.h>

#include "TetMesh/DetailGenerator.h"
#include "TetMesh/TetMeshToHoudiniDetail.h"
#include "TetMesh/VDBSampler.h"
#include "TetMesh/RefinementTetMesh.h"
#include "TetMesh/TetNode.h"
#include "TetMesh/Tetrahedron.h"


namespace destroyer {



static PRM_Name subdiv_group_prm_name("subdivGroup", "Subdiv Group");

static PRM_Name cull_depth_prm_name("cullDepth", "Cull Depth");
static PRM_Default cull_depth_prm_default(2);
static PRM_Range cull_depth_prm_range(PRM_RANGE_PRM, 0, PRM_RANGE_FREE, 4);

PRM_Template
SOP_RefineTetrahedra::myTemplateList[] = {
    PRM_Template(PRM_STRING, 1, &subdiv_group_prm_name, 0, &SOP_Node::primGroupMenu, 0, 0, 0, 1,
            "Primitive group containing tets to be subdivided."),
    PRM_Template(PRM_INT_J, 1, &cull_depth_prm_name, &cull_depth_prm_default, 0, &cull_depth_prm_range, 0, 0, 1,
            "The level set may pass through a tet even if all nodes are outside. Suspect tets may be recursively subdivided for purposes of testing. Higher values will be slower but are useful for detecting bad culls in high curvature areas."),
    PRM_Template(),
};


//////////////////////////////////////
//////////////////////////////////////


OP_Node *
SOP_RefineTetrahedra::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_RefineTetrahedra(net, name, op);
}


SOP_RefineTetrahedra::SOP_RefineTetrahedra(OP_Network *net, const char *name, OP_Operator *op) : SOP_Node(net, name, op)
{
}


OP_ERROR
SOP_RefineTetrahedra::cookMySop(OP_Context &context)
{
    OP_AutoLockInputs inputs(this);
    if (inputs.lock(context) >= UT_ERROR_ABORT)
        return error();

    duplicateSource(0, context);

    // An SDF sampler wil be necessary for tetrahedra culling and surface compression.
    auto sdf_sampler = std::make_shared<VDBSampler>(inputGeo(1, context));

    if (!sdf_sampler->IsValid()) {
        addError(SOP_ERR_INVALID_SRC, "No SDF VDB found in second input.");
        return error();
    }

    // Collect parameters.
    auto now = context.getTime();

    UT_String subdiv_group_name;
    evalString(subdiv_group_name, "subdivGroup", 0, now);
    GA_PrimitiveGroup *subdiv_group = gdp->findPrimitiveGroup(subdiv_group_name);

    auto cull_depth = evalInt("cullDepth", 0, now);


    // Instantiate an empty TetMesh.
    auto tet_mesh = std::make_shared<RefinementTetMesh>(sdf_sampler);

    // Fill the TetMesh using detail geometry.
    auto detail_generator = new DetailGenerator(tet_mesh, gdp);
    detail_generator->FillTetMesh();
    delete detail_generator;


    // Record all of the tet offsets in the group.
    std::set<Index> group_offsets;
    for (GA_Iterator it(gdp->getPrimitiveRange(subdiv_group)); !it.atEnd(); ++it)
        group_offsets.insert(*it);

    // Refine the group tets.
    tet_mesh->RefineIdGroup(group_offsets);

    // Delete tets outside the SDF
    tet_mesh->CullOutsideTets(cull_depth);

    // Condition TetMesh into Houdini detail geometry.
    TetMeshToHoudiniDetail conditioner(tet_mesh, gdp);
    conditioner.Convert();

    return error();
}


const char *
SOP_RefineTetrahedra::inputLabel(unsigned idx) const
{
    switch(idx)
    {
        case 0:
            return "Tet Mesh Geo";
        case 1:
            return "VDB Level Set";
        default:
            return "default";
    }
}


}; // namespace destroyer