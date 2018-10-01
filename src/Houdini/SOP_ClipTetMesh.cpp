//
// Created by Brett Miller on 9/22/18.
//

#include "Houdini/SOP_ClipTetMesh.h"

#include <OP/OP_AutoLockInputs.h>
#include <PRM/PRM_Range.h>

#include "TetMesh/DetailGenerator.h"
#include "TetMesh/TetMeshToHoudiniDetail.h"
#include "TetMesh/VDBSampler.h"
#include "TetMesh/ClipTetMesh.h"
#include "TetMesh/TetNode.h"
#include "TetMesh/Tetrahedron.h"


namespace destroyer {

static PRM_Name quality_threshold_prm_name("qualityThreshold", "Quality Threshold");
static PRM_Default quality_threshold_prm_default(0.1);
static PRM_Range quality_threshold_prm_range(PRM_RANGE_PRM, 0, PRM_RANGE_PRM, 1);

static PRM_Name distance_threshold_prm_name("distanceThreshold", "Distance Threshold");
static PRM_Default distance_threshold_prm_default(0.001);
static PRM_Range distance_threshold_prm_range(PRM_RANGE_PRM, 0, PRM_RANGE_FREE, 1);

PRM_Template
SOP_ClipTetMesh::myTemplateList[] = {
    PRM_Template(PRM_FLT_J, 1, &quality_threshold_prm_name, &quality_threshold_prm_default, 0, &quality_threshold_prm_range, 0, 0, 1,
            "A node will not be projected onto the level set if it results in a tet with quality lower than this threshold."),
    PRM_Template(PRM_FLT_J, 1, &distance_threshold_prm_name, &distance_threshold_prm_default, 0, &distance_threshold_prm_range, 0, 0, 1,
            "Nodes that are less than this distance from the level set are considered to be on the level set."),
    PRM_Template(),
};


//////////////////////////////////////
//////////////////////////////////////


OP_Node *
SOP_ClipTetMesh::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_ClipTetMesh(net, name, op);
}


SOP_ClipTetMesh::SOP_ClipTetMesh(OP_Network *net, const char *name, OP_Operator *op) : SOP_Node(net, name, op)
{
}


OP_ERROR
SOP_ClipTetMesh::cookMySop(OP_Context &context)
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

    auto quality_threshold = evalFloat("qualityThreshold", 0, now);
    auto distance_threshold = evalFloat("distanceThreshold", 0, now);

    // Instantiate an empty TetMesh.
    auto tet_mesh = std::make_shared<ClipTetMesh>(sdf_sampler);

    // Fill the TetMesh using detail geometry.
    auto detail_generator = new DetailGenerator(tet_mesh, gdp);
    detail_generator->FillTetMesh();
    delete detail_generator;

    tet_mesh->Clip(quality_threshold, distance_threshold);
    tet_mesh->DeleteUnusedTopology();

    // Condition TetMesh into Houdini detail geometry.
    TetMeshToHoudiniDetail conditioner(tet_mesh, gdp);
    conditioner.Convert();

    // Tear down TetMesh.
    tet_mesh->TearDown();



    return error();
}


const char *
SOP_ClipTetMesh::inputLabel(unsigned idx) const
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