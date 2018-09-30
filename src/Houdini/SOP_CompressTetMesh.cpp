//
// Created by Brett Miller on 8/15/18.
//

#include "Houdini/SOP_CompressTetMesh.h"

#include <chrono>
#include <OP/OP_AutoLockInputs.h>
#include <UT/UT_String.h>

#include "TetMesh/DetailGenerator.h"
#include "TetMesh/TetMeshToHoudiniDetail.h"
#include "TetMesh/VDBSampler.h"
#include "TetMesh/CompressionTetMesh.h"
#include "TetMesh/TetNode.h"
#include "TetMesh/Tetrahedron.h"


namespace destroyer {

static PRM_Name sweeps_prm_name("sweeps", "Sweeps");
static PRM_Default sweeps_prm_default(5);

static PRM_Name boundary_step_prm_name("boundaryStep", "Boundary Step Size");
static PRM_Default boundary_step_prm_default(0.05);

static PRM_Name interior_step_prm_name("interiorStep", "Interior Step Size");
static PRM_Default interior_step_prm_default(0.1);

static PRM_Name quality_threshold_prm_name("qualityThreshold", "Quality Threshold");
static PRM_Default quality_threshold_prm_default(0.5);

PRM_Template
        SOP_CompressTetMesh::myTemplateList[] = {
        PRM_Template(PRM_INT_J, 1, &sweeps_prm_name, &sweeps_prm_default),
        PRM_Template(PRM_FLT_J, 1, &boundary_step_prm_name, &boundary_step_prm_default),
        PRM_Template(PRM_FLT_J, 1, &interior_step_prm_name, &interior_step_prm_default),
        PRM_Template(PRM_FLT_J, 1, &quality_threshold_prm_name, &quality_threshold_prm_default),
        PRM_Template(),
};


//////////////////////////////////////
//////////////////////////////////////


OP_Node *
SOP_CompressTetMesh::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_CompressTetMesh(net, name, op);
}


SOP_CompressTetMesh::SOP_CompressTetMesh(OP_Network *net, const char *name, OP_Operator *op) : SOP_Node(net, name, op)
{
}


OP_ERROR
SOP_CompressTetMesh::cookMySop(OP_Context &context)
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

    auto sweeps = evalInt("sweeps", 0, now);
    auto boundary_step = evalFloat("boundaryStep", 0, now);
    auto interior_step = evalFloat("interiorStep", 0, now);
    auto quality_threshold = evalFloat("qualityThreshold", 0, now);

    // Instantiate an empty TetMesh.
    auto tet_mesh = std::make_shared<CompressionTetMesh>(sdf_sampler);

    // Fill the TetMesh using detail geometry.
    auto detail_generator = new DetailGenerator(tet_mesh, gdp);
    detail_generator->FillTetMesh();
    delete detail_generator;

    tet_mesh->Compress(sweeps, boundary_step, interior_step, quality_threshold);

    // Condition TetMesh into Houdini detail geometry.
    TetMeshToHoudiniDetail conditioner(tet_mesh, gdp);
    conditioner.Convert();

    // Tear down TetMesh.
    tet_mesh->TearDown();



    return error();
}


const char *
SOP_CompressTetMesh::inputLabel(unsigned idx) const
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