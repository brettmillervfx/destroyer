//
// Created by Brett Miller on 8/15/18.
//

#include "Houdini/SOP_TetrahedralizeVDB.h"

#include <OP/OP_AutoLockInputs.h>
#include <PRM/PRM_Range.h>

#include "TetMesh/BCCLatticeGenerator.h"
#include "TetMesh/TetMeshToHoudiniDetail.h"
#include "TetMesh/VDBSampler.h"
#include "TetMesh/TetMesh.h"
#include "TetMesh/TetNode.h"
#include "TetMesh/Tetrahedron.h"


namespace destroyer {


static PRM_Name edge_length_prm_name("edgeLength", "Base Edge Length");
static PRM_Default edge_length_prm_default(1.0);
static PRM_Range edge_length_prm_range(PRM_RANGE_PRM, 0, PRM_RANGE_FREE, 10.0);

static PRM_Name cull_depth_prm_name("cullDepth", "Cull Depth");
static PRM_Default cull_depth_prm_default(2);
static PRM_Range cull_depth_prm_range(PRM_RANGE_PRM, 0, PRM_RANGE_FREE, 4);

PRM_Template
SOP_TetrahedralizeVDB::myTemplateList[] = {
    PRM_Template(PRM_FLT_J, 1, &edge_length_prm_name, &edge_length_prm_default, 0, &edge_length_prm_range, 0, 0, 1,
            "Target edge length for base tetrahedra."),
    PRM_Template(PRM_INT_J, 1, &cull_depth_prm_name, &cull_depth_prm_default, 0, &cull_depth_prm_range, 0, 0, 1,
            "The level set may pass through a tet even if all nodes are outside. Suspect tets may be recursively subdivided for purposes of testing. Higher values will be slower but are useful for detecting bad culls in high curvature areas."),
    PRM_Template(),
};


//////////////////////////////////////
//////////////////////////////////////


OP_Node *
SOP_TetrahedralizeVDB::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_TetrahedralizeVDB(net, name, op);
}


SOP_TetrahedralizeVDB::SOP_TetrahedralizeVDB(OP_Network *net, const char *name, OP_Operator *op) : SOP_Node(net, name, op)
{
}


OP_ERROR
SOP_TetrahedralizeVDB::cookMySop(OP_Context &context)
{
    OP_AutoLockInputs inputs(this);
    if (inputs.lock(context) >= UT_ERROR_ABORT)
        return error();

    duplicateSource(0, context);

    // An SDF sampler wil be necessary for tetrahedra culling and surface compression.
    auto sdf_sampler = std::make_shared<VDBSampler>(inputGeo(0, context));

    if (!sdf_sampler->IsValid()) {
        addError(SOP_ERR_INVALID_SRC, "No SDF VDB found in first input.");
        return error();
    }

    // Collect parameters.
    auto now = context.getTime();

    auto edge_length = evalFloat("edgeLength", 0, now);
    if (edge_length<std::numeric_limits<Real>::epsilon()) {
        addError(SOP_WARN_PARMS_NOT_APPLICABLE , "Edge Length must be positive and greater than 0.");
        return error();
    }

    auto cull_depth = evalInt("cullDepth", 0, now);


    // Instantiate an empty TetMesh.
    auto tet_mesh = std::make_shared<TetMesh>(sdf_sampler);

    // Gather the initial bounds for the candidate tet mash.
    UT_BoundingBox bbox = sdf_sampler->GetBBox();
    auto xmin = bbox.xmin();
    auto ymin = bbox.ymin();
    auto zmin = bbox.zmin();
    auto xmax = bbox.xmax();
    auto ymax = bbox.ymax();
    auto zmax = bbox.zmax();

    // Fill the TetMesh with a bounded BCC Tetrahedral Lattice.
    auto bcc_gen = new BCCLatticeGenerator(tet_mesh, edge_length, xmin, ymin, zmin, xmax, ymax, zmax);
    bcc_gen->FillTetMesh();
    delete bcc_gen;

    // Delete tets outside the SDF
    tet_mesh->CullOutsideTets(cull_depth);

    // Condition TetMesh into Houdini detail geometry.
    TetMeshToHoudiniDetail conditioner(tet_mesh, gdp);
    conditioner.Convert();

    return error();
}


const char *
SOP_TetrahedralizeVDB::inputLabel(unsigned idx) const
{
    switch(idx)
    {
        case 0:
            return "VDB Level Set";
        default:
            return "default";
    }
}


}; // namespace destroyer