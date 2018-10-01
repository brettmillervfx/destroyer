//
// Created by Brett Miller on 9/23/18.
//

#include "Houdini/SOP_CollapseTetrahedra.h"

#include <OP/OP_AutoLockInputs.h>
#include <UT/UT_String.h>
#include <PRM/PRM_Range.h>

#include "TetMesh/DetailGenerator.h"
#include "TetMesh/TetMeshToHoudiniDetail.h"
#include "TetMesh/CollapseTetMesh.h"
#include "TetMesh/TetNode.h"
#include "TetMesh/Tetrahedron.h"


namespace destroyer {


static PRM_Name max_iter_prm_name("maxIter", "Max Iterations");
static PRM_Default max_iter_prm_default(3);
static PRM_Range max_iter_prm_range(PRM_RANGE_PRM, 0, PRM_RANGE_FREE, 6);

static PRM_Name quality_threshold_prm_name("qualityThreshold", "Quality Threshold");
static PRM_Default quality_threshold_prm_default(0.05);
static PRM_Range quality_threshold_prm_range(PRM_RANGE_PRM, 0, PRM_RANGE_PRM, 1);

PRM_Template
SOP_CollapseTetrahedra::myTemplateList[] = {
    PRM_Template(PRM_INT_J, 1, &max_iter_prm_name, &max_iter_prm_default, 0, &max_iter_prm_range, 0, 0, 1,
            "Collapsing tets may produce new bad tets. The process will repeat until all bad tets are removed or until max interations is reached."),
    PRM_Template(PRM_FLT_J, 1, &quality_threshold_prm_name, &quality_threshold_prm_default, 0, &quality_threshold_prm_range, 0, 0, 1,
            "All tets with quality lower than the threshold will be collapsed along the shortest edge."),
    PRM_Template(),
};


//////////////////////////////////////
//////////////////////////////////////


OP_Node *
SOP_CollapseTetrahedra::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_CollapseTetrahedra(net, name, op);
}


SOP_CollapseTetrahedra::SOP_CollapseTetrahedra(OP_Network *net, const char *name, OP_Operator *op) : SOP_Node(net, name, op)
{
}


OP_ERROR
SOP_CollapseTetrahedra::cookMySop(OP_Context &context)
{
    OP_AutoLockInputs inputs(this);
    if (inputs.lock(context) >= UT_ERROR_ABORT)
        return error();

    duplicateSource(0, context);

    // Collect parameters.
    auto now = context.getTime();

    auto max_iter = evalInt("maxIter", 0, now);
    auto quality_threshold = evalFloat("qualityThreshold", 0, now);

    // Instantiate an empty TetMesh.
    auto tet_mesh = std::make_shared<CollapseTetMesh>();

    // Fill the TetMesh using detail geometry.
    auto detail_generator = new DetailGenerator(tet_mesh, gdp);
    detail_generator->FillTetMesh();
    delete detail_generator;

    if (!tet_mesh->Cleanup(quality_threshold, max_iter))
        addWarning(SOP_MESSAGE, "Cleanup incomplete. Increase max iterations.");

    // Condition TetMesh into Houdini detail geometry.
    TetMeshToHoudiniDetail conditioner(tet_mesh, gdp);
    conditioner.Convert();

    // Tear down TetMesh.
    tet_mesh->TearDown();


    return error();
}


const char *
SOP_CollapseTetrahedra::inputLabel(unsigned idx) const
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
