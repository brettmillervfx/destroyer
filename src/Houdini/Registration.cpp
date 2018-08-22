//
// Created by Brett Miller on 8/15/18.
//

#include <UT/UT_DSOVersion.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>

#include "Houdini/SOP_TetrahedralizeVDB.h"
#include "Houdini/SOP_RefineTetrahedra.h"
//#include "Houdini/SOP_AnalyzeTetMesh.h"
//#include "Houdini/SOP_CompressTetMesh.h"
//#include "Houdini/SOP_CleanupTetMesh.h"



void
newSopOperator(OP_OperatorTable *table)
{

    OP_Operator *opTetrahedralizeVDB;
    opTetrahedralizeVDB = new OP_Operator(
            "TetrahedralizeVDB",
            "Tetrahedralize VDB",
            destroyer::SOP_TetrahedralizeVDB::myConstructor,
            destroyer::SOP_TetrahedralizeVDB::myTemplateList,
            1,
            1,
            nullptr,
            OP_FLAG_GENERATOR);

    opTetrahedralizeVDB->setOpTabSubMenuPath("Weta");
    table->addOperator(opTetrahedralizeVDB);


    OP_Operator *opRefineTetrahedra;
    opRefineTetrahedra = new OP_Operator(
            "RefineTetrahedra",
            "Refine Tetrahedra",
            destroyer::SOP_RefineTetrahedra::myConstructor,
            destroyer::SOP_RefineTetrahedra::myTemplateList,
            2,
            2,
            nullptr);

    opRefineTetrahedra->setOpTabSubMenuPath("Weta");
    table->addOperator(opRefineTetrahedra);

    /*
    OP_Operator *opAnalyzeTetMesh;
    opAnalyzeTetMesh = new OP_Operator(
            "AnalyzeTetMesh",
            "Analyze Tet Mesh",
            destroyer::SOP_AnalyzeTetMesh::myConstructor,
            destroyer::SOP_AnalyzeTetMesh::myTemplateList,
            1,
            1,
            nullptr);

    opAnalyzeTetMesh->setOpTabSubMenuPath("Weta");
    table->addOperator(opAnalyzeTetMesh);

    OP_Operator *opCompressTetMesh;
    opCompressTetMesh = new OP_Operator(
            "CompressTetMesh",
            "Compress Tet Mesh",
            destroyer::SOP_CompressTetMesh::myConstructor,
            destroyer::SOP_CompressTetMesh::myTemplateList,
            2,
            2,
            nullptr);

    opCompressTetMesh->setOpTabSubMenuPath("Weta");
    table->addOperator(opCompressTetMesh);

    OP_Operator *opCleanupTetMesh;
    opCleanupTetMesh = new OP_Operator(
            "CleanupTetMesh",
            "Cleanup Tet Mesh",
            destroyer::SOP_CleanupTetMesh::myConstructor,
            destroyer::SOP_CleanupTetMesh::myTemplateList,
            2,
            2,
            nullptr);

    opCleanupTetMesh->setOpTabSubMenuPath("Weta");
    table->addOperator(opCleanupTetMesh);
    */
}