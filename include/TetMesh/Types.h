//
// Created by Brett Miller on 8/15/18.
//

#pragma once

/*************************************************************************************

 Forward declarations and type aliases for TetMesh classes.

*************************************************************************************/


#include <UT/UT_Vector3.h>
#include <memory>

class GEO_PrimVDB;

namespace destroyer {

    class Tetrahedron;
    class TetFace;
    class TetEdge;
    class TetNode;
    class TetMesh;
    class VDBSampler;

    using Real = double;
    using Index = int;

    using TetMeshPtr = typename std::shared_ptr<TetMesh>;
    using TetMeshRef = TetMesh*;

    using TetrahedronPtr = typename std::shared_ptr<Tetrahedron>;
    using TetrahedronRef = Tetrahedron*;

    using TetFacePtr = typename std::shared_ptr<TetFace>;
    using TetFaceRef = TetFace*;

    using TetEdgePtr = typename std::shared_ptr<TetEdge>;
    using TetEdgeRef = TetEdge*;

    using TetNodePtr = typename std::shared_ptr<TetNode>;
    using TetNodeRef = TetNode*;

    using VDBGridPtr = const GEO_PrimVDB*;
    using VDBSamplerPtr = typename std::shared_ptr<VDBSampler>;

    using Vec3 = UT_Vector3;

    using MinMaxReal = typename std::array<Real,2>;

}; // namespace destroyer
