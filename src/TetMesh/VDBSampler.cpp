//
// Created by Brett Miller on 8/15/18.
//


#include "TetMesh/VDBSampler.h"

#include <limits>
#include <UT/UT_Vector3.h>
#include <GEO/GEO_PrimVDB.h>

namespace destroyer {


VDBSampler::VDBSampler(VDBGridPtr vdb) {

    vdb_ = vdb;
    background_ = EvaluateBackgroundValue();
    apply_fit_ = false;
    fit_min_ = 0.0;
    fit_max_ = 1.0;

}

VDBSampler::VDBSampler(VDBGridPtr vdb, Real min, Real max) {

    assert( fabs(max-min) > std::numeric_limits<Real>::epsilon() );

    vdb_ = vdb;
    background_ = EvaluateBackgroundValue();
    apply_fit_ = true;
    fit_min_ = min;
    fit_max_ = max;

}

VDBSampler::VDBSampler(const GU_Detail *detail) {

    vdb_ = LoadSDF(detail);
    background_ = EvaluateBackgroundValue();
    apply_fit_ = false;
    fit_min_ = 0.0;
    fit_max_ = 1.0;

}

VDBSampler::VDBSampler(const GU_Detail *detail, Real min, Real max) {

    assert( fabs(max-min) > std::numeric_limits<Real>::epsilon() );

    vdb_ = LoadSDF(detail);
    background_ = EvaluateBackgroundValue();
    apply_fit_ = true;
    fit_min_ = min;
    fit_max_ = max;

}

UT_BoundingBox VDBSampler::GetBBox() const {
    UT_BoundingBox bbox;
    vdb_->getBBox(&bbox);
    return bbox;
}

Real VDBSampler::Sample(Real x, Real y, Real z) const {

    UT_Vector3 p(x,y,z);
    auto sample = vdb_->getValueF(p);

    if (apply_fit_) {
        sample = (sample - fit_min_)  / (fit_max_ / fit_min_);
        sample = (sample>1.0) ? 1.0 : sample;
        sample = (sample<0.0) ? 0.0 : sample;
    }

    return sample;

}

Vec3 VDBSampler::SampleGradient(Real x, Real y, Real z) const {

    UT_Vector3 p(x,y,z);
    auto sample = vdb_->getGradient(p);

    return sample;

}

Real VDBSampler::EvaluateBackgroundValue() {

    // It's often useful to know the grid's background value, in case we need to
    // determine if the sample is outside the narrow band and lacks a gradient.
    GA_LocalIntrinsic intrinsic(vdb_->findIntrinsic("background"));
    exint tuplesize = vdb_->getIntrinsicTupleSize(intrinsic);
    UT_StackBuffer<fpreal64> values(tuplesize);
    vdb_->getIntrinsic(intrinsic, values, tuplesize);

    return values[0];

}

VDBGridPtr VDBSampler::LoadSDF(const GU_Detail *detail)
{

    // Find and return a pointer to the first SDF vdb grid in the detail.
    // If no valid vdb is found, nullptr is returned.
    for (GA_Iterator it(detail->getPrimitiveRange()); !it.atEnd(); it.advance())
    {
        const GEO_Primitive* prim = detail->getGEOPrimitive(it.getOffset());
        if(dynamic_cast<const GEO_PrimVDB *>(prim))
        {
            auto temp_vdb_ptr = dynamic_cast<const GEO_PrimVDB *>(prim);
            if (temp_vdb_ptr->isSDF())
                return temp_vdb_ptr;
        }
    }

    return nullptr;
}


}; // namespace destroyer