#pragma once

#include <sofa/collisionAlgorithm/BaseGeometryModifier.h>

namespace sofa
{

namespace collisionAlgorithm
{


template <typename TGeometry>
class SofaPhongNormalHandler : public SofaTNormalHandler<TGeometry>
{
public:
    SOFA_CLASS(SofaPhongNormalHandler, SofaBaseNormalHandler);

    SofaPhongNormalHandler(const BaseGeometry* geometry = NULL)
        : SofaTNormalHandler<TGeometry>(geometry)
    {

    }

    virtual ~SofaPhongNormalHandler() override {}

    defaulttype::Vector3 getNormal(const size_t elementID, const double* fact) const override
    {
        SOFA_UNUSED(elementID);
        SOFA_UNUSED(fact);

        msg_error(this) << "Geometry does not implement Phong Normal";
        return defaulttype::Vector3();
    }

protected:

};

}

}

