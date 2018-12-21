#pragma once

#include <sofa/collisionAlgorithm/BaseGeometryModifier.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class TElement>
class PhongNormalHandler : TNormalHandler<TElement>
{
public:
    defaulttype::Vector3 internalGetNormal(const TElement*, const double* fact) const;

};


}

}
