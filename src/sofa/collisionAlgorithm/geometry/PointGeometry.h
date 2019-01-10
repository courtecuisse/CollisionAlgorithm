#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
class PointGeometry : public TBaseGeometry<DataTypes> {
public:
    typedef TBaseGeometry<DataTypes> Inherit;
    SOFA_CLASS(SOFA_TEMPLATE(PointGeometry,DataTypes),Inherit);

    virtual ElementIterator::UPtr begin() const;

    ElementIterator::End end() const;

    virtual void draw(const core::visual::VisualParams *vparams) override;

};

}

}
