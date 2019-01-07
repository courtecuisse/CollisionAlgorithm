#pragma once

#include <memory>
#include <map>
#include <vector>
#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

class ConstraintElement
{
    friend class ConstraintProximity;

public:
    typedef std::unique_ptr<ConstraintElement> UPtr;
    typedef Data<helper::vector<defaulttype::Vector3> > DataVecCoord;

    virtual ~ConstraintElement() {}

    //this function returns a vector with all the control points of the element
    //if an id is not >=0 and <getNbControlPoints() this function should return the gravity center of the element
    virtual ConstraintProximity::SPtr getControlPoint(int i = -1) const = 0;

    //this function project the point P on the element and return the corresponding proximity
    virtual ConstraintProximity::SPtr project(defaulttype::Vector3 P) const = 0;

    //return the geometry // See covariant return type !
    virtual const BaseGeometry* geometry() const = 0;

    // return the number of control points
    virtual size_t getNbControlPoints() const = 0;

    virtual void draw(const core::visual::VisualParams *vparams) const = 0;

};

}

}
