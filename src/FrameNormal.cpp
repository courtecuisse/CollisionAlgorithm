#ifndef SOFA_COMPONENT_CONSTRAINT_FINDCLOSESTALGO_H
#define SOFA_COMPONENT_CONSTRAINT_FINDCLOSESTALGO_H

#include "CollisionAlgorithm.h"
#include "ConstraintProximity.h"

namespace sofa {

namespace core {

namespace behavior {


ConstraintNormal CollisionAlgorithm::createConstraint(defaulttype::Vector3 N1) {
    ConstraintNormal res;

    if (N1.norm() == 0) return res;

    N1.normalize();

    res.m_normals.push_back(N1);

    return res;
}

ConstraintNormal CollisionAlgorithm::createFrameConstraint(defaulttype::Vector3 N1) {
    ConstraintNormal res;

    if (N1.norm() == 0) return res;

    N1.normalize();

    defaulttype::Vector3 N2 = cross(N1,((fabs(dot(N1,defaulttype::Vector3(1,0,0)))>0.99) ? defaulttype::Vector3(0,1,0) : defaulttype::Vector3(1,0,0)));
    N2.normalize();

    defaulttype::Vector3 N3 = cross(N1,N2);
    N3.normalize();

    res.m_normals.push_back(N1);
    res.m_normals.push_back(N2);
    res.m_normals.push_back(N3);

    return res;
}

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
