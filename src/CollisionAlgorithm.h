#ifndef SOFA_COMPONENT_CONSTRAINT_COLLISIONALGORITHM_H
#define SOFA_COMPONENT_CONSTRAINT_COLLISIONALGORITHM_H

#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/core/behavior/MechanicalState.h>
#include "ConstraintGeometry.h"
#include "ConstraintProximity.h"
#include "TriangleGeometry.h"
namespace sofa {

namespace core {

namespace behavior {

class BaseDecorator  {
public:
    virtual BaseConstraintIteratorPtr getIterator(const ConstraintProximityPtr & P) = 0;

};

class CollisionFilter : public core::objectmodel::BaseObject {
public:
    virtual bool filter(const ConstraintProximityPtr & from,const ConstraintProximityPtr & dst) = 0;

};

class CollisionAlgorithm {
public:

    static ConstraintProximityPtr findClosestProximity(const ConstraintProximityPtr & T, BaseGeometry * geo);

    static void pointCloudBinding(const helper::vector<defaulttype::Vector3> & p1, const helper::vector<defaulttype::Vector3> & p2 , helper::vector<int> & bindId, double minDist = 0.0);

    static ConstraintProximityPtr newtonTriangularIterations(const defaulttype::Vector3 & P,unsigned eid,const ConstraintProximityPtr & pinfo,unsigned max_it, double tolerance, double threshold);

};

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
