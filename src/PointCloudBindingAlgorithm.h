#ifndef SOFA_COMPONENT_CONSTRAINT_BINDPOINTNALGORITHM_H
#define SOFA_COMPONENT_CONSTRAINT_BINDPOINTNALGORITHM_H

#include "CollisionAlgorithm.h"
#include "ConstraintProximity.h"
#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <math.h>
#include <sofa/defaulttype/Vec.h>

namespace sofa {

namespace core {

namespace behavior {

class PointCloudBindingAlgorithm : public CollisionAlgorithm {
public:

    SOFA_CLASS(PointCloudBindingAlgorithm , CollisionAlgorithm );

    Data<double> d_maxDist;

    PointCloudBindingAlgorithm();

    void processAlgorithm(BaseGeometry * geoFrom,BaseGeometry * geoTo,helper::vector<PariProximity> & output);

};

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
