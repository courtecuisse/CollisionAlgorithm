#ifndef SOFA_COMPONENT_CONSTRAINT_FINDCLOSESTALGO_H
#define SOFA_COMPONENT_CONSTRAINT_FINDCLOSESTALGO_H

#include "CollisionAlgorithm.h"
#include "ConstraintProximity.h"
#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <math.h>
#include <sofa/defaulttype/Vec.h>
#include "AABBDecorator.h"

namespace sofa {

namespace core {

namespace behavior {


void CollisionAlgorithm::pointCloudBinding(const helper::vector<defaulttype::Vector3> & p1, const helper::vector<defaulttype::Vector3> & p2 , helper::vector<int> bindId, double minDist) {
    if (p1.size()>p2.size())  {
        printf("ERROR : CollisionAlgorithm::pointCloudBinding :: CANNOT BIND p1.size() > p2.size() \n");
        return;
    }

    for (unsigned p=0;p<p1.size();p++) {
        defaulttype::Vector3 P = p1[p];
        int closestId = -1;
        double closestDist = 0;

        //Find minimal distance
        for (unsigned i=0;i<p2.size();i++) {
            defaulttype::Vector3 Q = p2[i];
            double dist = (Q-P).norm();

            if ((closestId == -1) || (dist < closestDist)) {
                closestId = i;
                closestDist = dist;
            }
        }

        if ((minDist>0) && (closestDist > minDist)) closestId = -1; // not acceptable min dist

        bindId.push_back(closestId);
    }
}

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
