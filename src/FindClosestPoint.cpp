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
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/visual/VisualParams.h>
#include <SofaOpenglVisual/OglModel.h>
#include <math.h>
#include <assert.h>     /* assert */

namespace sofa {

namespace core {

namespace behavior {

#define min3(a,b,c) std::min(std::min(a,b),c)
#define max3(a,b,c) std::max(std::max(a,b),c)

ConstraintProximity CollisionAlgorithm::findClosestProximity(const ConstraintProximity &T, BaseGeometry * geo, CollisionFilter * filter) {
    ConstraintProximity min_pinfo;

    double minDist = std::numeric_limits<double>::max();

    std::unique_ptr<BaseConstraintIterator> iterator = geo->getIterator(T);

    const defaulttype::Vector3 P = T.getPosition();
    while(! iterator->end(min_pinfo)) {
        int e = iterator->getElement();

        ConstraintProximity pinfo(geo,e);
        double dist = geo->projectPoint(P,pinfo);

        if (! filter || filter->filter(T,pinfo)) {
            if (dist < minDist) {
                min_pinfo = pinfo;
                minDist = dist;
            }
        }

        iterator->next();
    }

    return min_pinfo;
}






} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
