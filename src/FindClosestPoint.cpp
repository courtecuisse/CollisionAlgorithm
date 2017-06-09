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

ConstraintProximityPtr CollisionAlgorithm::findClosestProximity(const ConstraintProximityPtr &T, BaseGeometry * geo) {
    BaseDecorator * decorator = NULL;
    geo->getContext()->get(decorator, core::objectmodel::BaseContext::Local);

    BaseConstraintIteratorPtr iterator = (decorator == NULL) ? geo->getIterator() : decorator->getIterator(T);

    ConstraintProximityPtr min_pinfo = NULL;

    double minDist = std::numeric_limits<double>::max();

    helper::vector<CollisionFilter *> filters;
    geo->getContext()->get<CollisionFilter>(&filters, core::objectmodel::BaseContext::Local);

    const defaulttype::Vector3 P = T->getPosition();
    while(! iterator->end(min_pinfo)) {
        int e = iterator->getElement();

        ConstraintProximityPtr pinfo = geo->projectPoint(P,e);
        double dist = geo->getDistance(T,pinfo);

        if (dist < minDist) {
            for (unsigned i=0;i<filters.size();i++) {
                if (! filters[i]->filter(T,pinfo)) break;
            }

            min_pinfo = pinfo;
            minDist = dist;
        }

        iterator->next();
    }

    return min_pinfo;
}



} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
