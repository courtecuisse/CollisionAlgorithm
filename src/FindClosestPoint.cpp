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

inline bool can_filter(const ConstraintProximityPtr &T,const ConstraintProximityPtr &P, helper::vector<CollisionFilter *> & filters) {
    for (unsigned i=0;i<filters.size();i++) {
//        std::cout << filters[i]->getName() << std::endl;
        if (! filters[i]->filter(T,P)) return false;
    }

    return true;
}

ConstraintProximityPtr CollisionAlgorithm::findClosestProximity(const ConstraintProximityPtr &T, BaseGeometry * geo) {
    if (T == NULL) return NULL;

    const defaulttype::Vector3 P = T->getPosition();

    BaseDecorator * decorator = NULL;
    geo->getContext()->get(decorator, core::objectmodel::BaseContext::Local);

    BaseConstraintIteratorPtr iterator = (decorator == NULL) ? geo->getIterator() : decorator->getIterator(T);

    ConstraintProximityPtr min_pinfo = NULL;
    double minDist = std::numeric_limits<double>::max();

    helper::vector<CollisionFilter *> filters;
    geo->getContext()->get<CollisionFilter>(&filters, core::objectmodel::BaseContext::Local);

    while(! iterator->end(min_pinfo)) {        
        int e = iterator->getElement();

        ConstraintProximityPtr pinfo = geo->projectPoint(P,e);
        if (pinfo == NULL) continue;
        double dist = pinfo->distance(T);

        if (dist < minDist) {
            if (can_filter(T,pinfo,filters)) {
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
