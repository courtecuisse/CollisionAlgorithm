#ifndef SOFA_COMPONENT_CONSTRAINT_COLLISIONALGORITHM_H
#define SOFA_COMPONENT_CONSTRAINT_COLLISIONALGORITHM_H

#include "CollisionDetectionAlgorithm.h"
#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/core/behavior/MechanicalState.h>

namespace sofa {

namespace core {

namespace behavior {


CollisionDetectionAlgorithm::CollisionDetectionAlgorithm(){}


CollisionDetectionAlgorithm::PariProximity CollisionDetectionAlgorithm::getClosestElements(unsigned fromId,BaseGeometry *geoFrom,BaseGeometry *geoTo) {
    double min_dist = std::numeric_limits<double>::max();

    PariProximity min_pair;
    min_pair.first = NULL;
    min_pair.second = NULL;

    ElementIteratorPtr it;
    if (BroadPhase* broad = dynamic_cast<BroadPhase*>(geoTo)) it = broad->getCloseElementsIterator(geoFrom->getElementProximity(fromId));
    else it = geoTo->getElementIterator();

    for(;! it->end();it->next()) {
        ConstraintProximityPtr pfrom = geoFrom->getElementProximity(fromId);
        defaulttype::Vector3 P = pfrom->getPosition();

        ConstraintProximityPtr pto = geoTo->getElementProximity(it->getId());
        pto->refineToClosestPoint(P);

        //iterate until to find the correct location on pfrom
        int it=0;
        while (it<30) {
            pfrom->refineToClosestPoint(pto->getPosition());
            if ((P-pfrom->getPosition()).norm() < 0.0001) break;
            P = pfrom->getPosition();
            pto->refineToClosestPoint(P);
            it++;
        }

        //compute all the distances with to elements
        double dist = (pfrom->getPosition() - pto->getPosition()).norm();

        if (dist<min_dist) {
            min_dist = dist;
            min_pair.first = pfrom;
            min_pair.second = pto;
        }
    }

    return min_pair;
}

void CollisionDetectionAlgorithm::processAlgorithm(BaseGeometry *geoFrom, BaseGeometry *geoTo, helper::vector<PariProximity> &output) {
    if (geoFrom == NULL) return;
    if (geoTo == NULL) return;

    //for all the elements in the box from
    for(ElementIteratorPtr it = geoFrom->getElementIterator();! it->end();it->next()) {
        PariProximity prox = getClosestElements(it->getId(),geoFrom,geoTo);

        if (prox.first != NULL && prox.second!=NULL) output.push_back(prox);
    }
}

//void CollisionDetectionAlgorithm::draw(const core::visual::VisualParams * vparams) {
//    glDisable(GL_LIGHTING);

//    glBegin(GL_TRIANGLES);

//    for (std::set<int>::iterator it=m_selectElements.begin(); it!=m_selectElements.end(); ++it) {
//        ConstraintProximityPtr pto = m_geoTo->getElementProximity(*it);

//        helper::vector<defaulttype::Vector3> controlpoint;
//        pto->getControlPoints(controlpoint);
//        for (unsigned i=0;i<controlpoint.size();i++) helper::gl::glVertexT(controlpoint[i]);

//    }

//    glEnd();


//}

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
