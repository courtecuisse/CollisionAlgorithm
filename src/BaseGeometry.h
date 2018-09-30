#pragma once

#include <collisionAlgorithm.h>
#include <memory>
#include <map>
#include <vector>
#include <BaseElement.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/MechanicalState.h>

namespace sofa {

namespace collisionAlgorithm {

class BaseGeometry : public core::objectmodel::BaseObject {
public:

    typedef Data<helper::vector<defaulttype::Vector3> > DataVecCoord;

//    Port<Topology,_REQUIRED> p_topology;

    BaseGeometry() {
        m_dirty = true;
    }

    void init() {
        m_elements.clear();
    }

    virtual void handleEvent(core::objectmodel::Event * e) {
        if (dynamic_cast<simulation::AnimateBeginEvent *>(e)) m_dirty=true;
    }

    unsigned getNbElements() {
        if (m_dirty) {
            prepareDetection();
            m_dirty = false;
        }

        return (unsigned) m_elements.size();
    }

    ConstraintElementPtr getElement(unsigned i) {
        if (m_dirty) {
            prepareDetection();
            m_dirty = false;
        }

        return m_elements[i];
    }

//    ConstraintProximityPtr project(const Vector3 & P) {
//        double min_dist = std::numeric_limits<double>::max();
//        ConstraintProximityPtr min_prox = NULL;

//        for (unsigned i=0;i<m_elements.size();i++) {
//            ConstraintProximityPtr pdest = m_elements[i]->project(P);
//            double dist = (P - pdest->getPosition()).norm();
//            if (dist<min_dist) {
//                min_dist = dist;
//                min_prox = pdest;
//            }
//        }

//        return min_prox;
//    }

    virtual sofa::core::behavior::MechanicalState<defaulttype::Vec3dTypes> * getState() = 0;

    void draw(const core::visual::VisualParams *vparams) {
        if (! vparams->displayFlags().getShowCollisionModels()) return;

        glDisable(GL_LIGHTING);

        for(unsigned i=0;i<m_elements.size();i++) {
            m_elements[i]->draw(vparams);
        }
    }


protected:
    std::vector<ConstraintElementPtr> m_elements;
    bool m_dirty;

    virtual void prepareDetection() {}
};

}

}
