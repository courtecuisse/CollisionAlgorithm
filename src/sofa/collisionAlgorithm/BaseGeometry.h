#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/objectmodel/DataLink.h>
#include <sofa/core/BehaviorModel.h>
#include <memory>
#include <map>
#include <vector>
#include <qopengl.h>

namespace sofa {

namespace collisionAlgorithm {

class BaseGeometry : public core::BehaviorModel {
public:
    SOFA_CLASS(BaseGeometry,core::objectmodel::BaseObject);

    typedef Data<helper::vector<defaulttype::Vector3> > DataVecCoord;

    DataLink<sofa::core::behavior::MechanicalState<defaulttype::Vec3dTypes> > d_state;

    BaseGeometry()
    : d_state(initData(&d_state, "mstate", "this")) {}

    void updatePosition(SReal ) {
        prepareDetection();
    }

    unsigned getNbElements() {
        return (unsigned) m_elements.size();
    }

    const ConstraintElement * getElement(unsigned i) {
        return m_elements[i].get();
    }

    virtual sofa::core::behavior::MechanicalState<defaulttype::Vec3dTypes> * getState() {
        return d_state.get();
    }

    void draw(const core::visual::VisualParams *vparams) {
        if (! vparams->displayFlags().getShowCollisionModels()) return;

        glDisable(GL_LIGHTING);

        for(unsigned i=0;i<m_elements.size();i++) {
            m_elements[i]->draw(vparams);
        }
    }

protected:
    std::vector<ConstraintElement::UPtr> m_elements;

    virtual void prepareDetection() {}
};

}

}
