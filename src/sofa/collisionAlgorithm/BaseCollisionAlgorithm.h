#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/core/collision/Pipeline.h>

namespace sofa {

namespace collisionAlgorithm {

typedef std::pair<ConstraintProximity::SPtr,ConstraintProximity::SPtr> PairProximity;
typedef helper::vector<PairProximity> PairProximityVector;

class BaseCollisionAlgorithm : public core::collision::Pipeline {
public :

    void draw(const core::visual::VisualParams * vparams) {
        if (! vparams->displayFlags().getShowCollisionModels()) return;
        glDisable(GL_LIGHTING);

        glColor4f(0,1,0,1);
        glBegin(GL_LINES);
        for (unsigned i=0;i<m_pairDetection.size();i++) {
            glVertex3dv(m_pairDetection[i].first->getPosition().data());
            glVertex3dv(m_pairDetection[i].second->getPosition().data());
        }
        glEnd();

//        for (unsigned i=0;i<m_pairDetection.size();i++) {
//            vparams->drawTool()->drawArrow(m_pairDetection[i].second->getPosition(),
//                                           m_pairDetection[i].second->getPosition() + m_pairDetection[i].second->getNormal(),
//                                           0.1,
//                                           defaulttype::Vector4(0,0,1,1));

//            vparams->drawTool()->drawArrow(m_pairDetection[i].first->getPosition(),
//                                           m_pairDetection[i].first->getPosition() + m_pairDetection[i].first->getNormal(),
//                                           0.1,
//                                           defaulttype::Vector4(0,0,1,1));
//        }

    }

    virtual void reset() {}

    virtual void computeCollisionReset() {}

    virtual void computeCollisionResponse() {}

    void computeCollisionDetection() {
        m_pairDetection.clear();
        processAlgorithm();
    }

    PairProximityVector & getCollisionPairs() {
        computeCollisionDetection();
        return m_pairDetection;
    }

    virtual void getState(std::set<sofa::core::behavior::MechanicalState<defaulttype::Vec3dTypes>* > & list_state) = 0;

    virtual std::set< std::string > getResponseList() const {
        std::set< std::string > res;
        return res;
    }

protected:
    virtual void doCollisionReset() {
         m_pairDetection.clear();
    }

    virtual void doCollisionDetection(const sofa::helper::vector<core::CollisionModel*>& /*collisionModels*/) {

    }

    virtual void doCollisionResponse() {

    }

    virtual void processAlgorithm() = 0;

    PairProximityVector m_pairDetection;
};

}

}
