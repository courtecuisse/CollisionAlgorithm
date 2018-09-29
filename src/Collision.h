#pragma once

#include <collisionAlgorithm.h>
#include <BaseGeometry.h>
#include <BaseElement.h>
#include <qopengl.h>
#include <sofa/core/collision/Pipeline.h>

namespace sofa {

namespace collisionAlgorithm {

typedef std::pair<ConstraintProximityPtr,ConstraintProximityPtr> PairProximity;
typedef std::vector<PairProximity> PairProximityVector;

class Collision : public core::collision::Pipeline {
public :

    Collision();

    virtual void handleEvent(core::objectmodel::Event * e) {
        if (dynamic_cast<simulation::AnimateBeginEvent *>(e)) m_dirty= true;
    }

    static std::string getObjectCategory() {
        return std::string("Collision");
    }

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

        for (unsigned i=0;i<m_pairDetection.size();i++) {
            vparams->drawTool()->drawArrow(m_pairDetection[i].second->getPosition(),
                                           m_pairDetection[i].second->getPosition() + m_pairDetection[i].second->getNormal(),
                                           0.1,
                                           defaulttype::Vector4(0,0,1,1));

            vparams->drawTool()->drawArrow(m_pairDetection[i].first->getPosition(),
                                           m_pairDetection[i].first->getPosition() + m_pairDetection[i].first->getNormal(),
                                           0.1,
                                           defaulttype::Vector4(0,0,1,1));
        }

    }

    void computeCollisionDetection() {
        if (! m_dirty) return;
        m_pairDetection.clear();
        processAlgorithm();

        m_dirty = false;
    }

    PairProximityVector & getCollisionPairs() {
        computeCollisionDetection();
        return m_pairDetection;
    }

protected:
    PairProximityVector m_pairDetection;
    bool m_dirty;

    virtual void processAlgorithm() = 0;
};

}

}
