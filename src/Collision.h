#pragma once

#include <collisionAlgorithm.h>
#include <BaseGeometry.h>
#include <qopengl.h>

namespace collisionAlgorithm {

typedef std::pair<ConstraintProximityPtr,ConstraintProximityPtr> PairProximity;
typedef std::vector<PairProximity> PairProximityVector;

class Collision : public BaseObject {
public :
    Port<BaseObject> p_type;

    Collision();

    virtual void handleEvent(Event * e) {
        if (dynamic_cast<AnimateBeginEvent *>(e)) m_dirty= true;
    }

    static std::string getObjectCategory() {
        return std::string("Collision");
    }

    void draw(const VisualParams * vparams) {
        if (! vparams->displayFlags().getShowCollisionModels()) return;

        glDisable(GL_LIGHTING);
        glColor4f(0,1,0,1);
        glBegin(GL_LINES);

        for (unsigned i=0;i<m_pairDetection.size();i++) {
            glVertex3dv(m_pairDetection[i].first->getPosition().data());
            glVertex3dv(m_pairDetection[i].second->getPosition().data());
        }
        glEnd();
    }

    void computeCollisionDetection() {
        if (! m_dirty) return;

        Timer::beginStep("Collision");
        m_pairDetection.clear();
        processAlgorithm();
        m_dirty = false;
        Timer::endStep("Collision");
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

