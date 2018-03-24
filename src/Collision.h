#pragma once

#include <BaseGeometry.h>
#include <GL/gl.h>

namespace collisionAlgorithm {

typedef std::pair<ConstraintProximityPtr,ConstraintProximityPtr> PairProximity;
typedef std::vector<PairProximity> PairProximityVector;

class Collision : public BaseObject {
public :
    PortIn<BaseObject> p_type;

    Collision()
    : p_type(this,"Out") {
        m_dirty = true;
    }

    virtual void processAlgorithm() = 0;

    void beginStep() {
        if (m_dirty) {
            m_dirty = false;
            m_pairDetection.clear();
            processAlgorithm();
        }
    }

    void endStep() {
        m_dirty = true;
    }

    static std::string getObjectType() {
        return std::string("Collision");
    }

    void draw(const VisualParams * vparams) {        
        processAlgorithm();

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

    PairProximityVector & getCollisionPairs() {
        beginStep();
        return m_pairDetection;
    }

protected:
    PairProximityVector m_pairDetection;
    bool m_dirty;
};

}

