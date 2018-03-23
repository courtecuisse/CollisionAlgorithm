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
    : p_type(this,"Any") {}

    virtual void processAlgorithm() = 0;

    void beginStep() {
        m_pairDetection.clear();
//        processAlgorithm();
    }

    static std::string getObjectType() {
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

    PairProximityVector & getCollisionPairs() {
        return m_pairDetection;
    }

protected:
    PairProximityVector m_pairDetection;
};

}

