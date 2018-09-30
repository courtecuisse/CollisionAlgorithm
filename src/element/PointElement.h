#pragma once

#include <geometry/PointGeometry.h>
#include <qopengl.h>

namespace sofa {

namespace collisionAlgorithm {

/**************************************************************************/
/******************************ELEMENT*************************************/
/**************************************************************************/

class PointElement : public ConstraintElement {
    friend class PointProximity;
    friend class PointGeometry;

public:

    inline ConstraintProximityPtr createProximity(const int i);

    PointElement(PointGeometry *geo, unsigned pid) : ConstraintElement(geo,1) {
        m_pid = pid;
    }

    ConstraintProximityPtr getControlPoint(const int i) {
        return createProximity(i);
    }

    //this function project the point P on the element and return the corresponding proximity
    ConstraintProximityPtr project(defaulttype::Vector3 /*P*/) {
        return getControlPoint(0);
    }

    PointGeometry * geometry() {
        return (PointGeometry * ) m_geometry;
    }

    void draw(const core::visual::VisualParams * /*vparams*/ ) {
        glColor4dv(geometry()->d_color.getValue().data());

        glBegin(GL_POINTS);
            glVertex3dv(getControlPoint(0)->getPosition().data());
        glEnd();
    }

protected:
    unsigned m_pid;
};

/**************************************************************************/
/******************************PROXIMITY***********************************/
/**************************************************************************/

class PointProximity : public ConstraintProximity {
public :
    PointProximity(PointElement * elmt) : ConstraintProximity(elmt) {}

    defaulttype::Vector3 getPosition(core::VecCoordId v) const {
        const helper::ReadAccessor<DataVecCoord> & pos = m_state->read(v);
        return pos[element()->m_pid];
    }

    defaulttype::Vector3 getNormal() const {
        return defaulttype::Vector3(1,0,0);
    }

    std::map<unsigned,double> getContributions() {
        std::map<unsigned,double> res;

        res[element()->m_pid] = 1.0;

        return res;
    }

    inline PointElement * element() const {
        return (PointElement*) m_element;
    }
};

ConstraintProximityPtr PointElement::createProximity(const int /*i*/) {
    return std::make_shared<PointProximity>(this);
}

}

}
