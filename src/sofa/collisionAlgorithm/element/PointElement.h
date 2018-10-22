#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

class PointElement : public ConstraintElement {
    friend class PointProximity;
    friend class PointGeometry;

public:
    PointElement(PointGeometry *geo, unsigned pid) : ConstraintElement(geo,1) {
        m_pid = pid;
    }

    static ConstraintElement::UPtr createElement(PointGeometry *geo, unsigned pid) {
        return std::unique_ptr<PointElement>(new PointElement(geo,pid));
    }

    ConstraintProximity::SPtr getControlPoint(int /*i*/) const {
        return PointGeometry::createProximity(this);
    }

    //this function project the point P on the element and return the corresponding proximity
    ConstraintProximity::SPtr project(defaulttype::Vector3 /*P*/) const {
        return getControlPoint(0);
    }

    PointGeometry * geometry() const {
        return (PointGeometry * ) m_geometry;
    }

    void draw(const core::visual::VisualParams * /*vparams*/ ) const {
        glColor4dv(geometry()->d_color.getValue().data());

        glBegin(GL_POINTS);
            glVertex3dv(getControlPoint(0)->getPosition().data());
        glEnd();
    }

protected:
    unsigned m_pid;
};

}

}
