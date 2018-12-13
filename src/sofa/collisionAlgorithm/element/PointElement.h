#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>

namespace sofa
{

namespace collisionAlgorithm
{

class PointElement : public ConstraintElement
{
    friend class PointProximity;
    friend class PointGeometry;

public:
    PointElement(PointGeometry *geo, unsigned pid)
        : ConstraintElement()
        , m_geometry(geo)
    {
        m_pid = pid;
    }

    inline const PointGeometry* geometry() const override
    {
        return  m_geometry;
    }

    static ConstraintElement::UPtr createElement(PointGeometry *geo, unsigned pid)
    {
        return std::unique_ptr<PointElement>(new PointElement(geo,pid));
    }

    inline size_t getNbControlPoints() const override
    {
        return 1;
    }

    ConstraintProximity::SPtr getControlPoint(int /*i*/) const override
    {
        return m_geometry->createProximity(this);
    }

    //this function project the point P on the element and return the corresponding proximity
    ConstraintProximity::SPtr project(defaulttype::Vector3 /*P*/) const override
    {
        return getControlPoint(0);
    }

    void draw(const core::visual::VisualParams * /*vparams*/ ) const override
    {
        glColor4dv(geometry()->d_color.getValue().data());

        glBegin(GL_POINTS);
            glVertex3dv(getControlPoint(0)->getPosition().data());
        glEnd();
    }

protected:
    unsigned m_pid;
    const PointGeometry* m_geometry;
};

}

}
