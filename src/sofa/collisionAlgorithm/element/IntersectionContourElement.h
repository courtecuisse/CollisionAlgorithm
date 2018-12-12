#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/geometry/IntersectionContourGeometry.h>
#include <sofa/collisionAlgorithm/element/IntersectionContourElement.h>

namespace sofa
{

namespace collisionAlgorithm
{

class IntersectionContourElement : public ConstraintElement
{
    friend class IntersectionContourProximity;
    friend class IntersectionContourGeometry;
public:

    IntersectionContourElement(IntersectionContourGeometry *geo, unsigned pid1, unsigned pid2,double f1,double f2)
        : ConstraintElement()
        , m_geometry(geo)
    {
        m_pid[0] = pid1;
        m_pid[1] = pid2;

        m_fact[0] = f1;
        m_fact[1] = f2;
    }

    static ConstraintElement::UPtr createElement(IntersectionContourGeometry *geo, unsigned pid1, unsigned pid2,double f1,double f2)
    {
        return std::unique_ptr<IntersectionContourElement>(new IntersectionContourElement(geo,pid1,pid2,f1,f2));
    }

    inline size_t getNbControlPoints() const override
    {
        return 1;
    }

    // TODO?
    ConstraintProximity::SPtr project(defaulttype::Vector3 /*P*/) const override
    {
        return ConstraintProximity::SPtr();
//        return createProximity();
    }

    // TODO?
    ConstraintProximity::SPtr getControlPoint(const int /*i*/) const override
    {
        return ConstraintProximity::SPtr();
//        return createProximity();
    }

    inline const IntersectionContourGeometry* geometry() const override
    {
        return m_geometry;
    }

    void draw(const core::visual::VisualParams * /*vparams*/) const override
    {
        glColor4dv(geometry()->d_color.getValue().data());

        glBegin(GL_POINTS);
        glPointSize(20);
        glVertex3dv(getControlPoint(0)->getPosition().data());
        glEnd();
    }

protected:
    const IntersectionContourGeometry* m_geometry;
    unsigned m_pid[2];
    double m_fact[2];
};

}

}
