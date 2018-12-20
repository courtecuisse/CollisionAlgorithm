#pragma once

#include <sofa/collisionAlgorithm/geometry/EdgeGeometry.h>

namespace sofa
{

namespace collisionAlgorithm
{

class EdgeElement : public ConstraintElement
{
    friend class EdgeGeometry;
    friend class EdgeProximity;

public:

    EdgeElement(EdgeGeometry * geo,unsigned eid)
        : ConstraintElement()
        , m_geometry(geo)
    {
        m_eid = eid;

        const EdgeGeometry::VecEdges & edges = geometry()->edges();

        m_pid[0] = edges[eid][0];
        m_pid[1] = edges[eid][1];
    }

    inline const EdgeGeometry* geometry() const override
    {
        return m_geometry;
    }

    static ConstraintElement::UPtr createElement(EdgeGeometry * geo,unsigned eid)
    {
        return EdgeElement::UPtr(new EdgeElement(geo,eid));
    }

    inline size_t getNbControlPoints() const override
    {
        return 2;
    }

    ConstraintProximity::SPtr getControlPoint(int cid) const override
    {
        if (cid == 0)
            return m_geometry->createProximity(this,1,0);
        else if (cid == 1)
            return m_geometry->createProximity(this,0,1);

        return m_geometry->createProximity(this,1.0/2.0,1.0/2.0);
    }

    //this function project the point P on the element and return the corresponding proximity
    ConstraintProximity::SPtr project(defaulttype::Vector3 P) const override
    {
        double fact_u,fact_v;

        const helper::ReadAccessor<DataVecCoord> & pos = geometry()->getState()->read(core::VecCoordId::position());

        defaulttype::Vector3 P1 = pos[m_pid[0]];
        defaulttype::Vector3 P2 = pos[m_pid[1]];

        defaulttype::Vector3 v = P2-P1;
        fact_v = dot(P - P1,v) / dot(v,v);

        if (fact_v<0.0) fact_v = 0.0;
        else if (fact_v>1.0) fact_v = 1.0;

        fact_u = 1.0-fact_v;

        return m_geometry->createProximity(this,fact_u,fact_v);
    }

    void draw(const core::visual::VisualParams * /*vparams*/) const override
    {
        glColor4dv(geometry()->d_color.getValue().data());

        glBegin(GL_LINES);
            glVertex3dv(getControlPoint(0)->getPosition().data());
            glVertex3dv(getControlPoint(1)->getPosition().data());
        glEnd();
    }

protected:
    const EdgeGeometry* m_geometry;
    unsigned m_pid[2];
    unsigned m_eid;
};

}

}
