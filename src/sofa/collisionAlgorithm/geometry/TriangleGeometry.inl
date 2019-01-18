#pragma once

#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
BaseElementIterator::UPtr TriangleGeometry<DataTypes>::begin(unsigned eid) const {
    return DefaultElementIterator<GEOMETRY, TriangleProximity<GEOMETRY> >::create(this, d_triangles.getValue(), eid);
}

template<class DataTypes>
void TriangleGeometry<DataTypes>::prepareDetection()
{
    const VecTriangles& triangles = d_triangles.getValue();

    const helper::ReadAccessor<DataVecCoord> & pos = this->l_state->read(core::VecCoordId::position());

    m_triangle_info.resize(triangles.size());
    m_triangle_normals.resize(triangles.size());

    for (size_t t=0 ; t<triangles.size() ; t++)
    {
        const Triangle& tri = triangles[t];

        //Compute Bezier Positions
        const defaulttype::Vector3 & p0 = pos[tri[0]];
        const defaulttype::Vector3 & p1 = pos[tri[1]];
        const defaulttype::Vector3 & p2 = pos[tri[2]];

        TriangleInfo & tinfo = m_triangle_info[t];
        tinfo.v0 = p1 - p0;
        tinfo.v1 = p2 - p0;

        tinfo.d00 = dot(tinfo.v0,tinfo.v0);
        tinfo.d01 = dot(tinfo.v0,tinfo.v1);
        tinfo.d11 = dot(tinfo.v1,tinfo.v1);

        tinfo.invDenom = 1.0 / (tinfo.d00 * tinfo.d11 - tinfo.d01 * tinfo.d01);

        tinfo.ax1 = tinfo.v0;
        m_triangle_normals[t] = tinfo.v0.cross(tinfo.v1);
        tinfo.ax2 = tinfo.v0.cross(m_triangle_normals[t]);

        tinfo.ax1.normalize();
        m_triangle_normals[t].normalize();
        tinfo.ax2.normalize();
    }
}


//proj_P must be on the plane
template<class DataTypes>
void TriangleGeometry<DataTypes>::computeBaryCoords(const defaulttype::Vector3 & proj_P,const TriangleInfo & tinfo, const defaulttype::Vector3 & p0, double & fact_u,double & fact_v, double & fact_w) const
{
    defaulttype::Vector3 v2 = proj_P - p0;

    double d20 = dot(v2,tinfo.v0);
    double d21 = dot(v2,tinfo.v1);

    fact_v = (tinfo.d11 * d20 - tinfo.d01 * d21) * tinfo.invDenom;
    fact_w = (tinfo.d00 * d21 - tinfo.d01 * d20) * tinfo.invDenom;
    fact_u = 1.0 - fact_v  - fact_w;
}

//Barycentric coordinates are computed according to
//http://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
template<class DataTypes>
void TriangleGeometry<DataTypes>::project(unsigned eid, const defaulttype::Vector3 & P, core::topology::BaseMeshTopology::Triangle & triangle, defaulttype::Vector3 & factor) const {
    const helper::ReadAccessor<DataVecCoord> & pos = this->l_state->read(core::VecCoordId::position());

    const TriangleInfo & tinfo = m_triangle_info[eid];
    triangle = d_triangles.getValue()[eid];

    defaulttype::Vector3 P0 = pos[triangle[0]];
    defaulttype::Vector3 P1 = pos[triangle[1]];
    defaulttype::Vector3 P2 = pos[triangle[2]];

    defaulttype::Vector3 x1x2 = P - P0;

    //corrdinate on the plane
    double c0 = dot(x1x2,tinfo.ax1);
    double c1 = dot(x1x2,tinfo.ax2);
    defaulttype::Vector3 proj_P = P0 + tinfo.ax1 * c0 + tinfo.ax2 * c1;

    double fact_u,fact_v,fact_w;

    computeBaryCoords(proj_P, tinfo, P0, fact_u,fact_v,fact_w);

    if (fact_u<0)
    {
        defaulttype::Vector3 v3 = P1 - P2;
        defaulttype::Vector3 v4 = proj_P - P2;
        double alpha = dot(v4,v3) / dot(v3,v3);

        if (alpha<0) alpha = 0;
        else if (alpha>1) alpha = 1;

        fact_u = 0;
        fact_v = alpha;
        fact_w = 1.0 - alpha;
    }
    else if (fact_v<0)
    {
        defaulttype::Vector3 v3 = P0 - P2;
        defaulttype::Vector3 v4 = proj_P - P2;
        double alpha = dot(v4,v3) / dot(v3,v3);

        if (alpha<0) alpha = 0;
        else if (alpha>1) alpha = 1;

        fact_u = alpha;
        fact_v = 0;
        fact_w = 1.0 - alpha;
    }
    else if (fact_w<0)
    {
        defaulttype::Vector3 v3 = P1 - P0;
        defaulttype::Vector3 v4 = proj_P - P0;
        double alpha = dot(v4,v3) / dot(v3,v3);

        if (alpha<0) alpha = 0;
        else if (alpha>1) alpha = 1;

        fact_u = 1.0 - alpha;
        fact_v = alpha;
        fact_w = 0;
    }

    factor[0] = fact_u;
    factor[1] = fact_v;
    factor[2] = fact_w;
}

template<class DataTypes>
void TriangleGeometry<DataTypes>::init()
{
    ///To remove if we think every input has to be explicit
    if(d_triangles.getValue().empty())
    {
        msg_warning(this) << "Triangles are not set (data is empty). Will set from topology if present in the same context";
        sofa::core::topology::BaseMeshTopology* topology{nullptr};
        this->getContext()->get(topology);
        if(!topology)
        {
            msg_error(this) << "No topology to work with ; giving up.";
        }
        else
        {
            if(topology->getTriangles().empty())
            {
                msg_error(this) << "No topology with triangles to work with ; giving up.";
            }
            else
            {
                d_triangles.setParent(topology->findData("triangles"));
            }
        }
    }
}

template<class DataTypes>
void TriangleGeometry<DataTypes>::draw(const core::visual::VisualParams * vparams) {
    if (! vparams->displayFlags().getShowCollisionModels())
        return;

    if (this->d_color.getValue()[3] == 0.0)
        return;

    glDisable(GL_LIGHTING);

    double delta = 0.1;
    defaulttype::Vector4 color = this->d_color.getValue();
    const helper::ReadAccessor<DataVecCoord> & pos = this->l_state->read(core::VecCoordId::position());

    glBegin(GL_TRIANGLES);
    for (unsigned i=0;i<d_triangles.getValue().size();i++) {
        const Triangle& tri = this->d_triangles.getValue()[i];

        glColor4f(fabs(color[0]-delta),color[1],color[2],color[3]);
        glVertex3dv(pos[tri[0]].data());
        glColor4f(color[0],fabs(color[1]-delta),color[2],color[3]);
        glVertex3dv(pos[tri[1]].data());
        glColor4f(color[0],color[1],fabs(color[2]-delta),color[3]);
        glVertex3dv(pos[tri[2]].data());
    }
    glEnd();
}

}

}
