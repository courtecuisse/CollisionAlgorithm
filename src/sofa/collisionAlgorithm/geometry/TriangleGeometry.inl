#pragma once

#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>
#include <sofa/collisionAlgorithm/element/TriangleElement.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

ConstraintProximity::SPtr TriangleGeometry::createProximity(const TriangleElement * elmt,double f1,double f2,double f3,bool phongNormals) const
{
    return std::shared_ptr<TriangleProximity>(new TriangleProximity(elmt,f1,f2,f3,phongNormals));
}

void TriangleGeometry::prepareDetection()
{
    const VecTriangles& triangles = d_triangles.getValue();

    if (m_elements.size() != triangles.size())
        init();

    const helper::ReadAccessor<DataVecCoord> & pos = getState()->read(core::VecCoordId::position());

    m_pointNormal.resize(pos.size());

    m_triangle_info.resize(triangles.size());

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
        tinfo.tn = tinfo.v0.cross(tinfo.v1);
        tinfo.ax2 = tinfo.v0.cross(tinfo.tn);

        tinfo.ax1.normalize();
        tinfo.tn.normalize();
        tinfo.ax2.normalize();
    }

    if(triangles.size() > 0)
    {
        m_pointNormal.resize(pos.size());
        //TODO: remove need of topologycontainer
        for (size_t p=0;p<pos.size();p++)
        {
            const core::topology::BaseMeshTopology::TrianglesAroundVertex & tav = l_topology->getTrianglesAroundVertex(p);
            m_pointNormal[p] = defaulttype::Vector3(0,0,0);
            for (size_t t=0;t<tav.size();t++)
            {
                m_pointNormal[p] += this->m_triangle_info[tav[t]].tn;
            }
            m_pointNormal[p].normalize();

        }
    }
}

void TriangleGeometry::init()
{
    if(d_phongInterpolation.getValue() && !l_topology.get())
    {
        msg_error(this) << "Phong Interpolation needs (entire) topology.Phong will be disabled.";
        d_phongInterpolation.setValue(false);
    }

    if(d_triangles.getValue().empty())
    {
        msg_warning(this) << "Triangles are not set (data is empty). Will set from topology if given";
        if(!l_topology.get())
        {
            msg_error(this) << "No topology to work with ; giving up.";
        }
        else
        {
            d_triangles.setParent(l_topology->findData("triangles"));
        }
    }

    m_elements.clear();
    const VecTriangles& triangles = d_triangles.getValue();

    for (size_t i=0;i<triangles.size();i++)
    {
        m_elements.push_back(TriangleElement::createElement(this,i));
    }

    prepareDetection();
}

}

}
