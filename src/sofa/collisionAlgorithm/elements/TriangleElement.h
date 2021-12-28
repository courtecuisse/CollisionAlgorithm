#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>

namespace sofa::collisionAlgorithm {

template<class PROXIMITY>
class TriangleElement : public BaseElement {
public:

    struct TriangleInfo
    {
        type::Vec3d v0,v1;
        double d00;
        double d01;
        double d11;
        double invDenom;
        double area;

        type::Vec3d ax1,ax2;
        type::Vec3d P0,P1,P2;
    };

    TriangleElement(BaseGeometry * geo, unsigned p0,unsigned p1,unsigned p2)
    : m_geometry(geo), m_p0(p0), m_p1(p1), m_p2(p2) {}

    void update() override {
        m_tinfo.P0 = m_p0->getPosition();
        m_tinfo.P1 = m_p1->getPosition();
        m_tinfo.P2 = m_p2->getPosition();

        m_tinfo.v0 = m_tinfo.P1 - m_tinfo.P0;
        m_tinfo.v1 = m_tinfo.P2 - m_tinfo.P0;
        type::Vec3d N=cross(m_tinfo.v0,m_tinfo.v1);
        m_tinfo.area = N.norm()/2;
        N.normalize();

        m_tinfo.d00 = dot(m_tinfo.v0,m_tinfo.v0);
        m_tinfo.d01 = dot(m_tinfo.v0,m_tinfo.v1);
        m_tinfo.d11 = dot(m_tinfo.v1,m_tinfo.v1);

        m_tinfo.invDenom = 1.0 / (m_tinfo.d00 * m_tinfo.d11 - m_tinfo.d01 * m_tinfo.d01);

        m_tinfo.ax1 = m_tinfo.v0;
        m_tinfo.ax2 = m_tinfo.v0.cross(N);

        m_tinfo.ax1.normalize();
        m_tinfo.ax2.normalize();
    }



private:
    const BaseGeometry * m_geometry;
    unsigned m_p0,m_p1,m_p2;
    TriangleInfo m_tinfo;
};

}
