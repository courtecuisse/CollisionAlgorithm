#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>

namespace sofa::collisionAlgorithm {

class TriangleElementGeometry {
public:
    virtual type::Vector3 getPosition(unsigned pid) const = 0;

    virtual BaseProximity::SPtr createProximity(unsigned p0, unsigned p1,unsigned p2, double f0,double f1,double f2) const = 0;
};

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

    TriangleElement(const TriangleElementGeometry * c, unsigned p0,unsigned p1,unsigned p2)
    : m_geometry(c), m_p0(p0), m_p1(p1), m_p2(p2) {}

    void update() override {
        m_tinfo.P0 = getP0();
        m_tinfo.P1 = getP1();
        m_tinfo.P2 = getP2();

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

    inline BaseProximity::SPtr createProximity(double f0,double f1,double f2) const { return m_geometry->createProximity(m_p0,m_p1,m_p2,f0,f1,f2); }

    inline const TriangleInfo & getTriangleInfo() const { return m_tinfo; }

    inline type::Vector3 getP0() const { return m_geometry->getPosition(m_p0); }

    inline type::Vector3 getP1() const { return m_geometry->getPosition(m_p1); }

    inline type::Vector3 getP2() const { return m_geometry->getPosition(m_p2); }

private:
    const TriangleElementGeometry * m_geometry;
    unsigned m_p0,m_p1,m_p2;
    TriangleInfo m_tinfo;
};

}
