#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>

namespace sofa::collisionAlgorithm {

class TriangleElement : public TBaseElement<std::function<BaseProximity::SPtr(const TriangleElement *, double ,double , double )> > {
public:

    typedef std::shared_ptr<TriangleElement> SPtr;

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
        type::Vec3d N;
    };

    TriangleElement(unsigned p0,unsigned p1,unsigned p2)
    : m_p0(p0), m_p1(p1), m_p2(p2) {}

    void update() override {
        m_tinfo.P0 = createProximity(1,0,0)->getPosition();
        m_tinfo.P1 = createProximity(0,1,0)->getPosition();
        m_tinfo.P2 = createProximity(0,0,1)->getPosition();

        m_tinfo.v0 = m_tinfo.P1 - m_tinfo.P0;
        m_tinfo.v1 = m_tinfo.P2 - m_tinfo.P0;
        m_tinfo.N=cross(m_tinfo.v0,m_tinfo.v1);
        m_tinfo.area = m_tinfo.N.norm()/2;
        m_tinfo.N.normalize();

        m_tinfo.d00 = dot(m_tinfo.v0,m_tinfo.v0);
        m_tinfo.d01 = dot(m_tinfo.v0,m_tinfo.v1);
        m_tinfo.d11 = dot(m_tinfo.v1,m_tinfo.v1);

        m_tinfo.invDenom = 1.0 / (m_tinfo.d00 * m_tinfo.d11 - m_tinfo.d01 * m_tinfo.d01);

        m_tinfo.ax1 = m_tinfo.v0;
        m_tinfo.ax2 = m_tinfo.v0.cross(m_tinfo.N);

        m_tinfo.ax1.normalize();
        m_tinfo.ax2.normalize();
    }

    inline BaseProximity::SPtr createProximity(double f0,double f1,double f2) const {
        return m_createProxFunc(this,f0,f1,f2);
    }

    inline const TriangleInfo & getTriangleInfo() const { return m_tinfo; }

    inline unsigned getP0() const { return m_p0; }

    inline unsigned getP1() const { return m_p1; }

    inline unsigned getP2() const { return m_p2; }

private:
    unsigned m_p0,m_p1,m_p2;
    TriangleInfo m_tinfo;    
};

}
