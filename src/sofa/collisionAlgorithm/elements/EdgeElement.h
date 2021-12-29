#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>

namespace sofa::collisionAlgorithm {

class EdgeElementGeometry {
public:

    virtual type::Vector3 getPosition(unsigned pid) const = 0;

    virtual BaseProximity::SPtr createProximity(unsigned p0, unsigned p1,double f0,double f1) const = 0;
};

class EdgeElement : public BaseElement {
public:

    EdgeElement(const EdgeElementGeometry * c, unsigned p0,unsigned p1)
    : m_geometry(c), m_p0(p0), m_p1(p1) {}

    void update() override {}

    inline BaseProximity::SPtr createProximity(double f0,double f1) const { return m_geometry->createProximity(m_p0,m_p1,f0,f1); }

    inline type::Vector3 getP0() const { return m_geometry->getPosition(m_p0); }

    inline type::Vector3 getP1() const { return m_geometry->getPosition(m_p1); }

private:
    const EdgeElementGeometry * m_geometry;
    unsigned m_p0,m_p1;
};

}
