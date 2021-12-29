#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>

namespace sofa::collisionAlgorithm {

class PointElementGeometry {
public:

    virtual type::Vector3 getPosition(unsigned pid) const = 0;

    virtual BaseProximity::SPtr createProximity(unsigned p0) const = 0;

};

class PointElement : public BaseElement {
public:
    PointElement(const PointElementGeometry * g,unsigned p)
    : m_geometry(g), m_point(p) {}

    void update() override {}

    inline BaseProximity::SPtr createProximity() const { return m_geometry->createProximity(m_point); }

    inline type::Vector3 getP0() const { return m_geometry->getPosition(m_point); }

private:
    const PointElementGeometry * m_geometry;
    unsigned m_point;
};


}
