#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>

namespace sofa::collisionAlgorithm {

class PointElement : public TBaseElement<std::function<BaseProximity::SPtr(const PointElement *)> > {
public:

    typedef std::shared_ptr<PointElement> SPtr;

    PointElement(unsigned p)
    : m_point(p) {}

    void update() override {}

    inline BaseProximity::SPtr createProximity() const {
        return m_createProxFunc(this);
    }

    inline unsigned getP0() const { return m_point; }

private:
    unsigned m_point;
};


}
