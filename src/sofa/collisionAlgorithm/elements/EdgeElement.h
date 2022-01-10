#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>

namespace sofa::collisionAlgorithm {

class EdgeElement : public TBaseElement<std::function<BaseProximity::SPtr(const EdgeElement *, double ,double )> > {
public:

    typedef std::shared_ptr<EdgeElement> SPtr;

    EdgeElement(unsigned p0,unsigned p1)
    : m_p0(p0), m_p1(p1) {}

    void update() override {}

    inline BaseProximity::SPtr createProximity(double f0,double f1) const {
        return m_createProxFunc(this,f0,f1);
    }

private:
    unsigned m_p0,m_p1;
};

}
