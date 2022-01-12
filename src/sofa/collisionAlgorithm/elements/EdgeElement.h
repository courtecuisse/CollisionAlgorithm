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

    inline unsigned getP0() const { return m_p0; }

    inline unsigned getP1() const { return m_p1; }

    void getControlProximities(std::vector<BaseProximity::SPtr> & res) const override {
        res.push_back(createProximity(1,0));
        res.push_back(createProximity(0,1));
    }

private:
    unsigned m_p0,m_p1;
};

}
