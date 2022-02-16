#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>

namespace sofa::collisionAlgorithm {

class TetrahedronElement : public TBaseElement<std::function<BaseProximity::SPtr(const TetrahedronElement *, double ,double , double, double )> > {
public:

    using Inherit = TBaseElement;
    typedef std::shared_ptr<TetrahedronElement> SPtr;

    TetrahedronElement(unsigned eid, unsigned p0,unsigned p1,unsigned p2,unsigned p3,Inherit::ProxCreatorFunc f)
    : TBaseElement(eid, f), m_p0(p0), m_p1(p1), m_p2(p2), m_p3(p3) {}

    void update() {}

    inline BaseProximity::SPtr createProximity(double f0,double f1,double f2, double f3) const {
        return m_createProxFunc(this,f0,f1,f2,f3);
    }

    void getControlProximities(std::vector<BaseProximity::SPtr> & res) const override {
        res.push_back(createProximity(1,0,0,0));
        res.push_back(createProximity(0,1,0,0));
        res.push_back(createProximity(0,0,1,0));
        res.push_back(createProximity(0,0,0,1));
    }

    inline unsigned getP0() const { return m_p0; }

    inline unsigned getP1() const { return m_p1; }

    inline unsigned getP2() const { return m_p2; }

    inline unsigned getP3() const { return m_p3; }

    void draw(const core::visual::VisualParams * vparams) override {
        type::Vector3 p0 = createProximity(1,0,0,0)->getPosition();
        type::Vector3 p1 = createProximity(0,1,0,0)->getPosition();
        type::Vector3 p2 = createProximity(0,0,1,0)->getPosition();
        type::Vector3 p3 = createProximity(0,0,0,1)->getPosition();

        if (vparams->displayFlags().getShowWireFrame()) {
            glBegin(GL_LINES);
                glVertex3dv(p0.data());glVertex3dv(p1.data());
                glVertex3dv(p1.data());glVertex3dv(p2.data());
                glVertex3dv(p2.data());glVertex3dv(p0.data());
            glEnd();
        } else {
            glBegin(GL_TRIANGLES);
                glVertex3dv(p0.data());
                glVertex3dv(p1.data());
                glVertex3dv(p2.data());
            glEnd();
        }
    }

private:
    unsigned m_p0,m_p1,m_p2,m_p3;
};

}
