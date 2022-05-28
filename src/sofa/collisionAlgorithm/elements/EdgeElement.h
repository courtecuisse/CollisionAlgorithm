#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/collisionAlgorithm/elements/PointElement.h>
#include <math.h>

namespace sofa::collisionAlgorithm {

class EdgeElement : public BaseElement {
public:

    typedef std::shared_ptr<EdgeElement> SPtr;

    EdgeElement(BaseProximity::SPtr p0,BaseProximity::SPtr p1) {
        m_sptr = new SPtr(this);

        insertElement<PointElement>(new PointElement(p0));
        insertElement<PointElement>(new PointElement(p1));
        insertElement<EdgeElement>(this);
    }

    EdgeElement(PointElement::SPtr point0, PointElement::SPtr point1) {
        m_sptr = new SPtr(this);

        insertElement<PointElement>(point0);
        insertElement<PointElement>(point1);
        insertElement<EdgeElement>(this);
    }

    inline SPtr sptr() const { return *m_sptr; }

    BaseProximity::SPtr createProximity(double f0,double f1) const;

    inline BaseProximity::SPtr getP0() const { return pointElements()[0]->getP0(); }

    inline BaseProximity::SPtr getP1() const { return pointElements()[1]->getP0(); }

    size_t getOperationsHash() const override { return typeid(EdgeElement).hash_code(); }

    std::string name() const override { return "EdgeElement"; }

    void update() override {}

    void draw(const core::visual::VisualParams * /*vparams*/) override {

        type::Vector3 p0 = getP0()->getPosition();
        type::Vector3 p1 = getP1()->getPosition();

        glBegin(GL_LINES);
            glVertex3dv(p0.data());
            glVertex3dv(p1.data());
        glEnd();
    }

private:
    SPtr * m_sptr;
};

}
