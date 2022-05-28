#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/collisionAlgorithm/elements/PointElement.h>

namespace sofa::collisionAlgorithm {

class EdgeElementSPtr : public std::shared_ptr<EdgeElement> {
public:

    EdgeElementSPtr(BaseProximity::SPtr p0,BaseProximity::SPtr p1);

    EdgeElementSPtr(PointElement::SPtr point0, PointElement::SPtr point1);

    BaseProximity::SPtr createProximity(double f0,double f1) const;
};

class EdgeElement : public BaseElement {
public:
    friend class EdgeElementSPtr;

    typedef EdgeElementSPtr SPtr;

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
    EdgeElement() {}
};

}
