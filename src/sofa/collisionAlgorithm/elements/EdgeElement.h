#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/collisionAlgorithm/elements/PointElement.h>

namespace sofa::collisionAlgorithm {

class EdgeElement : public BaseElement {
public:
    typedef std::shared_ptr<EdgeElement> SPtr;

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

    static SPtr create(PointElement::SPtr point0, PointElement::SPtr point1);

    static SPtr create(BaseProximity::SPtr p0,BaseProximity::SPtr p1);

    const ElementContainer<PointElement> & pointElements() const override { return m_pointElements; }

    const ElementContainer<EdgeElement> & edgeElements() const override { return ElementContainer<EdgeElement>::empty(); }

    const ElementContainer<TriangleElement> & triangleElements() const override { return ElementContainer<TriangleElement>::empty(); }

    const ElementContainer<TetrahedronElement> & tetrahedronElements() const override { return ElementContainer<TetrahedronElement>::empty(); }

private:
    EdgeElement() {}

    ElementContainer<PointElement> m_pointElements;
//    ElementContainer<EdgeElement> m_edgeElements;
};

}
