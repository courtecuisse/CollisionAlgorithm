#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/gl/gl.h>

namespace sofa::collisionAlgorithm {

class PointElementSPtr : public std::shared_ptr<PointElement> {
public:
    friend class PointElement;

    BaseProximity::SPtr createProximity() const;

private:
    PointElementSPtr(BaseProximity::SPtr prox);
};

class PointElement : public BaseElement {
public:

    friend class PointElementSPtr;

    typedef PointElementSPtr SPtr;

    size_t getOperationsHash() const override { return typeid(PointElement).hash_code(); }

    std::string name() const override { return "PointElement"; }

    inline BaseProximity::SPtr getP0() const { return m_point; }

    void update() override {}

    void draw(const core::visual::VisualParams * /*vparams*/) override {
        type::Vector3 p0 = m_point->getPosition();

        glBegin(GL_POINT);
            glVertex3dv(p0.data());
        glEnd();
    }

    ElementContainer<TriangleElementSPtr > & triangleAround() { return m_triangleAround; }

    static SPtr create(BaseProximity::SPtr prox);

private:
    BaseProximity::SPtr m_point;
    ElementContainer<TriangleElementSPtr > m_triangleAround;

    PointElement(BaseProximity::SPtr prox) : m_point(prox) {}

};


}
