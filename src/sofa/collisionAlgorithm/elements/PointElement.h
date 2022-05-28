#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/gl/gl.h>

namespace sofa::collisionAlgorithm {

class PointElement : public BaseElement {
public:

    typedef std::shared_ptr<PointElement> SPtr;

    PointElement(BaseProximity::SPtr prox)
    : m_point(prox) {
        m_sptr = new SPtr(this);

        this->insertElement<PointElement>(this);
    }

    inline SPtr sptr() const { return *m_sptr; }

    size_t getOperationsHash() const override { return typeid(PointElement).hash_code(); }

    std::string name() const override { return "PointElement"; }

    inline BaseProximity::SPtr createProximity() const { return m_point; }

    inline BaseProximity::SPtr getP0() const { return m_point; }

    void update() override {}

    void draw(const core::visual::VisualParams * /*vparams*/) override {
        type::Vector3 p0 = m_point->getPosition();

        glBegin(GL_POINT);
            glVertex3dv(p0.data());
        glEnd();
    }

private:
    BaseProximity::SPtr m_point;
    SPtr * m_sptr;
};


}
