#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>

namespace sofa::collisionAlgorithm {

class PointElement;

class PointProximityCreator : public ProximityCreator {
public:
    virtual BaseProximity::SPtr createProximity(const PointElement * elmt) = 0;

};

class PointElement : public BaseElement {
public:

    typedef std::shared_ptr<PointElement> SPtr;

    PointElement(PointProximityCreator * parent, unsigned eid)
    : m_point(eid)
    , m_parent(parent) {}

    unsigned id() override { return m_point; }

    inline BaseProximity::SPtr createProximity() const {
        return m_parent->createProximity(this);
    }

    inline unsigned getP0() const { return m_point; }

    void getControlProximities(std::vector<BaseProximity::SPtr> & res) const override {
        res.push_back(createProximity());
    }

    void draw(const core::visual::VisualParams * /*vparams*/) override {
        type::Vector3 p0 = m_parent->getPosition(m_point);

        glBegin(GL_POINT);
            glVertex3dv(p0.data());
        glEnd();
    }

private:
    unsigned m_point;
    PointProximityCreator * m_parent;
};


}
