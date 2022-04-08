#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>

namespace sofa::collisionAlgorithm {

class PointElement;

class PointProximityCreator {
public:
    virtual BaseProximity::SPtr createProximity(const PointElement * elmt) = 0;

};

class PointElement : public BaseElement {
public:

    typedef std::shared_ptr<PointElement> SPtr;

    PointElement(PointProximityCreator * parent, BaseProximity::SPtr prox)
    : m_point(prox)
    , m_parent(parent) {}

    size_t getOperationsHash() const override { return typeid(PointElement).hash_code(); }

    std::string name() const override { return "PointElement"; }

    inline BaseProximity::SPtr createProximity() const {
        return m_parent->createProximity(this);
    }

    inline BaseProximity::SPtr getP0() const { return m_point; }

    void getControlProximities(std::vector<BaseProximity::SPtr> & res) const override {
        res.push_back(createProximity());
    }

    void draw(const core::visual::VisualParams * /*vparams*/) override {
        type::Vector3 p0 = m_point->getPosition();

        glBegin(GL_POINT);
            glVertex3dv(p0.data());
        glEnd();
    }

private:
    BaseProximity::SPtr m_point;
    PointProximityCreator * m_parent;
};


}
