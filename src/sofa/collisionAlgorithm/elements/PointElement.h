#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>

namespace sofa::collisionAlgorithm {

class PointElement;

//class PointProximityCreator {
//public:
//    virtual BaseProximity::SPtr createProximity(const PointElement * elmt) = 0;

//};

class PointElement : public BaseElement {
public:

    typedef std::shared_ptr<PointElement> SPtr;
    typedef std::function<BaseProximity::SPtr(const PointElement * elmt)> ProximityCreatorFunc;

    PointElement(BaseProximity::SPtr prox)
    : m_point(prox){
        f_createProximity = [=](const PointElement * elmt) -> BaseProximity::SPtr {
            return BaseProximity::create<PointProximity>(elmt->getP0());
        };

        getControlProximities().insert(prox);
    }

    size_t getOperationsHash() const override { return typeid(PointElement).hash_code(); }

    std::string name() const override { return "PointElement"; }

    inline BaseProximity::SPtr createProximity() const {
        return f_createProximity(this);
    }

    inline BaseProximity::SPtr getP0() const { return m_point; }

//    void getControlProximities(std::vector<BaseProximity::SPtr> & res) const override {
//        res.push_back(m_point);
//    }

    void draw(const core::visual::VisualParams * /*vparams*/) override {
        type::Vector3 p0 = m_point->getPosition();

        glBegin(GL_POINT);
            glVertex3dv(p0.data());
        glEnd();
    }

    void setCreateProximity(ProximityCreatorFunc f) {
        f_createProximity = f;
    }

private:
    BaseProximity::SPtr m_point;
    ProximityCreatorFunc f_createProximity;
};


}
