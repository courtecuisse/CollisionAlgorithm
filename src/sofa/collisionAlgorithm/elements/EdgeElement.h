#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/collisionAlgorithm/proximity/EdgeProximity.h>
#include <sofa/collisionAlgorithm/elements/PointElement.h>
#include <math.h>

namespace sofa::collisionAlgorithm {

class EdgeElement;

//class EdgeProximityCreator {
//public:
//    virtual BaseProximity::SPtr createProximity(const EdgeElement * elmt,double f0,double f1) = 0;

//};

class EdgeElement : public BaseElement {
public:

    typedef std::shared_ptr<EdgeElement> SPtr;
    typedef std::function<BaseProximity::SPtr(const EdgeElement * elmt,double f0,double f1)> ProximityCreatorFunc;

    EdgeElement(BaseProximity::SPtr p0,BaseProximity::SPtr p1)
    : m_p0(p0), m_p1(p1) {
        f_createProximity = [=](const EdgeElement * elmt,double f0,double f1) -> BaseProximity::SPtr {
            return BaseProximity::create<EdgeProximity>(elmt->getP0(),elmt->getP1(),f0,f1);
        };

        m_point0 = BaseElement::create<PointElement>(p0);
        m_point1 = BaseElement::create<PointElement>(p1);

        getControlProximities().insert(p0);
        getControlProximities().insert(p1);
    }

    EdgeElement(PointElement::SPtr point0, PointElement::SPtr point1, BaseProximity::SPtr p0, BaseProximity::SPtr p1)
    : m_point0(point0), m_point1(point1), m_p0(p0), m_p1(p1) {
        f_createProximity = [=](const EdgeElement * elmt,double f0,double f1) -> BaseProximity::SPtr {
            return BaseProximity::create<EdgeProximity>(elmt->getP0(),elmt->getP1(),f0,f1);
        };

//        getControlProximities().insert(m_point0->getControlProximities());
//        getControlProximities().insert(m_point1->getControlProximities());

        getControlProximities().insert(p0);
        getControlProximities().insert(p1);

//        m_p0 = getControlProximities().getProximities()[0];
//        m_p1 = getControlProximities().getProximities()[1];
    }

    inline BaseProximity::SPtr createProximity(double f0,double f1) const {
        return f_createProximity(this,f0,f1);
    }

    size_t getOperationsHash() const override { return typeid(EdgeElement).hash_code(); }

    std::string name() const override { return "EdgeElement"; }

    inline BaseProximity::SPtr getP0() const { return m_p0; }

    inline BaseProximity::SPtr getP1() const { return m_p1; }

    inline PointElement::SPtr getPoint0() const { return m_point0; }

    inline PointElement::SPtr getPoint1() const { return m_point1; }

    void getSubElements(std::set<BaseElement::SPtr> & subElem) const override {
        subElem.insert(m_point0);
        subElem.insert(m_point1);
    }

//    void getControlProximities(std::vector<BaseProximity::SPtr> & res) const override {
//        res.push_back(m_p0);
//        res.push_back(m_p1);
//    }

    void draw(const core::visual::VisualParams * /*vparams*/) override {
        type::Vector3 p0 = m_p0->getPosition();
        type::Vector3 p1 = m_p1->getPosition();

        glBegin(GL_LINES);
            glVertex3dv(p0.data());glVertex3dv(p1.data());
        glEnd();
    }

    void setCreateProximity(ProximityCreatorFunc f) {
        f_createProximity = f;
    }



//    //////////////////////

//    BaseElement::SPtr intersection_edge_edge(EdgeElement* e) {
//        auto a1 = this->getP0();
//        auto b1 = this->getP1();
//        auto a2 = e->getP0();
//        auto b2 = e->getP1();

//        type::Vec3 B = a2 - a1;
// //        std::vector<type::Vec3> A = {-a1+b1 , a2-b2};
// //        type::Mat<3,2,double> A(-a1+b1 , a2-b2);

// //        return //// ;
//        }

//    //////////////////////


private:
    BaseProximity::SPtr m_p0,m_p1;
    ProximityCreatorFunc f_createProximity;
    PointElement::SPtr m_point0, m_point1;
};

}
