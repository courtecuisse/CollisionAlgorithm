#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>
#include <math.h>

namespace sofa::collisionAlgorithm {

class EdgeElement;

class EdgeProximityCreator : public ProximityCreator {
public:
    virtual BaseProximity::SPtr createProximity(const EdgeElement * elmt,double f0,double f1) = 0;

};

class EdgeElement : public BaseElement {
public:

    typedef std::shared_ptr<EdgeElement> SPtr;

    EdgeElement(EdgeProximityCreator * parent, unsigned eid, unsigned p0,unsigned p1)
    : m_parent(parent), m_eid(eid), m_p0(p0), m_p1(p1) {}

//    unsigned id() override { return m_eid; }

    inline BaseProximity::SPtr createProximity(double f0,double f1) const {
        return m_parent->createProximity(this,f0,f1);
    }

    size_t getOperationsHash() const override { return typeid(EdgeElement).hash_code(); }

    std::string name() const override { return "EdgeElement"; }

    inline unsigned getP0() const { return m_p0; }

    inline unsigned getP1() const { return m_p1; }

    void getControlProximities(std::vector<BaseProximity::SPtr> & res) const override {
        res.push_back(createProximity(1,0));
        res.push_back(createProximity(0,1));
    }

    void draw(const core::visual::VisualParams * /*vparams*/) override {
        type::Vector3 p0 = m_parent->getPosition(m_p0);
        type::Vector3 p1 = m_parent->getPosition(m_p1);

        glBegin(GL_LINES);
            glVertex3dv(p0.data());glVertex3dv(p1.data());
        glEnd();
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
    EdgeProximityCreator * m_parent;
    unsigned m_eid, m_p0,m_p1;
};

}
