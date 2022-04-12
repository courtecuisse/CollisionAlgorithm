#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>

namespace sofa::collisionAlgorithm {

class TriangleElement;

class TriangleProximityCreator{
public:
    virtual BaseProximity::SPtr createProximity(const TriangleElement * elmt,double f0,double f1,double f2) = 0;

};

class TriangleElement : public BaseElement {
public:

    typedef std::shared_ptr<TriangleElement> SPtr;

    struct TriangleInfo
    {
        type::Vec3d v0,v1;
        double d00;
        double d01;
        double d11;
        double invDenom;
        double area;

        type::Vec3d ax1,ax2;
        type::Vec3d P0,P1,P2;
        type::Vec3d N;

        void update(const type::Vec3d & e0, const type::Vec3d & e1,const type::Vec3d & e2) {
            P0 = e0;
            P1 = e1;
            P2 = e2;

            v0 = P1 - P0;
            v1 = P2 - P0;
            N=cross(v0,v1);
            area = N.norm()/2;
            N.normalize();

            d00 = dot(v0,v0);
            d01 = dot(v0,v1);
            d11 = dot(v1,v1);

            invDenom = 1.0 / (d00 * d11 - d01 * d01);

            ax1 = v0;
            ax2 = v0.cross(N);

            ax1.normalize();
            ax2.normalize();
        }
    };

    TriangleElement(TriangleProximityCreator * parent, BaseProximity::SPtr p0, BaseProximity::SPtr p1,BaseProximity::SPtr p2)
    : m_parent(parent), m_p0(p0), m_p1(p1), m_p2(p2) {}

    size_t getOperationsHash() const override { return typeid(TriangleElement).hash_code(); }

    std::string name() const override { return "TriangleElement"; }

    void update(const std::vector<type::Vector3> & pos) {
        m_tinfo.update(m_p0->getPosition(), m_p1->getPosition(), m_p2->getPosition());
    }

    inline BaseProximity::SPtr createProximity(double f0,double f1,double f2) const {
        return m_parent->createProximity(this, f0,f1, f2);
    }

    inline const TriangleInfo & getTriangleInfo() const { return m_tinfo; }

    inline BaseProximity::SPtr getP0() const { return m_p0; }

    inline BaseProximity::SPtr getP1() const { return m_p1; }

    inline BaseProximity::SPtr getP2() const { return m_p2; }

    void getControlProximities(std::vector<BaseProximity::SPtr> & res) const override {
        res.push_back(createProximity(1,0,0));
        res.push_back(createProximity(0,1,0));
        res.push_back(createProximity(0,0,1));
    }


    void draw(const core::visual::VisualParams * vparams) override {
        type::Vector3 p0 = m_p0->getPosition();
        type::Vector3 p1 = m_p1->getPosition();
        type::Vector3 p2 = m_p2->getPosition();

        if (vparams->displayFlags().getShowWireFrame()) {
            glBegin(GL_LINES);
                glVertex3dv(p0.data());glVertex3dv(p1.data());
                glVertex3dv(p1.data());glVertex3dv(p2.data());
                glVertex3dv(p2.data());glVertex3dv(p0.data());
            glEnd();
        } else {
            glBegin(GL_TRIANGLES);
                glVertex3dv(p0.data());
                glVertex3dv(p1.data());
                glVertex3dv(p2.data());
            glEnd();
        }
    }

private:
    TriangleProximityCreator * m_parent;
    BaseProximity::SPtr m_p0,m_p1,m_p2;
    TriangleInfo m_tinfo;
};

}
