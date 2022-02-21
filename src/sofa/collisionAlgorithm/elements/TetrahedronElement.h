#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>

namespace sofa::collisionAlgorithm {

class TetrahedronElement;

class TetrahedronProximityCreator : public ProximityCreator {
public:
    virtual BaseProximity::SPtr createProximity(const TetrahedronElement * elmt,double f1,double f2,double f3,double f4) = 0;

};

class TetrahedronElement : public BaseElement {
public:

    typedef std::shared_ptr<TetrahedronElement> SPtr;

    struct TetraInfo
    {
        double V0;
        type::Vec3d P0,P1,P2,P3;
        type::Vec3d ax1,ax2,ax3,ax2Cax3;
    };

    TetrahedronElement(TetrahedronProximityCreator * parent, unsigned eid, unsigned p0,unsigned p1,unsigned p2,unsigned p3)
    : m_parent(parent), m_eid(eid), m_p0(p0), m_p1(p1), m_p2(p2), m_p3(p3) {}

    unsigned id() override { return m_eid; }

    void update(const std::vector<type::Vector3> & pos) {
        m_tinfo.P0 = pos[m_p0];
        m_tinfo.P1 = pos[m_p1];
        m_tinfo.P2 = pos[m_p2];
        m_tinfo.P3 = pos[m_p3];

        m_tinfo.ax1 = m_tinfo.P1 - m_tinfo.P0;
        m_tinfo.ax2 = m_tinfo.P2 - m_tinfo.P0;
        m_tinfo.ax3 = m_tinfo.P3 - m_tinfo.P0;
        m_tinfo.ax2Cax3 = m_tinfo.ax2.cross(m_tinfo.ax3);
        m_tinfo.V0 = 1.0/6.0 * dot(m_tinfo.ax1,m_tinfo.ax2Cax3);
    }

    inline BaseProximity::SPtr createProximity(double f0,double f1,double f2, double f3) const {
        return m_parent->createProximity(this, f0,f1, f2, f3);
    }

    void getControlProximities(std::vector<BaseProximity::SPtr> & res) const override {
        res.push_back(createProximity(1,0,0,0));
        res.push_back(createProximity(0,1,0,0));
        res.push_back(createProximity(0,0,1,0));
        res.push_back(createProximity(0,0,0,1));
    }

    inline const TetraInfo & getTetrahedronInfo() const { return m_tinfo; }

    inline unsigned getP0() const { return m_p0; }

    inline unsigned getP1() const { return m_p1; }

    inline unsigned getP2() const { return m_p2; }

    inline unsigned getP3() const { return m_p3; }

    void draw(const core::visual::VisualParams * vparams) override {
        type::Vector3 p0 = m_parent->getPosition(m_p0);
        type::Vector3 p1 = m_parent->getPosition(m_p1);
        type::Vector3 p2 = m_parent->getPosition(m_p2);
        type::Vector3 p3 = m_parent->getPosition(m_p3);

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
    TetrahedronProximityCreator * m_parent;
    unsigned m_eid, m_p0,m_p1,m_p2,m_p3;
    TetraInfo m_tinfo;
};

}
