#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/collisionAlgorithm/proximity/TetrahedronProximity.h>

namespace sofa::collisionAlgorithm {

class TetrahedronElement;

//class TetrahedronProximityCreator{
//public:
//    virtual BaseProximity::SPtr createProximity(const TetrahedronElement * elmt,double f1,double f2,double f3,double f4) = 0;

//};

class TetrahedronElement : public BaseElement {
public:

    typedef std::shared_ptr<TetrahedronElement> SPtr;
    typedef std::function<BaseProximity::SPtr(const TetrahedronElement * elmt,double f0,double f1,double f2,double f3)> ProximityCreatorFunc;

    struct TetraInfo
    {
        double V0;
        type::Vec3d P0,P1,P2,P3;
        type::Vec3d ax1,ax2,ax3,ax2Cax3;
    };

    TetrahedronElement(BaseProximity::SPtr p0,BaseProximity::SPtr p1,BaseProximity::SPtr p2,BaseProximity::SPtr p3)
    : m_p0(p0), m_p1(p1), m_p2(p2), m_p3(p3) {
        f_createProximity = [=](const TetrahedronElement * elmt,double f0,double f1,double f2,double f3) -> BaseProximity::SPtr {
            return BaseProximity::create<TetrahedronProximity>(elmt->getP0(),elmt->getP1(),elmt->getP2(),elmt->getP3(),
                                                               f0,f1,f2,f3);
        };
    }

    size_t getOperationsHash() const override { return typeid(TetrahedronElement).hash_code(); }

    std::string name() const override { return "TetrahedronElement"; }

    void update(const std::vector<type::Vector3> & pos) {
        m_tinfo.P0 = m_p0->getPosition();
        m_tinfo.P1 = m_p1->getPosition();
        m_tinfo.P2 = m_p2->getPosition();
        m_tinfo.P3 = m_p3->getPosition();

        m_tinfo.ax1 = m_tinfo.P1 - m_tinfo.P0;
        m_tinfo.ax2 = m_tinfo.P2 - m_tinfo.P0;
        m_tinfo.ax3 = m_tinfo.P3 - m_tinfo.P0;
        m_tinfo.ax2Cax3 = m_tinfo.ax2.cross(m_tinfo.ax3);
        m_tinfo.V0 = 1.0/6.0 * dot(m_tinfo.ax1,m_tinfo.ax2Cax3);
    }

    inline BaseProximity::SPtr createProximity(double f0,double f1,double f2, double f3) const {
        return f_createProximity(this,f0,f1,f2,f3);
    }

    void getControlProximities(std::vector<BaseProximity::SPtr> & res) const override {
        res.push_back(createProximity(1,0,0,0));
        res.push_back(createProximity(0,1,0,0));
        res.push_back(createProximity(0,0,1,0));
        res.push_back(createProximity(0,0,0,1));
    }

    inline const TetraInfo & getTetrahedronInfo() const { return m_tinfo; }

    inline BaseProximity::SPtr getP0() const { return m_p0; }

    inline BaseProximity::SPtr getP1() const { return m_p1; }

    inline BaseProximity::SPtr getP2() const { return m_p2; }

    inline BaseProximity::SPtr getP3() const { return m_p3; }

    void draw(const core::visual::VisualParams * vparams) override {
        type::Vector3 p0 = m_p0->getPosition();
        type::Vector3 p1 = m_p1->getPosition();
        type::Vector3 p2 = m_p2->getPosition();
        type::Vector3 p3 = m_p3->getPosition();

        if (vparams->displayFlags().getShowWireFrame()) {
            glBegin(GL_LINES);
                glVertex3dv(p0.data());glVertex3dv(p1.data());
                glVertex3dv(p1.data());glVertex3dv(p2.data());
                glVertex3dv(p2.data());glVertex3dv(p0.data());

                glVertex3dv(p0.data());glVertex3dv(p3.data());
                glVertex3dv(p3.data());glVertex3dv(p1.data());
                glVertex3dv(p3.data());glVertex3dv(p2.data());

            glEnd();
        } else {
            glBegin(GL_TRIANGLES);
                glVertex3dv(p0.data());glVertex3dv(p1.data());glVertex3dv(p2.data());
                glVertex3dv(p2.data());glVertex3dv(p1.data());glVertex3dv(p3.data());
                glVertex3dv(p3.data());glVertex3dv(p0.data());glVertex3dv(p2.data());
            glEnd();
        }
    }

    void setCreateProximity(ProximityCreatorFunc f) {
        f_createProximity = f;
    }

private:
    BaseProximity::SPtr m_p0,m_p1,m_p2,m_p3;
    TetraInfo m_tinfo;
    ProximityCreatorFunc f_createProximity;
};

}
