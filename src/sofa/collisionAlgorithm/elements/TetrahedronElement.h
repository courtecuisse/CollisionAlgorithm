#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/collisionAlgorithm/proximity/TetrahedronProximity.h>
#include <sofa/collisionAlgorithm/elements/TriangleElement.h>

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

        void update(type::Vec3d p0,type::Vec3d p1,type::Vec3d p2,type::Vec3d p3)
        {
          P0 = p0;
          P1 = p1;
          P2 = p2;
          P3 = p3;

          ax1 = P1 - P0;
          ax2 = P2 - P0;
          ax3 = P3 - P0;
          ax2Cax3 = ax2.cross(ax3);
          V0 = 1.0/6.0 * dot(ax1,ax2Cax3);
        }
    };

    TetrahedronElement(BaseProximity::SPtr p0,BaseProximity::SPtr p1,BaseProximity::SPtr p2,BaseProximity::SPtr p3)
    : m_p0(p0), m_p1(p1), m_p2(p2), m_p3(p3) {
        f_createProximity = [=](const TetrahedronElement * elmt,double f0,double f1,double f2,double f3) -> BaseProximity::SPtr {
            return BaseProximity::create<TetrahedronProximity>(elmt->getP0(),elmt->getP1(),elmt->getP2(),elmt->getP3(),
                                                               f0,f1,f2,f3);
        };

        m_triangle0 = BaseElement::create<TriangleElement>(p0,p1,p2);
        m_triangle1 = BaseElement::create<TriangleElement>(p1,p2,p3);
        m_triangle2 = BaseElement::create<TriangleElement>(p2,p3,p0);
        m_triangle3 = BaseElement::create<TriangleElement>(p3,p0,p1);
    }


   TetrahedronElement(TriangleElement::SPtr tri0, TriangleElement::SPtr tri1, TriangleElement::SPtr tri2, TriangleElement::SPtr tri3)
   : m_triangle0(tri0), m_triangle1(tri1), m_triangle2(tri2), m_triangle3(tri3) {
       f_createProximity = [=](const TetrahedronElement * elmt,double f0,double f1,double f2,double f3) -> BaseProximity::SPtr {
           return BaseProximity::create<TetrahedronProximity>(elmt->getP0(),elmt->getP1(),elmt->getP2(),elmt->getP3(),
                                                              f0,f1,f2,f3);
       };
   }

    size_t getOperationsHash() const override { return typeid(TetrahedronElement).hash_code(); }

    std::string name() const override { return "TetrahedronElement"; }

    void update(const std::vector<type::Vector3> & pos) {
        m_tinfo.update( m_p0->getPosition(),m_p1->getPosition(),m_p2->getPosition(),m_p3->getPosition());
    }



    inline BaseProximity::SPtr createProximity(double f0,double f1,double f2, double f3) const {
        return f_createProximity(this,f0,f1,f2,f3);
    }

    void getControlProximities(std::vector<BaseProximity::SPtr> & res) const override {
        res.push_back(m_p0);
        res.push_back(m_p1);
        res.push_back(m_p2);
        res.push_back(m_p3);
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
    TriangleElement::SPtr m_triangle0, m_triangle1, m_triangle2, m_triangle3;
};

}
