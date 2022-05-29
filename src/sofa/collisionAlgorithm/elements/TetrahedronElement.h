#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/collisionAlgorithm/elements/TriangleElement.h>

namespace sofa::collisionAlgorithm {

class TetrahedronElementSPtr : public std::shared_ptr<TetrahedronElement> {
public:
    friend class TetrahedronElement;

    BaseProximity::SPtr createProximity(double f0,double f1,double f2,double f3) const;

private:
    TetrahedronElementSPtr(TriangleElement::SPtr tri0, TriangleElement::SPtr tri1, TriangleElement::SPtr tri2, TriangleElement::SPtr tri3);
};

class TetrahedronElement : public BaseElement {
public:
    friend class TetrahedronElementSPtr;

    typedef TetrahedronElementSPtr SPtr;

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

    size_t getOperationsHash() const override { return typeid(TetrahedronElement).hash_code(); }

    std::string name() const override { return "TetrahedronElement"; }

    void update() override {
        m_tinfo.update(getP0()->getPosition(),
                       getP1()->getPosition(),
                       getP2()->getPosition(),
                       getP3()->getPosition());
    }

    inline const TetraInfo & getTetrahedronInfo() const { return m_tinfo; }

    inline BaseProximity::SPtr getP0() const { return this->pointElements()[0]->getP0(); }

    inline BaseProximity::SPtr getP1() const { return this->pointElements()[1]->getP0(); }

    inline BaseProximity::SPtr getP2() const { return this->pointElements()[2]->getP0(); }

    inline BaseProximity::SPtr getP3() const { return this->pointElements()[3]->getP0(); }

    void draw(const core::visual::VisualParams * vparams) override {
        type::Vector3 p0 = getP0()->getPosition();
        type::Vector3 p1 = getP1()->getPosition();
        type::Vector3 p2 = getP2()->getPosition();
        type::Vector3 p3 = getP3()->getPosition();

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

    static SPtr create(BaseProximity::SPtr p0, BaseProximity::SPtr p1,BaseProximity::SPtr p2,BaseProximity::SPtr p3);

    static SPtr create(TriangleElement::SPtr tri0, TriangleElement::SPtr tri1, TriangleElement::SPtr tri2, TriangleElement::SPtr tri3);

private:
    TetraInfo m_tinfo;
    TetrahedronElement() {}

};

}
