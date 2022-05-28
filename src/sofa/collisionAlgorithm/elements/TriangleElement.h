#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>

namespace sofa::collisionAlgorithm {

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

    TriangleElement(BaseProximity::SPtr p0, BaseProximity::SPtr p1,BaseProximity::SPtr p2) {
        m_sptr = new SPtr(this);

        insertElement<PointElement>(new PointElement(p0));
        insertElement<PointElement>(new PointElement(p1));
        insertElement<PointElement>(new PointElement(p2));

        insertElement<EdgeElement>(new EdgeElement(p0,p1));
        insertElement<EdgeElement>(new EdgeElement(p1,p2));
        insertElement<EdgeElement>(new EdgeElement(p2,p0));

        insertElement<TriangleElement>(this);
    }

    TriangleElement(EdgeElement::SPtr edge0, EdgeElement::SPtr edge1, EdgeElement::SPtr edge2) {
        m_sptr = new SPtr(this);

        insertElement<PointElement>(edge0->pointElements()[0]);
        insertElement<PointElement>(edge0->pointElements()[1]);
        insertElement<PointElement>(edge1->pointElements()[0]);
        insertElement<PointElement>(edge1->pointElements()[1]);

        insertElement<EdgeElement>(edge0);
        insertElement<EdgeElement>(edge1);
        insertElement<EdgeElement>(edge2);

        insertElement<TriangleElement>(this);
    }

    inline SPtr sptr() const { return *((SPtr*) m_sptr); }

    size_t getOperationsHash() const override { return typeid(TriangleElement).hash_code(); }

    std::string name() const override { return "TriangleElement"; }

    void update() override {
        m_tinfo.update(getP0()->getPosition(),
                       getP1()->getPosition(),
                       getP2()->getPosition());
    }

    BaseProximity::SPtr createProximity(double f0,double f1,double f2) const;

    inline const TriangleInfo & getTriangleInfo() const { return m_tinfo; }

    inline BaseProximity::SPtr getP0() const { return this->pointElements()[0]->getP0(); }

    inline BaseProximity::SPtr getP1() const { return this->pointElements()[1]->getP0(); }

    inline BaseProximity::SPtr getP2() const { return this->pointElements()[2]->getP0(); }

    void draw(const core::visual::VisualParams * vparams) override {
        type::Vector3 p0 = getP0()->getPosition();
        type::Vector3 p1 = getP1()->getPosition();
        type::Vector3 p2 = getP2()->getPosition();

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
    TriangleInfo m_tinfo;
    SPtr * m_sptr;
};

}
