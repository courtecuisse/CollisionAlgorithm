#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/collisionAlgorithm/operations/PointOperation.h>
#include <sofa/collisionAlgorithm/operations/EdgeOperation.h>

namespace sofa::collisionAlgorithm {

//static int createPointProximity =

class TriangleOperation : public BaseOperations {
public:

    class TriangleProximity : public BaseProximity {
    public:
        TriangleProximity(BaseProximity::SPtr p0,BaseProximity::SPtr p1, BaseProximity::SPtr p2, double f0,double f1,double f2)
        : m_p0(p0), m_p1(p1), m_p2(p2), m_f0(f0), m_f1(f1), m_f2(f2) {}

        void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vector3> & dir, double fact, Index constraintId) const override {
            for (Index j=0;j<dir.size();j++) {
                std::vector<sofa::type::Vector3> N0,N1,N2;

                N0.push_back(dir[j] * m_f0);
                N1.push_back(dir[j] * m_f1);
                N2.push_back(dir[j] * m_f2);

                m_p0->buildJacobianConstraint(cId,N0,fact,constraintId + j);
                m_p1->buildJacobianConstraint(cId,N1,fact,constraintId + j);
                m_p2->buildJacobianConstraint(cId,N2,fact,constraintId + j);
            }
        }

        virtual sofa::type::Vector3 getNormal() const {
            sofa::type::Vector3 G = m_p0->getNormal() * m_f0 +
                                    m_p1->getNormal() * m_f1 +
                                    m_p2->getNormal() * m_f2;
            return G * 1.0/3.0;
        }

        /// return proximiy position in a vector3
        sofa::type::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const {
            sofa::type::Vector3 G = m_p0->getPosition(v) * m_f0 +
                                    m_p1->getPosition(v) * m_f1 +
                                    m_p2->getPosition(v) * m_f2;
            return G * 1.0/3.0;
        }

        BaseProximity::SPtr m_p0,m_p1,m_p2;
        double m_f0,m_f1,m_f2;
    };

    class TriangleElement : public BaseElement {
    public:

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
        };

        TriangleElement(BaseProximity::SPtr p0,BaseProximity::SPtr p1, BaseProximity::SPtr p2)
        : m_p0(p0), m_p1(p1), m_p2(p2) {}

        void update() override {
            m_tinfo.P0 = m_p0->getPosition();
            m_tinfo.P1 = m_p1->getPosition();
            m_tinfo.P2 = m_p2->getPosition();

            m_tinfo.v0 = m_tinfo.P1 - m_tinfo.P0;
            m_tinfo.v1 = m_tinfo.P2 - m_tinfo.P0;
            type::Vec3d N=cross(m_tinfo.v0,m_tinfo.v1);
            m_tinfo.area = N.norm()/2;
            N.normalize();

            m_tinfo.d00 = dot(m_tinfo.v0,m_tinfo.v0);
            m_tinfo.d01 = dot(m_tinfo.v0,m_tinfo.v1);
            m_tinfo.d11 = dot(m_tinfo.v1,m_tinfo.v1);

            m_tinfo.invDenom = 1.0 / (m_tinfo.d00 * m_tinfo.d11 - m_tinfo.d01 * m_tinfo.d01);

            m_tinfo.ax1 = m_tinfo.v0;
            m_tinfo.ax2 = m_tinfo.v0.cross(N);

            m_tinfo.ax1.normalize();
            m_tinfo.ax2.normalize();
        }

        const TriangleInfo & getTriangleInfo() const { return m_tinfo; }
        BaseProximity::SPtr getP0() const { return m_p0; }
        BaseProximity::SPtr getP1() const { return m_p1; }
        BaseProximity::SPtr getP2() const { return m_p2; }

    private:
        TriangleInfo m_tinfo;
        BaseProximity::SPtr m_p0,m_p1,m_p2;
    };

    static const BaseOperations * operation() {
        static TriangleOperation s_triop;
        return &s_triop;
    }

    static BaseProximity::SPtr createCenterProximity(BaseElement::SPtr elmt) {
        auto tri = toTriangleElement(elmt);
        return BaseProximity::SPtr(new TriangleProximity(tri->getP0(),tri->getP1(),tri->getP2(),1.0/3.0,1.0/3.0,1.0/3.0));
    }

    //Barycentric coordinates are computed according to
    //http://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
    static BaseProximity::SPtr project(BaseProximity::SPtr P, BaseElement::SPtr elmt) {
        auto tri = toTriangleElement(elmt);

        const TriangleElement::TriangleInfo & tinfo = tri->getTriangleInfo();
//        auto triangle = getTriangle(eid);

        double fact_u,fact_v,fact_w;
        projectOnTriangle(P->getPosition(),tinfo,fact_u,fact_v,fact_w);

        return BaseProximity::SPtr(new TriangleProximity(tri->getP0(),tri->getP1(),tri->getP2(),fact_u,fact_v,fact_w));
    }


    static void computeTriangleBaryCoords(const type::Vec3d & proj_P, const TriangleElement::TriangleInfo & tinfo, double & fact_u, double & fact_v, double & fact_w)
    {
        type::Vec3d v2 = proj_P - tinfo.P0;

        double d20 = dot(v2,tinfo.v0);
        double d21 = dot(v2,tinfo.v1);

        fact_v = (tinfo.d11 * d20 - tinfo.d01 * d21) * tinfo.invDenom;
        fact_w = (tinfo.d00 * d21 - tinfo.d01 * d20) * tinfo.invDenom;
        fact_u = 1.0 - fact_v  - fact_w;
    }

    static void projectOnTriangle(const type::Vec3d projectP, const TriangleElement::TriangleInfo & tinfo, double & fact_u, double & fact_v, double & fact_w) {
        type::Vec3d x1x2 = projectP - tinfo.P0;

        //corrdinate on the plane
        double c0 = dot(x1x2,tinfo.ax1);
        double c1 = dot(x1x2,tinfo.ax2);
        type::Vec3d proj_P = tinfo.P0 + tinfo.ax1 * c0 + tinfo.ax2 * c1;

        computeTriangleBaryCoords(proj_P, tinfo, fact_u,fact_v,fact_w);

        if (fact_u<0)
        {
            EdgeOperation::projectOnEdge(proj_P, tinfo.P1, tinfo.P2, fact_v, fact_w);
            fact_u=0;
        }
        else if (fact_v<0)
        {
            EdgeOperation::projectOnEdge(proj_P, tinfo.P0, tinfo.P2, fact_u, fact_w);
            fact_v=0;
        }
        else if (fact_w<0)
        {
            EdgeOperation::projectOnEdge(proj_P, tinfo.P0, tinfo.P1, fact_u, fact_v);
            fact_w=0;
        }
    }

protected:
    TriangleOperation() {
        BaseOperations::register_createCenterProximity(TriangleOperation::operation(),&TriangleOperation::createCenterProximity);
        BaseOperations::register_project(TriangleOperation::operation(),&TriangleOperation::project);
    }

    static const TriangleElement * toTriangleElement(BaseElement::SPtr elmt) {
        return (TriangleElement *) elmt.get();
    }

};



}

