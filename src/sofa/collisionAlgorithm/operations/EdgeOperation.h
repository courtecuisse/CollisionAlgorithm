#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/collisionAlgorithm/operations/PointOperation.h>

namespace sofa::collisionAlgorithm {

class EdgeOperation : public BaseOperations {
public:

    class EdgeProximity : public BaseProximity {
    public:

        EdgeProximity(BaseProximity::SPtr p0,BaseProximity::SPtr p1, double f0,double f1)
        : m_p0(p0), m_p1(p1), m_f0(f0), m_f1(f1) {}

        void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vector3> & dir, double fact, Index constraintId) const override {
            for (Index j=0;j<dir.size();j++) {
                std::vector<sofa::type::Vector3> N0,N1;

                N0.push_back(dir[j] * m_f0);
                N1.push_back(dir[j] * m_f1);

                m_p0->buildJacobianConstraint(cId,N0,fact,constraintId + j);
                m_p1->buildJacobianConstraint(cId,N1,fact,constraintId + j);
            }
        }

        virtual sofa::type::Vector3 getNormal() const {
            sofa::type::Vector3 G = m_p0->getNormal() * m_f0 +
                                    m_p1->getNormal() * m_f1;
            return G * 1.0/2.0;
        }

        /// return proximiy position in a vector3
        sofa::type::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const {
            sofa::type::Vector3 G = m_p0->getPosition(v) * m_f0 +
                                    m_p1->getPosition(v) * m_f1;
            return G * 1.0/2.0;
        }

        BaseProximity::SPtr m_p0,m_p1;
        double m_f0,m_f1;
    };

    class EdgeElement : public BaseElement {
    public:

        EdgeElement(BaseProximity::SPtr p0,BaseProximity::SPtr p1)
        : m_p0(p0), m_p1(p1) {}

        void update() override {}

        BaseProximity::SPtr getP0() const { return m_p0; }
        BaseProximity::SPtr getP1() const { return m_p1; }

    private:
        BaseProximity::SPtr m_p0,m_p1;
    };

    template<class DataTypes>
    static BaseElement::SPtr createElement(sofa::core::behavior::MechanicalState<DataTypes> * state, core::topology::BaseMeshTopology::Edge edge) {
        auto p0 = PointOperation::createProximity(state,edge[0]);
        auto p1 = PointOperation::createProximity(state,edge[1]);

        return BaseElement::SPtr(new EdgeElement(p0,p1));
    }


    static const BaseOperations * operation() {
        static EdgeOperation s_edgeOp;
        return &s_edgeOp;
    }

    static BaseProximity::SPtr createCenterProximity(BaseElement::SPtr elmt) {
        auto edge = toEdgeElement(elmt);
        return BaseProximity::SPtr(new EdgeProximity(edge->getP0(),edge->getP1(),0.5,0.5));
    }

    static BaseProximity::SPtr project(const BaseProximity::SPtr & P, BaseElement::SPtr elmt) {
        auto edge = toEdgeElement(elmt);

        double fact_u,fact_v;
        projectOnEdge(P->getPosition(),edge->getP0()->getPosition(),edge->getP1()->getPosition(),fact_u,fact_v);

        return BaseProximity::SPtr(new EdgeProximity(edge->getP0(),edge->getP1(),fact_u,fact_v));
    }

    static void projectOnEdge(const type::Vec3d & projP, const type::Vec3d & e1, const type::Vec3d & e2, double & fact_u, double & fact_v) {
        type::Vec3d v = e2 - e1;
        fact_v = dot(projP - e1,v) / dot(v,v);

        if (fact_v<0.0) fact_v = 0.0;
        else if (fact_v>1.0) fact_v = 1.0;

        fact_u = 1.0-fact_v;
    }


protected:
    EdgeOperation() {
        BaseOperations::register_createCenterProximity(EdgeOperation::operation(),&EdgeOperation::createCenterProximity);
    }

    static const EdgeElement * toEdgeElement(BaseElement::SPtr elmt) {
        return (EdgeElement *) elmt.get();
    }


};



}

