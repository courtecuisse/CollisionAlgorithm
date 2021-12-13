#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/BaseOperation.h>

namespace sofa::collisionAlgorithm {

//static int createPointProximity =

class PointOperation : public BaseOperations {
public:

    template<class DataTypes>
    class PointProximity : public BaseProximity {
    public:
        typedef sofa::core::behavior::MechanicalState<DataTypes> State;
        typedef typename DataTypes::VecCoord VecCoord;
        typedef typename DataTypes::Coord Coord;
        typedef typename DataTypes::Real Real;
        typedef typename DataTypes::VecDeriv VecDeriv;
        typedef typename DataTypes::MatrixDeriv MatrixDeriv;
        typedef typename DataTypes::Deriv Deriv1;
        typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;
        typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
        typedef core::objectmodel::Data< VecDeriv >        DataVecDeriv;
        typedef core::objectmodel::Data< MatrixDeriv >     DataMatrixDeriv;

        PointProximity(State * s, unsigned pid)
        : m_state(s), m_pid(pid) {}

        void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vector3> & dir, double fact, Index constraintId) const override {
            DataMatrixDeriv & c1_d = *cId[this->getState()].write();
            MatrixDeriv & c1 = *c1_d.beginEdit();

            for (Index j=0;j<dir.size();j++) {
                MatrixDerivRowIterator c_it = c1.writeLine(constraintId+j);
                c_it.addCol(m_pid, dir[j] * fact);
            }

            c1_d.endEdit();
        }

        virtual sofa::type::Vector3 getNormal() const {

        }

        /// return proximiy position in a vector3
        sofa::type::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const {
            const helper::ReadAccessor<DataVecCoord> & pos = m_state->read(v);
            return pos[m_pid];
        }


    private:
        State * m_state;
        unsigned m_pid;
    };

    class PointElement : public BaseElement {
    public:
        PointElement(BaseProximity::SPtr p)
        : m_point(p) {}

        void update() override {}

        BaseProximity::SPtr getProximity() const { return m_point; }

    private:
        BaseProximity::SPtr m_point;
    };

    template<class DataTypes>
    static BaseProximity::SPtr createProximity(sofa::core::behavior::MechanicalState<DataTypes> * state, unsigned pid) {
        return BaseProximity::SPtr(new PointProximity(state,pid));
    }

    static const BaseOperations * operation() {
        static PointOperation s_pointOp;
        return &s_pointOp;
    }

    static BaseProximity::SPtr createCenterProximity(BaseElement::SPtr elmt) {
        auto point = toPointElement(elmt);
        return point->getProximity();
    }


    static BaseProximity::SPtr project(const BaseProximity::SPtr & /*P*/, BaseElement::SPtr elmt) {
        auto point = toPointElement(elmt);
        return point->getProximity();
    }

protected:
    PointOperation() {
        BaseOperations::register_createCenterProximity(PointOperation::operation(),&PointOperation::createCenterProximity);
    }

    static const PointElement * toPointElement(BaseElement::SPtr elmt) {
        return (PointElement *) elmt.get();
    }
};



}

