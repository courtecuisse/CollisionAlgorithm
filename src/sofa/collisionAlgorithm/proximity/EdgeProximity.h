#pragma once

#include <sofa/collisionAlgorithm/geometry/EdgeGeometry.h>
#include <sofa/collisionAlgorithm/element/EdgeElement.h>

namespace sofa
{

namespace collisionAlgorithm
{

class EdgeProximity : public ConstraintProximity
{
public :
    typedef sofa::defaulttype::Vec3dTypes DataTypes;
    typedef DataTypes::VecCoord VecCoord;
    typedef DataTypes::Coord Coord;
    typedef DataTypes::Real Real;
    typedef DataTypes::VecDeriv VecDeriv;
    typedef DataTypes::MatrixDeriv MatrixDeriv;
    typedef DataTypes::Deriv Deriv1;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef core::objectmodel::Data< VecDeriv >        DataVecDeriv;
    typedef core::objectmodel::Data< MatrixDeriv >     DataMatrixDeriv;
    typedef MatrixDeriv::RowIterator MatrixDerivRowIterator;

    EdgeProximity(const EdgeElement * elmt,double f1,double f2)
    : m_element(elmt)
    , m_state(elmt->geometry()->getState()) {
        m_fact[0] = f1;
        m_fact[1] = f2;
    }

    inline const EdgeElement* element() const
    {
        return  m_element;
    }

    defaulttype::Vector3 getPosition(core::VecCoordId v) const
    {
        const helper::ReadAccessor<DataVecCoord> pos = m_state->read(v);
        return pos[element()->m_pid[0]] * m_fact[0] + pos[element()->m_pid[1]] * m_fact[1];
    }

    defaulttype::Vector3 getNormal() const
    {
        return defaulttype::Vector3(1,0,0);
    }

    void buildJacobianConstraint(core::MultiMatrixDerivId cId, ConstraintNormal & normals, double fact, unsigned constraintId) const {
        DataMatrixDeriv & c1_d = *cId[m_state].write();
        MatrixDeriv & c1 = *c1_d.beginEdit();

        for (unsigned j=0;j<normals.size();j++) {
            MatrixDerivRowIterator c_it = c1.writeLine(constraintId+j);

            c_it.addCol(element()->m_pid[0], normals[j] * m_fact[0] * fact);
            c_it.addCol(element()->m_pid[1], normals[j] * m_fact[1] * fact);
        }

        c1_d.endEdit();
    }

protected:
    double m_fact[2];
    const EdgeElement* m_element;
    sofa::core::behavior::MechanicalState<defaulttype::Vec3dTypes> * m_state;
};

}

}
