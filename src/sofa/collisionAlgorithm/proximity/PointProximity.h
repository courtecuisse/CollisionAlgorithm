#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
class PointProximity : public TBaseProximity<DataTypes> {
public :
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
    typedef sofa::core::behavior::MechanicalState<DataTypes> State;

    PointProximity(State * state, unsigned pid)
    : TBaseProximity<DataTypes>(state)
    , m_pid(pid) {}

    defaulttype::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const {
        const helper::ReadAccessor<DataVecCoord> & pos = this->m_state->read(v);

        return pos[m_pid];
    }

    virtual defaulttype::Vector3 getNormal() const {
        return defaulttype::Vector3();
    }

    void addContributions(MatrixDerivRowIterator & c_it, const defaulttype::Vector3 & N) const {
        c_it.addCol(m_pid, N);
    }

protected:
    const unsigned m_pid;

};

}

}
