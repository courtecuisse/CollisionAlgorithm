#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class EdgeProximity : public TBaseProximity<DataTypes> {
public :

    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef core::objectmodel::Data< VecDeriv >        DataVecDeriv;
    typedef core::objectmodel::Data< MatrixDeriv >     DataMatrixDeriv;

    EdgeProximity(sofa::core::behavior::MechanicalState<DataTypes> * state,unsigned p0,unsigned p1,double f0,double f1)
    : TBaseProximity<DataTypes>(state)
    , m_p0(p0) , m_p1(p1)
    , m_f0(f0) , m_f1(f1){}

    inline defaulttype::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const {
        const helper::ReadAccessor<DataVecCoord> & pos = this->m_state->read(v);

        return pos[m_p0] * m_f0 +
               pos[m_p1] * m_f1;
    }

    inline defaulttype::Vector3 getNormal() const {
        const helper::ReadAccessor<DataVecCoord> & pos = this->m_state->read(core::VecCoordId::position());
        return (pos[m_p1] - pos[m_p0]).normalized();
    }

    void addContributions(MatrixDerivRowIterator & it, const defaulttype::Vector3 & N) const {
        it.addCol(m_p0, N * m_f0);
        it.addCol(m_p1, N * m_f1);
    }

protected:
    unsigned m_p0,m_p1;
    double m_f0,m_f1;

};

}

}
