#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class GenericProximity : public TBaseProximity<DataTypes> {
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

    GenericProximity(sofa::core::behavior::MechanicalState<DataTypes> * state)
    : TBaseProximity<DataTypes>(state) {}

    inline defaulttype::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const {
    }

    inline defaulttype::Vector3 getNormal() const {
    }

    void addContributions(MatrixDerivRowIterator & it, const defaulttype::Vector3 & N) const {
    }

protected:

};

}

}
