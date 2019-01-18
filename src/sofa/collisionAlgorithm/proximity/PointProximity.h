#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
class PointProximity : public TBaseProximity<DataTypes>
{
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

    PointProximity(unsigned pid, State * state)
    : TBaseProximity<DataTypes>(state)
    , m_pid(pid) {}

    defaulttype::Vector3 getPosition(core::VecCoordId v) const
    {
        const helper::ReadAccessor<DataVecCoord> & pos = this->m_state->read(v);
        return pos[m_pid];
    }

    virtual defaulttype::Vector3 getNormal() const
    {
        return defaulttype::Vector3(1,0,0);
    }

    void addContributions(MatrixDerivRowIterator & c_it, const defaulttype::Vector3 & N) const {
        c_it.addCol(m_pid, N);
    }

    static BaseProximity::SPtr project(const PointGeometry<DataTypes>* geometry, unsigned pid, const defaulttype::Vector3 & /*P*/) {
        return BaseProximity::create<PointProximity<DataTypes>>(pid,geometry->l_state.get());
    }

    static BaseProximity::SPtr center(const PointGeometry<DataTypes>* geometry, unsigned pid) {
        return BaseProximity::create<PointProximity<DataTypes>>(pid,geometry->l_state.get());
    }

    static defaulttype::BoundingBox getBBox(const PointGeometry<DataTypes>* geometry, unsigned pid) {
        const helper::ReadAccessor<DataVecCoord>& x = *geometry->l_state->read(core::VecCoordId::position());
        defaulttype::BoundingBox bbox;
        bbox.include(x[pid]);
        return bbox;
    }

protected:
    const unsigned m_pid;

};

}

}
