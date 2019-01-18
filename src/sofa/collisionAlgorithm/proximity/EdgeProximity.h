#pragma once

#include <sofa/collisionAlgorithm/geometry/EdgeGeometry.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
class EdgeProximity : public TBaseProximity<DataTypes>
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

    EdgeProximity(unsigned p1,unsigned p2,double f1,double f2, State * state)
    : TBaseProximity<DataTypes>(state){
        m_pid[0] = p1;
        m_pid[1] = p2;
        m_fact[0] = f1;
        m_fact[1] = f2;
    }

    defaulttype::Vector3 getPosition(core::VecCoordId v) const
    {
        const helper::ReadAccessor<DataVecCoord> pos = this->m_state->read(v);
        return pos[m_pid[0]] * m_fact[0] + pos[m_pid[1]] * m_fact[1];
    }

    defaulttype::Vector3 getNormal() const
    {
        return defaulttype::Vector3(1,0,0);
    }

    virtual void addContributions(MatrixDerivRowIterator & it, const defaulttype::Vector3 & N) const {
        it.addCol(m_pid[0], N * m_fact[0]);
        it.addCol(m_pid[1], N * m_fact[1]);
    }

    static BaseProximity::SPtr project(const EdgeGeometry<DataTypes>* geometry, unsigned eid, const defaulttype::Vector3 & P) {
        const core::topology::BaseMeshTopology::Edge edge = geometry->d_edges.getValue()[eid];

        const helper::ReadAccessor<Data <VecCoord> >& x = *geometry->l_state->read(core::VecCoordId::position());

        double fact_u;
        double fact_v;

        Coord v = x[edge[1]] - x[edge[0]];
        fact_v = dot (P - x[edge[0]],v) / dot (v,v);

        if (fact_v<0.0) fact_v = 0.0;
        else if (fact_v>1.0) fact_v = 1.0;

        fact_u = 1.0-fact_v;

        return BaseProximity::create<EdgeProximity<DataTypes>>(edge[0],edge[1],fact_u,fact_v,geometry->l_state.get());
    }

    static BaseProximity::SPtr center(const EdgeGeometry<DataTypes>* geometry, unsigned eid) {
        const core::topology::BaseMeshTopology::Edge edge = geometry->d_edges.getValue()[eid];
        return BaseProximity::create<EdgeProximity<DataTypes>>(edge[0],edge[1],0.5,0.5,geometry->l_state.get());
    }

    static defaulttype::BoundingBox getBBox(const EdgeGeometry<DataTypes>* geometry, unsigned eid) {
        const core::topology::BaseMeshTopology::Edge edge = geometry->d_edges.getValue()[eid];
        const helper::ReadAccessor<DataVecCoord >& x = *geometry->l_state->read(core::VecCoordId::position());
        defaulttype::BoundingBox bbox;
        bbox.include(x[edge[0]]);
        bbox.include(x[edge[1]]);
        return bbox;
    }

protected:
    double m_pid[2];
    double m_fact[2];
};

}

}
