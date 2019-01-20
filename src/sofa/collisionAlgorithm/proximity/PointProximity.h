//#pragma once

//#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>

//namespace sofa
//{

//namespace collisionAlgorithm
//{

//template<class GEOMETRY>
//class PointProximity : public TBaseProximity<GEOMETRY>
//{
//    friend class GEOMETRY::GEOMETRY;

//public :
//    typedef PointProximity<GEOMETRY> PROXIMITY;

//    typedef typename GEOMETRY::TDataTypes DataTypes;
//    typedef typename DataTypes::VecCoord VecCoord;
//    typedef typename DataTypes::Coord Coord;
//    typedef typename DataTypes::Real Real;
//    typedef typename DataTypes::VecDeriv VecDeriv;
//    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
//    typedef typename DataTypes::Deriv Deriv1;
//    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;
//    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
//    typedef core::objectmodel::Data< VecDeriv >        DataVecDeriv;
//    typedef core::objectmodel::Data< MatrixDeriv >     DataMatrixDeriv;
//    typedef sofa::core::behavior::MechanicalState<DataTypes> State;

//    PointProximity(const GEOMETRY * geometry, unsigned pid)
//    : TBaseProximity<GEOMETRY>(geometry)
//    , m_pid(pid) {}

//    defaulttype::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const {
//        return this->m_geometry->getPosition(this, v);
//    }

//    virtual defaulttype::Vector3 getNormal() const {
//        return this->m_geometry->getNormal(this);
//    }

//    void addContributions(MatrixDerivRowIterator & c_it, const defaulttype::Vector3 & N) const {
//        c_it.addCol(m_pid, N);
//    }

//    static BaseProximity::SPtr project(const GEOMETRY * geometry, unsigned pid, const defaulttype::Vector3 & /*P*/) {
//        return BaseProximity::create<PROXIMITY>(geometry,pid);
//    }

//    static BaseProximity::SPtr center(const GEOMETRY * geometry, unsigned pid) {
//        return BaseProximity::create<PROXIMITY>(geometry,pid);
//    }

//    static defaulttype::BoundingBox getBBox(const GEOMETRY * geometry, unsigned pid) {
//        const helper::ReadAccessor<DataVecCoord>& x = *geometry->l_state->read(core::VecCoordId::position());
//        defaulttype::BoundingBox bbox;
//        bbox.include(x[pid]);
//        return bbox;
//    }

//protected:
//    const unsigned m_pid;

//};

//}

//}
