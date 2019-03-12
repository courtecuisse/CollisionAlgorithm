#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseElementContainer.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class GEOMETRY>
class DataPointContainer : public DataElemntContainer<sofa::defaulttype::Vector3> {
public:

    typedef typename GEOMETRY::TDataTypes DataTypes;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef core::objectmodel::Data< VecDeriv >        DataVecDeriv;
    typedef core::objectmodel::Data< MatrixDeriv >     DataMatrixDeriv;
    typedef sofa::core::behavior::MechanicalState<DataTypes> State;
    typedef sofa::core::topology::BaseMeshTopology::Triangle Triangle;
    typedef size_t TriangleID; // to remove once TriangleID has been changed to size_t in BaseMeshTopology
    typedef helper::vector<Triangle> VecTriangles;

    explicit DataPointContainer(const typename DataElemntContainer<sofa::defaulttype::Vector3>::InitData& init)
    : DataElemntContainer<sofa::defaulttype::Vector3>(init) {
        m_geometry = dynamic_cast<const GEOMETRY*>(init.owner);
    }

    virtual BaseElementIterator::UPtr begin(unsigned eid = 0) const {
        return DefaultElementIterator<DataPointContainer<GEOMETRY> >::create(this, this->getValue().size(), eid);
    }

    virtual const BaseGeometry * end() const {
        return m_geometry;
    }

    inline BaseProximity::SPtr project(unsigned pid, const defaulttype::Vector3 & ) const {
        return BaseProximity::create<PointProximity<DataTypes> >(m_geometry->getState(),pid);
    }

    inline BaseProximity::SPtr center(unsigned pid) const {
        return BaseProximity::create<PointProximity<DataTypes> >(m_geometry->getState(),pid);
    }

    inline defaulttype::BoundingBox getBBox(unsigned pid) const {
        const helper::ReadAccessor<Data <VecCoord> >& x = m_geometry->getState()->read(core::VecCoordId::position());
        defaulttype::BoundingBox bbox;
        bbox.include(x[pid]);
        return bbox;
    }

protected:
    const GEOMETRY * m_geometry;
};


}

}
