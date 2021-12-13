#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>
#include <sofa/collisionAlgorithm/operations/TriangleOperation.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class TriangleGeometry : public TBaseGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef TriangleGeometry<DataTypes> GEOMETRY;
    typedef TBaseGeometry<DataTypes,TriangleProximity> Inherit;
    typedef BaseProximity::Index Index;
    typedef typename Inherit::PROXIMITYDATA PROXIMITYDATA;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;

    typedef size_t TriangleID;
    typedef sofa::topology::Triangle Triangle;
    typedef sofa::type::vector<Triangle> VecTriangles;

    SOFA_CLASS(GEOMETRY,Inherit);

    core::objectmodel::SingleLink<GEOMETRY,core::topology::BaseMeshTopology,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_topology;

    TriangleGeometry()
        : l_topology(initLink("topology", "link to topology")) {
        l_topology.setPath("@.");
    }



    inline BaseElement::Iterator begin(Index eid = 0) const override {
        return toolBox::TriangleOperation::begin(this, this->l_topology->getTriangles(), eid);
    }

    virtual type::Vector3 computeNormal(const PROXIMITYDATA & data) const override {
        auto tinfo = getTriangleInfo()[data.m_eid];
        return cross(tinfo.ax2,tinfo.ax1);
    }

};


}

}
