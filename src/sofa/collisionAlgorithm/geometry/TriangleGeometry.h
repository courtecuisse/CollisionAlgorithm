#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>
#include <sofa/collisionAlgorithm/operations/TriangleOperation.h>
#include <sofa/collisionAlgorithm/elements/TriangleElement.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class TriangleGeometry : public TBaseGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef TriangleGeometry<DataTypes> GEOMETRY;
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef BaseProximity::Index Index;
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

    class PointTriangleNormalHandler : public PointOperation::NormalHandler {
    public:

        PointTriangleNormalHandler(TriangleOperation::TriangleInfo & t) : m_tinfo(t) {}

        virtual sofa::type::Vector3 getNormal() const { return cross(m_tinfo.ax2,m_tinfo.ax1); }

        TriangleOperation::TriangleInfo & m_tinfo;
    };

    void init() {
//        m_tinfoVector.resize(this->l_topology->getNbTriangles());

        for (unsigned i=0;i<this->l_topology->getNbTriangles();i++) {
            auto tri = this->l_topology->getTriangle(i);
            createElement<TriangleElement<TriangleProximity> >(this->getState(),tri[0],tri[1],tri[2]);


//            auto p0 = BaseProximity::SPtr(new PointOperation::PointProximity(this->getState(),PointOperation::NormalHandler::SPtr(new PointTriangleNormalHandler(m_tinfoVector[i])),tri[0]));
//            auto p1 = BaseProximity::SPtr(new PointOperation::PointProximity(this->getState(),PointOperation::NormalHandler::SPtr(new PointTriangleNormalHandler(m_tinfoVector[i])),tri[1]));
//            auto p2 = BaseProximity::SPtr(new PointOperation::PointProximity(this->getState(),PointOperation::NormalHandler::SPtr(new PointTriangleNormalHandler(m_tinfoVector[i])),tri[2]));

//            auto elmt = BaseElement::SPtr(new TriangleOperation::TriangleElement(p0,p1,p2,m_tinfoVector[i]));

//            auto triangle = this->l_topology->getTriangles()[i];
        }
    }

    inline BaseElement::Iterator begin(Index eid = 0) const override {
        return TriangleOperation::begin(this, this->l_topology->getTriangles(), eid);
    }

//    virtual type::Vector3 computeNormal(const PROXIMITYDATA & data) const override {
//        auto tinfo = getTriangleInfo()[data.m_eid];
//        return cross(tinfo.ax2,tinfo.ax1);
//    }

//    std::vector<TriangleOperation::TriangleInfo> m_tinfoVector;

};


}

}
