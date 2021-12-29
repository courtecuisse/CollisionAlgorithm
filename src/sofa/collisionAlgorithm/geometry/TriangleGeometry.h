#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>
#include <sofa/collisionAlgorithm/operations/TriangleOperation.h>
#include <sofa/collisionAlgorithm/elements/TriangleElement.h>
#include <sofa/collisionAlgorithm/operations/TriangleOperation.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class TriangleGeometry : public TBaseGeometry<DataTypes>, public TriangleElementGeometry {
public:
    typedef DataTypes TDataTypes;
    typedef TriangleGeometry<DataTypes> GEOMETRY;
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;

    SOFA_CLASS(GEOMETRY,Inherit);

    core::objectmodel::SingleLink<GEOMETRY,core::topology::BaseMeshTopology,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_topology;

    TriangleGeometry()
        : l_topology(initLink("topology", "link to topology")) {
        l_topology.setPath("@.");
    }

    void init() {
        for (unsigned i=0;i<this->l_topology->getNbTriangles();i++) {
            auto tri = this->l_topology->getTriangle(i);
            m_elements.push_back(BaseElement::SPtr(new TriangleElement(this,tri[0],tri[1],tri[2])));
        }
    }

    const Operations::BaseOperation * getOperations() const override { return Operations::TriangleOperation::getOperation(); }

    BaseProximity::SPtr createProximity(unsigned p0, unsigned p1, unsigned p2, double f0,double f1,double f2) const override {

    }

    type::Vector3 getPosition(unsigned pid) const override {
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());
        return pos[pid];
    }

    inline BaseElement::Iterator begin(Index eid = 0) const override {
        return BaseElement::Iterator(new TDefaultElementIterator(m_elements,eid));
    }

private:
    std::vector<TriangleElement::SPtr> m_elements;

};


}

}
