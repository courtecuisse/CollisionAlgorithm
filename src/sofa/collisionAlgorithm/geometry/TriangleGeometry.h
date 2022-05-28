#pragma once

#include <sofa/collisionAlgorithm/geometry/EdgeGeometry.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>
#include <sofa/collisionAlgorithm/elements/TriangleElement.h>
#include <sofa/collisionAlgorithm/toolbox/TriangleToolBox.h>
#include <sofa/collisionAlgorithm/proximity/TopologyProximity.h>

namespace sofa::collisionAlgorithm {

template<class DataTypes>
class TriangleGeometry : public EdgeGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef TriangleElement ELEMENT;
    typedef TriangleGeometry<DataTypes> GEOMETRY;
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef std::function<BaseProximity::SPtr(const TriangleElement * elmt, double f0,double f1,double f2)> ProximityCreatorFunc;

    SOFA_CLASS(GEOMETRY,Inherit);

    core::objectmodel::SingleLink<GEOMETRY,core::topology::BaseMeshTopology,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_topology;

    TriangleGeometry()
    : l_topology(initLink("topology", "link to topology")) {
        l_topology.setPath("@.");
    }

    void buildTriangleElements() override {
        for (unsigned i=0;i<this->l_topology->getNbTriangles();i++) {
//            auto tri = this->l_topology->getTriangle(i);
            auto edgeId = this->l_topology->getEdgesInTriangle(i);

            EdgeElement::SPtr edge0 = this->edgeElements()[edgeId[0]];
            EdgeElement::SPtr edge1 = this->edgeElements()[edgeId[1]];
            EdgeElement::SPtr edge2 = this->edgeElements()[edgeId[2]];

//            auto elmt = BaseElement::create<TriangleElement>(this->m_topoProx[tri[0]],this->m_topoProx[tri[1]],this->m_topoProx[tri[2]]);

            this->insert(TriangleElement::SPtr(edge0, edge1, edge2));
        }
    }

    inline ElementIterator::SPtr begin(unsigned id = 0) const override {
        return ElementIterator::SPtr(new TDefaultElementIteratorSPtr(this->triangleElements(),id));
    }

};

}

