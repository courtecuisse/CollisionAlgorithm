#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/toolbox/EdgeToolBox.h>
#include <sofa/collisionAlgorithm/proximity/EdgeProximity.h>
#include <sofa/collisionAlgorithm/proximity/TopologyProximity.h>

namespace sofa::collisionAlgorithm {

template<class DataTypes>
class EdgeGeometry : public TBaseGeometry<DataTypes>, public EdgeProximityCreator {
public:
    typedef DataTypes TDataTypes;
    typedef EdgeElement ELEMENT;
    typedef EdgeGeometry<DataTypes> GEOMETRY;
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef std::function<BaseProximity::SPtr(const EdgeElement * elmt,double f0,double f1)> ProximityCreatorFunc;

    SOFA_CLASS(GEOMETRY,Inherit);

    core::objectmodel::SingleLink<GEOMETRY,core::topology::BaseMeshTopology,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_topology;

    EdgeGeometry()
    : l_topology(initLink("topology", "link to topology")) {
        l_topology.setPath("@.");
        f_createProximity = [=](const EdgeElement * elmt,double f0,double f1) -> BaseProximity::SPtr {
            return BaseProximity::create<EdgeProximity>(elmt->getP0(),elmt->getP1(),f0,f1);
        };
    }

    void init() {
        for (unsigned i=0;i<this->l_topology->getNbEdges();i++) {
            auto edge = this->l_topology->getEdge(i);
            m_elements.push_back(BaseElement::create<EdgeElement>(this,this->m_topoProx[edge[0]],this->m_topoProx[edge[1]]));
        }
    }

//    type::Vector3 getPosition(unsigned pid) override {
//        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());
//        return pos[pid];
//    }

    void prepareDetection() override {}

    inline ElementIterator::SPtr begin() const override {
        return ElementIterator::SPtr(new TDefaultElementIteratorSPtr(m_elements));
    }

    BaseProximity::SPtr createProximity(const EdgeElement * elmt,double f0,double f1) override {
        return f_createProximity(elmt,f0,f1);
    }

    void setCreateProximity(ProximityCreatorFunc f) {
        f_createProximity = f;
    }

private:
    std::vector<EdgeElement::SPtr> m_elements;
    ProximityCreatorFunc f_createProximity;
};

}
