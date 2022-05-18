#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
#include <sofa/collisionAlgorithm/toolbox/EdgeToolBox.h>
#include <sofa/collisionAlgorithm/proximity/EdgeProximity.h>
#include <sofa/collisionAlgorithm/proximity/TopologyProximity.h>

namespace sofa::collisionAlgorithm {

template<class DataTypes>
class EdgeGeometry : public PointGeometry<DataTypes> {
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
    }

    void init() {
        this->m_topoProx.clear();
        for (unsigned j=0; j<this->getState()->getSize(); j++) {
//            this->m_topoProx.push_back(TBaseProximity<DataTypes>::template create<TopologyProximity<DataTypes>>(this->getState(), j));
            this->m_topoProx.push_back(BaseProximity::create<TopologyProximity<DataTypes>>(this->getState(), j));
        }

        PointGeometry<DataTypes>::init();

        m_edgeElements.clear();
        for (unsigned i=0;i<this->l_topology->getNbEdges();i++) {
            auto edge = this->l_topology->getEdge(i);

//            PointElement::SPtr point0 = PointGeometry<DataTypes>::getElements()[edge[0]];
//            PointElement::SPtr point1 = PointGeometry<DataTypes>::getElements()[edge[1]];
//            m_edgeElements.push_back(BaseElement::create<EdgeElement>(point0, point1));
            m_edgeElements.push_back(BaseElement::create<EdgeElement>(this->m_topoProx[edge[0]], this->m_topoProx[edge[1]]));
        }
    }

//    type::Vector3 getPosition(unsigned pid) override {
//        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());
//        return pos[pid];
//    }

    void prepareDetection() override {PointGeometry<DataTypes>::prepareDetection();}

    inline ElementIterator::SPtr begin(unsigned id = 0) const override {
        return edgeBegin(id);
    }

    inline ElementIterator::SPtr edgeBegin(unsigned id = 0) const {
        return ElementIterator::SPtr(new TDefaultElementIteratorSPtr(m_edgeElements,id));
    }





    void setCreateEdgeProximity(ProximityCreatorFunc f) {
        for (unsigned i = 0; i<m_edgeElements.size(); i++) {
            m_edgeElements[i]->setCreateProximity(f);
        }
    }

    inline std::vector<EdgeElement::SPtr> & getElements() {
        return m_edgeElements;
    }


private:
    std::vector<EdgeElement::SPtr> m_edgeElements;
};

}
