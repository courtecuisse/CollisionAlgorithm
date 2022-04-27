#pragma once

#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>
#include <sofa/collisionAlgorithm/elements/TetrahedronElement.h>
#include <sofa/collisionAlgorithm/proximity/TetrahedronProximity.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/collisionAlgorithm/proximity/TopologyProximity.h>

namespace sofa::collisionAlgorithm {

template<class DataTypes>
class TetrahedronGeometry : public TriangleGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef TetrahedronElement ELEMENT;
    typedef TetrahedronGeometry<DataTypes> GEOMETRY;
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef std::function<BaseProximity::SPtr(const TetrahedronElement * elmt,double f0,double f1,double f2,double f3)> ProximityCreatorFunc;

    SOFA_CLASS(GEOMETRY,Inherit);

    core::objectmodel::SingleLink<GEOMETRY,core::topology::BaseMeshTopology,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_topology;

    TetrahedronGeometry()
    : l_topology(initLink("topology", "link to topology")) {
        l_topology.setPath("@.");
    }

//    type::Vector3 getPosition(unsigned pid) override {
//        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());
//        return pos[pid];
//    }

    void init() {
        for (unsigned j=0; j<this->getState()->getSize(); j++) {
            this->m_topoProx.push_back(TBaseProximity<DataTypes>::template create<TopologyProximity<DataTypes>>(this->getState(), j));
        }

        //default proximity creator
        for (unsigned i=0;i<this->l_topology->getNbTetrahedra();i++) {
            auto tetra = this->l_topology->getTetrahedron(i);
            auto elmt = BaseElement::create<TetrahedronElement>(this->m_topoProx[tetra[0]],this->m_topoProx[tetra[1]],this->m_topoProx[tetra[2]],this->m_topoProx[tetra[3]]);
            m_elements.push_back(elmt);
        }

        prepareDetection();
    }

    ElementIterator::SPtr begin() const override {
        return ElementIterator::SPtr(new TDefaultElementIteratorSPtr(m_elements));
    }

    virtual void prepareDetection() override {
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());
        for (unsigned i=0;i<m_elements.size();i++) m_elements[i]->update(pos.ref());
    }


    void setCreateProximity(ProximityCreatorFunc f) {
        for (unsigned i=0; i<m_elements.size(); i++) {
            m_elements[i]->setCreateProximity(f);
        }
    }


private:
    std::vector<TetrahedronElement::SPtr> m_elements;
};

}
