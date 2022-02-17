#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/elements/TetrahedronElement.h>
#include <sofa/collisionAlgorithm/proximity/TetrahedronProximity.h>
#include <sofa/core/topology/BaseMeshTopology.h>

namespace sofa::collisionAlgorithm {

template<class DataTypes>
class TetrahedronGeometry : public TBaseGeometry<DataTypes,TetrahedronElement> {
public:
    typedef DataTypes TDataTypes;
    typedef TetrahedronElement ELEMENT;
    typedef TetrahedronGeometry<DataTypes> GEOMETRY;
    typedef TBaseGeometry<DataTypes,ELEMENT> Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;

    SOFA_CLASS(GEOMETRY,Inherit);

    core::objectmodel::SingleLink<GEOMETRY,core::topology::BaseMeshTopology,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_topology;

    TetrahedronGeometry()
    : l_topology(initLink("topology", "link to topology")) {
        l_topology.setPath("@.");
    }

    void init() {
        //default proximity creator
        auto f = [=](const TetrahedronElement * elmt, double f0,double f1,double f2,double f3) -> BaseProximity::SPtr {
            return BaseProximity::SPtr(new DefaultTetrahedronProximity<DataTypes>(this->getState(),
                                                                                  elmt->getP0(),elmt->getP1(),elmt->getP2(),elmt->getP3(),
                                                                                  f0,f1,f2,f3));
        };

        for (unsigned i=0;i<this->l_topology->getNbTetrahedra();i++) {
            auto tetra = this->l_topology->getTetrahedron(i);
            TetrahedronElement::SPtr elmt = this->createElement(i,tetra[0],tetra[1],tetra[2],tetra[3],f);
            m_elements.push_back(elmt);
        }

        prepareDetection();
    }

    ElementIterator::SPtr begin() const override {
        return ElementIterator::SPtr(new TDefaultElementIterator(m_elements));
    }

    virtual void prepareDetection() override {
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());
        for (unsigned i=0;i<m_elements.size();i++) m_elements[i]->update(pos.ref());
    }

private:
    std::vector<TetrahedronElement::SPtr> m_elements;
};

}
