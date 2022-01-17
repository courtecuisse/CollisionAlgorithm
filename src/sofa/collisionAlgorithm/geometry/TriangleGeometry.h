#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>
#include <sofa/collisionAlgorithm/elements/TriangleElement.h>
#include <sofa/collisionAlgorithm/toolbox/TriangleToolBox.h>

namespace sofa::collisionAlgorithm {

template<class DataTypes>
class TriangleGeometry : public TBaseGeometry<DataTypes,TriangleElement> {
public:
    typedef DataTypes TDataTypes;
    typedef TriangleElement ELEMENT;
    typedef TriangleGeometry<DataTypes> GEOMETRY;
    typedef TBaseGeometry<DataTypes,ELEMENT> Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;

    SOFA_CLASS(GEOMETRY,Inherit);

    core::objectmodel::SingleLink<GEOMETRY,core::topology::BaseMeshTopology,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_topology;

    TriangleGeometry()
        : l_topology(initLink("topology", "link to topology")) {
        l_topology.setPath("@.");
    }

    void init() {
        //default proximity creator
        auto f = [=](const TriangleElement * elmt, double f0,double f1,double f2) -> BaseProximity::SPtr {
            return BaseProximity::SPtr(new DefaultTriangleProximity<DataTypes>(this->getState(),
                                                                        elmt->getP0(),elmt->getP1(),elmt->getP2(),
                                                                        f0,f1,f2));
        };

        for (unsigned i=0;i<this->l_topology->getNbTriangles();i++) {
            auto tri = this->l_topology->getTriangle(i);
            TriangleElement::SPtr elmt = this->createElement(i,tri[0],tri[1],tri[2],f);
            m_elements.push_back(elmt);
        }
    }

    inline BaseElement::Iterator begin(Index eid = 0) const override {
        return BaseElement::Iterator(new TDefaultElementIterator(m_elements,eid));
    }

private:
    std::vector<TriangleElement::SPtr> m_elements;

};

}

