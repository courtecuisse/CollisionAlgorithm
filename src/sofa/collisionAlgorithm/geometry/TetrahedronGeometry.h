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

//    void init() {
////        this->m_topoProx.clear();
////        for (unsigned j=0; j<this->getState()->getSize(); j++) {
////            this->m_topoProx.push_back(TBaseProximity<DataTypes>::template create<TopologyProximity<DataTypes>>(this->getState(), j));
////        }

//        TriangleGeometry<DataTypes>::init();

//        //default proximity creator
//        m_tetraElements.clear();
//        for (unsigned i=0;i<this->l_topology->getNbTetrahedra();i++) {
//            auto tetra = this->l_topology->getTetrahedron(i);
//            auto triId = this->l_topology->getTrianglesInTetrahedron(i);

//// //            auto triangle0 = TriangleGeometry<DataTypes>::triangleBegin(triId[0])->element();
//// //            auto triangle1 = TriangleGeometry<DataTypes>::triangleBegin(triId[1])->element();
//// //            auto triangle2 = TriangleGeometry<DataTypes>::triangleBegin(triId[2])->element();
//// //            auto triangle3 = TriangleGeometry<DataTypes>::triangleBegin(triId[3])->element();
//            TriangleElement::SPtr triangle0 = TriangleGeometry<DataTypes>::getElements()[triId[0]];
//            TriangleElement::SPtr triangle1 = TriangleGeometry<DataTypes>::getElements()[triId[1]];
//            TriangleElement::SPtr triangle2 = TriangleGeometry<DataTypes>::getElements()[triId[2]];
//            TriangleElement::SPtr triangle3 = TriangleGeometry<DataTypes>::getElements()[triId[3]];

////            auto elmt = BaseElement::create<TetrahedronElement>(this->m_topoProx[tetra[0]],this->m_topoProx[tetra[1]],this->m_topoProx[tetra[2]],this->m_topoProx[tetra[3]]);
//            auto elmt = BaseElement::create<TetrahedronElement>(triangle0, triangle1, triangle2, triangle3, this->m_topoProx[tetra[0]],this->m_topoProx[tetra[1]],this->m_topoProx[tetra[2]],this->m_topoProx[tetra[3]]);
//            m_tetraElements.push_back(elmt);
//        }

//        prepareDetection();
//    }


    void buildTetrahedronElements() override {
        for (unsigned i=0;i<this->l_topology->getNbTetrahedra();i++) {
            auto triId = this->l_topology->getTrianglesInTetrahedron(i);

            TriangleElement::SPtr triangle0 = this->triangleElements()[triId[0]];
            TriangleElement::SPtr triangle1 = this->triangleElements()[triId[1]];
            TriangleElement::SPtr triangle2 = this->triangleElements()[triId[2]];
            TriangleElement::SPtr triangle3 = this->triangleElements()[triId[3]];

            this->insert(TetrahedronElement::SPtr(triangle0, triangle1, triangle2, triangle3));
        }
    }

    ElementIterator::SPtr begin(unsigned id = 0) const override {
        return ElementIterator::SPtr(new TDefaultElementIteratorSPtr(this->tetrahedronElements(),id));
    }

};

}
