#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
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
//        m_tetra_info.clear();
    }

//    inline PROXIMITYDATA project(const type::Vector3 & P, Index eid) const {
//        auto tetrahedron = getTetrahedron(eid);
//        const TetraInfo & tinfo = getTetraInfo()[eid];

//        double fact[4];
//        toolBox::projectOnTetra( P, tinfo,fact[0],fact[1],fact[2],fact[3]);

//        return PROXIMITYDATA(eid, tetrahedron[0], tetrahedron[1], tetrahedron[2], tetrahedron[3], fact[0],fact[1],fact[2],fact[3]);
//    }

//    //proj_P must be on the plane

//    const std::vector<TetraInfo> & getTetraInfo(core::VecCoordId v = core::VecCoordId::position()) const {
//        if (m_tetra_info[v.getIndex()].empty()) computeTetraInfo(v);
//        return m_tetra_info[v.getIndex()];
//    }

//protected:
//    mutable std::map<int,std::vector<TetraInfo> > m_tetra_info;

//    void computeTetraInfo(core::VecCoordId v = core::VecCoordId::position()) const {
//        std::vector<TetraInfo> & vecInfo = m_tetra_info[v.getIndex()];
//        const VecTetrahedron& tetrahedra = this->l_topology->getTetrahedra();

//        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());

//        vecInfo.clear();
//        for (size_t t=0 ; t<tetrahedra.size() ; t++) {
//            const Tetrahedron& tri = tetrahedra[t];

//            const sofa::core::topology::BaseMeshTopology::TrianglesInTetrahedron& TIT = this->l_topology->getTrianglesInTetrahedron(t);

//            const type::Vector3 & p0 = pos[tri[0]];
//            const type::Vector3 & p1 = pos[tri[1]];
//            const type::Vector3 & p2 = pos[tri[2]];
//            const type::Vector3 & p3 = pos[tri[3]];

//            vecInfo.push_back(toolBox::computeTetraInfo(p0,p1,p2,p3));
//        }
//    }

private:
    std::vector<TetrahedronElement::SPtr> m_elements;
};

}
