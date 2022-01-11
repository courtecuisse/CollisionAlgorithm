#pragma once

#include <sofa/collisionAlgorithm/CollisionPipeline.h>
#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>

namespace sofa {

namespace collisionAlgorithm {


template<class DataTypes>
class PhongTriangleNormalHandler : public CollisionComponent {
public:

    SOFA_CLASS(SOFA_TEMPLATE(PhongTriangleNormalHandler,DataTypes), CollisionComponent);

    core::objectmodel::SingleLink<PhongTriangleNormalHandler<DataTypes>,TriangleGeometry<DataTypes>,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;

    /*!
     * \brief BaseAlgorithm Constructor
     */
    PhongTriangleNormalHandler()
    : l_geometry(initLink("geometry","Link to TriangleGeometry")){}

    void init() {
        update();

        //change the behavior of elements
        l_geometry->setPoximityCreator(
            [=](const TriangleElement * elmt, double f0,double f1,double f2) -> BaseProximity::SPtr {
                return BaseProximity::SPtr(new PhongTriangleProximity<DataTypes>(l_geometry->getState(),
                                                                                 elmt->getP0(),elmt->getP1(),elmt->getP2(),
                                                                                 f0,f1,f2,
                                                                                 m_point_normals));
            }
        );
    }

    void update() override {
        m_point_normals.resize(this->l_geometry->l_topology->getNbPoints());

        auto elements = this->l_geometry->getElements();

        m_triangle_normals.clear();
        for (size_t t=0;t<elements.size();t++) {
            auto tinfo = elements[t]->getTriangleInfo();
            m_triangle_normals.push_back(tinfo.N);
        }

        for (size_t p=0;p<m_point_normals.size();p++) {
            const sofa::core::topology::BaseMeshTopology::TrianglesAroundVertex & tav = this->l_geometry->l_topology->getTrianglesAroundVertex(p);
            m_point_normals[p] = type::Vector3(0,0,0);
            for (size_t t=0;t<tav.size();t++) {
                m_point_normals[p] += m_triangle_normals[tav[t]];
            }
            m_point_normals[p].normalize();
        }
    }

protected:
    type::vector<type::Vector3> m_triangle_normals;
    type::vector<type::Vector3> m_point_normals;

};


}

}
