#pragma once

#include <sofa/collisionAlgorithm/CollisionPipeline.h>
#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>

namespace sofa::collisionAlgorithm {

template<class DataTypes>
class PhongTriangleNormalHandler : public CollisionComponent {
public:

    SOFA_CLASS(SOFA_TEMPLATE(PhongTriangleNormalHandler,DataTypes), CollisionComponent);

    typedef TriangleGeometry<DataTypes> GEOMETRY;
    typedef typename GEOMETRY::ELEMENT ELEMENT;

    core::objectmodel::SingleLink<PhongTriangleNormalHandler<DataTypes>,GEOMETRY,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;

    PhongTriangleNormalHandler()
    : l_geometry(initLink("geometry","Link to TriangleGeometry")){}

    class PhongTriangleProximity : public DefaultTriangleProximity<DataTypes> {
    public:
        typedef sofa::core::behavior::MechanicalState<DataTypes> State;

        PhongTriangleProximity(State * state, unsigned p0,unsigned p1, unsigned p2, double f0,double f1,double f2, type::vector<type::Vector3> & pn)
        : DefaultTriangleProximity<DataTypes>(state,p0,p1,p2,f0,f1,f2)
        , m_pointNormals(pn) {}

        virtual sofa::type::Vector3 getNormal() const override {
            return m_pointNormals[this->m_p0] * this->m_f0 +
                   m_pointNormals[this->m_p1] * this->m_f1 +
                   m_pointNormals[this->m_p2] * this->m_f2;
        }

    private:
        const type::vector<type::Vector3> & m_pointNormals;
    };

    void init() {
        prepareDetection();

        //change the behavior of elements
        for (auto it = l_geometry->begin();it != l_geometry->end(); it++) {
            ELEMENT * elmt = it->element_cast();
            elmt->setProximityCreator(
                [=](const TriangleElement * elmt, double f0,double f1,double f2) -> BaseProximity::SPtr {
                    return BaseProximity::SPtr(new PhongTriangleProximity(l_geometry->getState(),
                                                                          elmt->getP0(),elmt->getP1(),elmt->getP2(),
                                                                          f0,f1,f2,
                                                                          m_point_normals));
                }
            );
        }
    }

    void prepareDetection() override {
        m_point_normals.resize(this->l_geometry->l_topology->getNbPoints());

        m_triangle_normals.clear();
        for (auto it=l_geometry->begin();it!=l_geometry->end();it++) {
            TriangleElement * elmt = it->element_cast();
            auto tinfo = elmt->getTriangleInfo();
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
