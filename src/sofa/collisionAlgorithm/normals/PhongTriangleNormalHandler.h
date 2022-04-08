#pragma once

#include <sofa/collisionAlgorithm/CollisionPipeline.h>
#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>

namespace sofa::collisionAlgorithm {

//#ifdef NOT_COMPILING

template<class DataTypes>
class PhongTriangleNormalHandler : public CollisionComponent {
public:

    SOFA_CLASS(SOFA_TEMPLATE(PhongTriangleNormalHandler,DataTypes), CollisionComponent);

    typedef TriangleGeometry<DataTypes> GEOMETRY;
    typedef typename GEOMETRY::ELEMENT ELEMENT;

    core::objectmodel::SingleLink<PhongTriangleNormalHandler<DataTypes>,GEOMETRY,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;

    PhongTriangleNormalHandler()
    : l_geometry(initLink("geometry","Link to TriangleGeometry")){}

//    class PhongTriangleProximity : public TriangleProximity {
//    public:
//        typedef sofa::core::behavior::MechanicalState<DataTypes> State;

//        PhongTriangleProximity(typename TopologyProximity<DataTypes>::SPtr p0,typename TopologyProximity<DataTypes>::SPtr p1,typename TopologyProximity<DataTypes>::SPtr p2, double f0,double f1,double f2, const type::vector<type::Vector3> * pn)
//        : TriangleProximity(p0,p1,p2,f0,f1,f2)
//        , m_pointNormals(*pn) {}

//        virtual sofa::type::Vector3 getNormal() const override {
////            return m_pointNormals[this->m_p0] * this->m_f0 +
////                   m_pointNormals[this->m_p1] * this->m_f1 +
////                   m_pointNormals[this->m_p2] * this->m_f2;


////            if (dynamic_cast<typename TopologyProximity<DataTypes>::SPtr>(this->m_p0) != NULL) {
//            if ((typeid(this->m_p0) == typeid(typename TopologyProximity<DataTypes>::SPtr)) &&
//                (typeid(this->m_p1) == typeid(typename TopologyProximity<DataTypes>::SPtr)) &&
//                (typeid(this->m_p2) == typeid(typename TopologyProximity<DataTypes>::SPtr))) {

////            if ((typename TopologyProximity<DataTypes>* p0Topo = dynamic_cast<typename TopologyProximity<DataTypes>*>(this->m_p0)) &&
////                (typename TopologyProximity<DataTypes>* p1Topo = dynamic_cast<typename TopologyProximity<DataTypes>*>(this->m_p1)) &&
////                (typename TopologyProximity<DataTypes>* p2Topo = dynamic_cast<typename TopologyProximity<DataTypes>*>(this->m_p2))) {

////            if (typename TopologyProximity<DataTypes>* p0Topo = dynamic_cast<typename TopologyProximity<DataTypes>*>(this->m_p0)) {

//                typename TopologyProximity<DataTypes>::SPtr p0Topo1 = this->m_p0;
//                typename TopologyProximity<DataTypes>::SPtr p1Topo1 = this->m_p1;
//                typename TopologyProximity<DataTypes>::SPtr p2Topo1 = this->m_p2;

//                TopologyProximity<DataTypes>* p0Topo = dynamic_cast<TopologyProximity<DataTypes>*>(p0Topo1.get());
//                TopologyProximity<DataTypes>* p1Topo = dynamic_cast<TopologyProximity<DataTypes>*>(p1Topo1.get());
//                TopologyProximity<DataTypes>* p2Topo = dynamic_cast<TopologyProximity<DataTypes>*>(p2Topo1.get());

////                typename TopologyProximity<DataTypes>::SPtr p0Topo = dynamic_cast<typename TopologyProximity<DataTypes>::SPtr>(this->m_p0.get());
////                typename TopologyProximity<DataTypes>::SPtr p1Topo = dynamic_cast<typename TopologyProximity<DataTypes>::SPtr>(this->m_p1.get());
////                typename TopologyProximity<DataTypes>::SPtr p2Topo = dynamic_cast<typename TopologyProximity<DataTypes>::SPtr>(this->m_p2.get());

//                return m_pointNormals[p0Topo->getPId()] * this->m_f0 +
//                       m_pointNormals[p1Topo->getPId()] * this->m_f1 +
//                       m_pointNormals[p2Topo->getPId()] * this->m_f2;

////                return m_pointNormals[this->m_p0->getPId()] * this->m_f0 +
////                       m_pointNormals[this->m_p1->getPId()] * this->m_f1 +
////                       m_pointNormals[this->m_p2->getPId()] * this->m_f2;
//            }

//               return type::Vector3();


//        }

//    private:
//        const type::vector<type::Vector3> & m_pointNormals;
//    };

//    BaseProximity::SPtr createPhongProximity(const TriangleElement * elmt, double f0,double f1,double f2)  {
//        return BaseProximity::create<PhongTriangleProximity>(elmt->getP0(),elmt->getP1(),elmt->getP2(),
//                                                              f0,f1,f2,
//                                                              &m_point_normals);
//    }

//    void init() {
//        prepareDetection();

//        l_geometry->setCreateProximity(
//                std::bind(&PhongTriangleNormalHandler::createPhongProximity,this,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4)
//        );
//    }

    void prepareDetection() override {
//        m_point_normals.resize(this->l_geometry->l_topology->getNbPoints());

//        m_triangle_normals.clear();
//        for (auto it=l_geometry->begin();it!=l_geometry->end();it++) {
//            TriangleElement * elmt = it->element_cast();
//            auto tinfo = elmt->getTriangleInfo();
//            m_triangle_normals.push_back(tinfo.N);
//        }

//        for (size_t p=0;p<m_point_normals.size();p++) {
//            const sofa::core::topology::BaseMeshTopology::TrianglesAroundVertex & tav = this->l_geometry->l_topology->getTrianglesAroundVertex(p);
//            m_point_normals[p] = type::Vector3(0,0,0);
//            for (size_t t=0;t<tav.size();t++) {
//                m_point_normals[p] += m_triangle_normals[tav[t]];
//            }
//            m_point_normals[p].normalize();
//        }

        for (unsigned i=0;i<l_geometry->getTopoProx().size();i++) {
            const sofa::core::topology::BaseMeshTopology::TrianglesAroundVertex & tav = this->l_geometry->l_topology->getTrianglesAroundVertex(i);
            type::Vector3 N(0,0,0);
            for (size_t t=0;t<tav.size();t++) {
                unsigned eid = tav[t];
                auto element = l_geometry->getElements()[eid];
                N += element->getTriangleInfo().N;
            }
            N.normalize();

            l_geometry->getTopoProx()[i]->setNormal(N);
        }
    }

protected:
//    type::vector<type::Vector3> m_triangle_normals;
//    type::vector<type::Vector3> m_point_normals;
};


//#endif
}
