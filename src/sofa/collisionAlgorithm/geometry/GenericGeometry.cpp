#include <sofa/collisionAlgorithm/geometry/GenericGeometry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(GenericGeometry)

int GenericGeometryClass = core::RegisterObject("GenericGeometry")
.add< GenericGeometry<sofa::defaulttype::Vec3dTypes, sofa::core::topology::BaseMeshTopology::Edge,2> >()
.add< GenericGeometry<sofa::defaulttype::Vec3dTypes, sofa::core::topology::BaseMeshTopology::Triangle,3> >()
.add< GenericGeometry<sofa::defaulttype::Vec3dTypes, sofa::core::topology::BaseMeshTopology::Quad,4> >();


//Data<helper::vector<sofa::core::topology::BaseMeshTopology::Edge>> d_edges;
//Data<helper::vector<sofa::core::topology::BaseMeshTopology::Triangle>> d_triangles;
//Data<helper::vector<sofa::core::topology::BaseMeshTopology::Quad>> d_quads;

}

}




/*
 *
 *
 *

#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class GEOMETRY, class ELMT>
class GenericElement : public BaseElement {
public:
    typedef GEOMETRY TGeometry;
    typedef typename GEOMETRY::TDataTypes DataTypes;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;

    GenericElement(unsigned id,const GEOMETRY * geo) : m_tid(id), m_container(geo) {}

    inline BaseProximity::SPtr project(const defaulttype::Vector3 & P) const {
        return NULL;
    }

    inline BaseProximity::SPtr center() const {
        return NULL;
    }

    inline defaulttype::BoundingBox getBBox() const {
//        const ELMT & elmt = this->getValue()[pid];

        defaulttype::BoundingBox bbox;
//        for (unsigned i=0;i<ELMT::size();i++) {
//            defaulttype::Vector3 pi(m_owner->getState()->getPX(elmt[i]),
//                                    m_owner->getState()->getPY(elmt[i]),
//                                    m_owner->getState()->getPZ(elmt[i]));
//            bbox.include(pi);
//        }

        return bbox;
    }
protected:
    unsigned m_tid;
    const GEOMETRY * m_container;
};



template<class DataTypes>
class GenericGeometry : public TBaseGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
        typedef TBaseGeometry<DataTypes> Inherit;
    typedef GenericGeometry<DataTypes> GEOMETRY;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;
    typedef sofa::core::topology::BaseMeshTopology::Triangle Triangle;
    typedef size_t TriangleID; // to remove once TriangleID has been changed to size_t in BaseMeshTopology
    typedef helper::vector<Triangle> VecTriangles;

    SOFA_CLASS(GEOMETRY,Inherit);

    Data<helper::vector<sofa::core::topology::BaseMeshTopology::Edge>> d_edges;
    Data<helper::vector<sofa::core::topology::BaseMeshTopology::Triangle>> d_triangles;
    Data<helper::vector<sofa::core::topology::BaseMeshTopology::Quad>> d_quads;

    GenericGeometry()
    : d_edges(initData(&d_edges, "edges", "Vector of Edges"))
    , d_triangles(initData(&d_triangles, "triangles", "Vector of Triangles"))
    , d_quads(initData(&d_quads, "quads", "Vector of Quads")){}

    virtual BaseElementIterator::UPtr getElementIterator(unsigned eid = 0) const {
        if (eid < d_edges.getValue().size())
            return DefaultElementIterator<GenericElement<GEOMETRY,sofa::core::topology::BaseMeshTopology::Edge> >::create(this, d_edges.getValue().size(), eid);

        eid -= d_edges.getValue().size();
        if (eid < d_triangles.getValue().size())
            return DefaultElementIterator<GenericElement<GEOMETRY,sofa::core::topology::BaseMeshTopology::Triangle> >::create(this, d_triangles.getValue().size(), eid);

    }

protected:

    ////void BaseProximity::getControlPoints(helper::vector<Vector3> & controlPoints) {
    ////    helper::vector<double> prev = m_fact;
    ////    m_fact.clear();
    ////    m_fact.resize(prev.size(),0.0);
    ////    for (unsigned i=0;i<m_fact.size();i++) {
    ////        m_fact[i] = 1.0;
    ////        controlPoints.push_back(getPosition());
    ////        m_fact[i] = 0.0;
    ////    }
    ////    m_fact = prev;
    ////}

    ////void BaseProximity::inc(const helper::vector<double> & dir) {
    ////    //apply dx
    ////    for (unsigned i=0;i<dir.size();i++) m_fact[i]+=dir[i];

    ////    //clamp the bary coord to the element
    //////            double sum = 0.0;
    //////            for (unsigned i=0;i<m_fact.size();i++) sum += m_fact[i];
    //////            if (sum == 0.0) return;
    //////            for (unsigned j=0;j<m_fact.size();j++) m_fact[j] *= 1.0/sum;

    ////    for (unsigned i=0;i<m_fact.size();i++) {
    ////        if (m_fact[i] < 0) {
    ////            double remove = m_fact[i];
    ////            m_fact[i] = 0;
    ////            for (unsigned j=0;j<m_fact.size();j++) m_fact[j] /= (1.0 - remove);
    //////                    usePoints[i] = false;
    //////                    printf("CLAMP %d\n",i);
    ////        }
    ////    }

    ////    double sum = 0.0;
    ////    for (unsigned i=0;i<m_fact.size();i++) sum += m_fact[i];
    ////    if (sum == 0.0) return;
    ////    for (unsigned j=0;j<m_fact.size();j++) m_fact[j] *= 1.0/sum;

    //////            std::cout << "FACT (" << sum << ") = " << m_fact << std::endl;
    ////}
    ///
    ///
    ///


//    inline Eigen::MatrixXd pinv(const Eigen::MatrixXd & m) {
//        double epsilon= 1e-15;
//        Eigen::JacobiSVD<Eigen::MatrixXd> svd(m, Eigen::ComputeThinU | Eigen::ComputeThinV);
//        const Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singVals = svd.singularValues();
//        Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType invSingVals = singVals;
//        for(int i=0; i<singVals.rows(); i++) {
//            if(singVals(i)*singVals(i) <= epsilon*epsilon) invSingVals(i) = 0.0;
//            else invSingVals(i) = 1.0 / invSingVals(i);
//        }
//        Eigen::MatrixXd S_inv = invSingVals.asDiagonal();
//        Eigen::MatrixXd m_inverse = svd.matrixV()*S_inv* svd.matrixU().transpose();
//        return m_inverse;
//    }

//    virtual BaseProximityPtr newton_project(defaulttype::Vector3 Q) {
//        const int maxIt = 1;
//        const double tolerance = 0.0001;
////        const double threshold = 0.0000001;
//        double delta = 0.001;

//        std::vector<BaseProximityPtr> controlPoints;
//        for (unsigned i=0;i<m_controlPoints;i++) controlPoints.push_back(getControlPoint(i));

//        BaseProximityPtr res = getControlPoint();

//        if (controlPoints.size() <= 1) return res;

//        //check the control points that are necessary
//        std::vector<bool> usePoints(controlPoints.size(), true);
//        std::vector<double> inc(controlPoints.size(), 0.0);

//        int it = 0;

//        while (it< maxIt) {
//            defaulttype::Vector3 P = res->getPosition();


//    //        for (unsigned i=0;i<m_fact.size();i++) {
//    //            if (m_fact[i] == 0) usePoints[i] = dot(Q - P,controlPoints[i] - P) > 0;
//    //            else usePoints[i] = true;

//    //            if (usePoints[i]) normals.push_back((controlPoints[i] - P)*delta);
//    //        }

//            unsigned JLin = (unsigned) controlPoints.size();

//    //        if (JLin == 0) break;

//            Eigen::VectorXd e0(JLin);
//            Eigen::MatrixXd J = Eigen::MatrixXd::Zero(JLin,JLin);

//            defaulttype::Vector3 PQ = Q-P;

//            std::vector<defaulttype::Vector3> normals;

//            double err=0.0;
//            for (unsigned j=0;j<JLin;j++) {
//    //            helper::vector<double> inc(controlPoints.size(), 0.0);
//    //            inc[j] = delta;
//    //            BaseProximityPtr sigma = move(inc.ptr());
//                normals.push_back(controlPoints[j]->getPosition() - P);
//                double e = dot(PQ,normals[j]);
//                e0(j) = e;
//                err += e*e;
//            }

//            if (sqrt(err)<tolerance) break;

//            for (unsigned j=0;j<JLin;j++) {
//                const defaulttype::Vector3 R = P + normals[j] * delta;
//                for (unsigned i=0;i<JLin;i++) {
//                    const double fxdx = dot(R, normals[i]);
//                    J(i,j) = (fxdx - e0(i))/ delta;
//                }
//            }

//            std::cout << "e0=\n" << e0 << std::endl;
//            std::cout << "J=\n" << J << std::endl;

//            Eigen::MatrixXd invJ = pinv(J);
//            Eigen::VectorXd dx = -invJ * e0;

//            std::cout << "dx=\n" << dx << std::endl;

////            res->moveToControlPoints(dx.data());

//    //        helper::vector<double> Dx;

//    //        int k=0;
//    //        for (unsigned i=0;i<usePoints.size();i++) {
//    //            if (usePoints[i]) Dx.push_back(dx(k++));
//    //            else Dx.push_back(0.0);
//    //        }

//    //        helper::vector<double> prev = m_fact;

//            it++;
//    //        this->inc(Dx);


//    //        double res = 0.0;
//    //        for (unsigned i=0;i<m_fact.size();i++) res += std::pow((prev[i] - m_fact[i]),2);
//    //        if (sqrt(res) < threshold) break;

//    //                std::cout << "DX(" << sqrt(res) << ")=" << Dx << "    |||||| " << normals << std::endl;
//        }

//        return res;


//    //            double sum = 0.0;
//    //            for (unsigned i=0;i<m_fact.size();i++) sum += m_fact[i];
//    //            std::cout << "NEWTON ITERATIONS " << it << " sum=" << sum << " fact=" << m_fact << std::endl;
//    }
};


}

}


*/
