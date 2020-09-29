#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/GenericProximity.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes, class Element, CONTROL_POINT SIZE>
class GenericGeometry : public TBaseGeometry<DataTypes,GenericProximity<SIZE> > {
public:
    typedef DataTypes TDataTypes;
    typedef TBaseGeometry<DataTypes,GenericProximity<SIZE> > Inherit;
    typedef BaseProximity::index_type index_type;
    typedef typename Inherit::PROXIMITYDATA PROXIMITYDATA;
    typedef GenericGeometry<DataTypes,Element,SIZE> GEOMETRY;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;
    typedef sofa::core::topology::BaseMeshTopology::Triangle Triangle;
    typedef size_t TriangleID; // to remove once TriangleID has been changed to size_t in BaseMeshTopology
    typedef helper::vector<Triangle> VecTriangles;

    SOFA_CLASS(GEOMETRY,Inherit);

    Data <index_type> d_nonlin_max_it;
    Data <double> d_nonlin_tolerance;
    Data <double> d_nonlin_threshold;
    Data <double> d_delta;

    Data<helper::vector<Element> > d_elements;

    GenericGeometry()
    : d_nonlin_max_it(initData(&d_nonlin_max_it,(index_type) 20,"nonlin_max_it", "number of iterations"))
    , d_nonlin_tolerance(initData(&d_nonlin_tolerance,(double) 0.001,"nonlin_tol", "Tolerance"))
    , d_nonlin_threshold(initData(&d_nonlin_threshold,(double) 0.00001,"nonlin_th", "Threshold"))
    , d_delta(initData(&d_delta,(double) 0.00001,"delta", "Delta"))
    , d_elements(initData(&d_elements, "elements", "Vector of Elements")) {}

    virtual BaseElementIterator::UPtr begin(index_type eid = 0) const override {
        return DefaultElementIterator<PROXIMITYDATA>::create(this, d_elements.getValue(), eid);
    }

    inline defaulttype::Vector3 getPosition(const PROXIMITYDATA & data, core::VecCoordId v = core::VecCoordId::position()) const {
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(v);
        defaulttype::Vector3 P;
        for (index_type i=0;i<data.m_prox.size();i++) {
            P += pos[data.m_prox[i].first] * data.m_prox[i].second;
        }
        return P;
    }

    inline PROXIMITYDATA createProximity(index_type eid,CONTROL_POINT pid = CONTROL_DEFAULT) const {
        return PROXIMITYDATA::create(eid, pid);
    }

    void normalize(PROXIMITYDATA & result) const {
        //Normalize (sum = 1)
        double sum = 0.0;
        for (index_type i=0;i<result.m_prox.size();i++) sum += result.m_prox[i].second;
        if (sum == 0.0) return;
        for (index_type j=0;j<result.m_prox.size();j++) result.m_prox[j].second *= 1.0/sum;
    }

    inline Eigen::MatrixXd pinv(const Eigen::MatrixXd & m) const {
        double epsilon= 1e-15;
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(m, Eigen::ComputeThinU | Eigen::ComputeThinV);
        const Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singVals = svd.singularValues();
        Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType invSingVals = singVals;
        for(int i=0; i<singVals.rows(); i++) {
            if(singVals(i)*singVals(i) <= epsilon*epsilon) invSingVals(i) = 0.0;
            else invSingVals(i) = 1.0 / invSingVals(i);
        }
        Eigen::MatrixXd S_inv = invSingVals.asDiagonal();
        Eigen::MatrixXd m_inverse = svd.matrixV()*S_inv* svd.matrixU().transpose();
        return m_inverse;
    }

    virtual defaulttype::Vector3 computeNormal(const PROXIMITYDATA & /*data*/) const override {
        return defaulttype::Vector3();
    }

    inline PROXIMITYDATA project(defaulttype::Vector3 Q, index_type eid) const {
        double delta = d_delta.getValue();

        PROXIMITYDATA result = createProximity(eid);

        const helper::ReadAccessor<Data <VecCoord> >& x = this->getState()->read(core::VecCoordId::position());

        helper::vector<bool> usePoints(Element::size(),true);

        index_type it = 0;
        while (it< d_nonlin_max_it.getValue()) {
            defaulttype::Vector3 P = getPosition(result);
            defaulttype::Vector3 PQ = Q-P;

            //pair : index of control point <-> normals i.e. direction between the current point P and the control points of the element
            std::vector<std::pair<index_type,Coord> > normals;
            for (index_type i=0;i<usePoints.size();i++) {
                if (usePoints[i]) {
                    Coord dir = x[result.m_prox[i].first] - P;
                    normals.push_back(std::pair<index_type,Coord>(i,dir));
                }
            }

            if (normals.size() <= 1) break;

            index_type JLin = normals.size();
            Eigen::VectorXd e0(JLin);
            Eigen::MatrixXd J = Eigen::MatrixXd::Zero(JLin,JLin);

            //Compute error0 the error is the dot product between PQ (P is the current position) and the normals
            double err=0.0;
            for (index_type j=0;j<JLin;j++) {
                double e = dot(PQ,normals[j].second);
                e0(j) = e;
                err += e*e;
            }

            if (sqrt(err)<d_nonlin_tolerance.getValue()) break;

            //Compute jacobian
            for (index_type j=0;j<JLin;j++) {
                const defaulttype::Vector3 RQ = PQ + normals[j].second * delta;
                for (index_type i=0;i<JLin;i++) {
                    const double fxdx = dot(RQ, normals[i].second);
                    J(i,j) = (e0(i) - fxdx) / delta;
                }
            }

//            std::cout << "e0=\n" << e0 << std::endl;
//            std::cout << "J=\n" << J << std::endl;

            Eigen::MatrixXd invJ = pinv(J);
            Eigen::VectorXd dx = -invJ * e0;
//            std::cout << "dx=\n" << dx << std::endl;

            //Apply increment dx obtained from newton
            PROXIMITYDATA new_result = result;
            for (index_type i=0;i<JLin;i++) {
                index_type pid = normals[i].first;
                new_result.m_prox[pid].second += dx(i);
            }

            //make sure the point remains inside the element i.e. no bary coord negative
            //clamp the bary coord to the element
            // check if the point has left the element, If yes we reproject on the surface
            for (index_type i=0;i<Element::size();i++) {
                double & fact = new_result.m_prox[i].second;

                if (fact < 0) {
                    for (index_type j=0;j<new_result.m_prox.size();j++) {
                        new_result.m_prox[j].second /= (1.0 - fact);
                    }
                    usePoints[i] = false; // disable the point from the search
                    fact = 0;
                }
            }

            //make sure the sum equal 1
            normalize(new_result);

            //compute the real displacement
            double real_dx = 0.0;
            for (index_type i=0;i<Element::size();i++) real_dx += std::pow((new_result.m_prox[i].second - result.m_prox[i].second),2);
            real_dx *= 1.0/Element::size();

            result = new_result;

            if (sqrt(real_dx) < d_nonlin_threshold.getValue()) break;

//            std::cout << "DX(" << sqrt(res) << ")=" << Dx << "    |||||| " << normals << std::endl;

            it++;
        }

        return result;

//        double sum = 0.0;
//        for (index_type i=0;i<m_fact.size();i++) sum += m_fact[i];
//        std::cout << "NEWTON ITERATIONS " << it << " sum=" << sum << " fact=" << m_fact << std::endl;
    }


    virtual std::string getTemplateName() const
    {
        return templateName(this);
    }

    static std::string templateName(const GenericGeometry<DataTypes,Element,SIZE>* )
    {
        std::stringstream ss;
        ss << Element::size();
        return ss.str();
    }
};


}

}
