#pragma once

#include <collisionAlgorithm.h>
#include <memory>
#include <map>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <qopengl.h>

namespace collisionAlgorithm {

class ConstraintElement;
class BaseGeometry;

class ConstraintProximity {
public :
    typedef std::shared_ptr<ConstraintProximity> ConstraintProximityPtr;

    ConstraintProximity(ConstraintElement * elmt)
    : m_element(elmt) {}

    virtual Vector3 getPosition(VecCoordId v = VecCoordId::position()) const = 0;

    virtual Vector3 getNormal() const = 0;

    virtual std::map<unsigned,double> getContributions() = 0;

    virtual double distance(const Vector3 & P) {
        return (this->getPosition() - P).norm();
    }

    ConstraintElement * m_element;
    State * m_state;
};

typedef ConstraintProximity::ConstraintProximityPtr ConstraintProximityPtr;

template<int C>
using ControlPoint=fixed_array<ConstraintProximityPtr,C>;

class ConstraintElement {
    friend class ConstraintProximity;

public:
    typedef std::shared_ptr<ConstraintElement> ConstraintElementPtr;

    ConstraintElement(State * s,ControlPoint<1> cp,ConstraintProximityPtr c)
    : m_centerPoint(c) {
        m_controlPoint.resize(1);
        m_controlPoint[0] = cp[0];

        m_controlPoint[0]->m_state = s;
    }

    ConstraintElement(State * s,ControlPoint<2> cp,ConstraintProximityPtr c)
    : m_centerPoint(c) {
        m_controlPoint.resize(2);
        m_controlPoint[0] = cp[0];
        m_controlPoint[1] = cp[1];

        m_controlPoint[0]->m_state = s;
        m_controlPoint[1]->m_state = s;
    }

    ConstraintElement(State * s,ControlPoint<3> cp,ConstraintProximityPtr c)
    : m_centerPoint(c) {
        m_controlPoint.resize(3);
        m_controlPoint[0] = cp[0];
        m_controlPoint[1] = cp[1];
        m_controlPoint[2] = cp[2];

        m_controlPoint[0]->m_state = s;
        m_controlPoint[1]->m_state = s;
        m_controlPoint[2]->m_state = s;
    }

    //this function returns a vector with all the control points of the element
    //if an id is not >=0 and <getNbControlPoints() this function should return the gravity center of the element
    ConstraintProximityPtr getControlPoint(const int cid = -1) {
        if (cid >= 0 && cid<m_controlPoint.size()) return m_controlPoint[cid];
        return m_centerPoint;
    }


    //this function project the point P on the element and return the corresponding proximity
    virtual ConstraintProximityPtr project(Vector3 P) = 0;

    // return the number of control points
    unsigned getNbControlPoints() {
        return m_controlPoint.size();
    }

    virtual void draw(const VisualParams *vparams) = 0;

protected:

    inline Eigen::MatrixXd pinv(const Eigen::MatrixXd & m) {
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

    virtual ConstraintProximityPtr newton_project(Vector3 Q) {
        const int maxIt = 1;
        const double tolerance = 0.0001;
//        const double threshold = 0.0000001;
        double delta = 0.001;

        std::vector<ConstraintProximityPtr> controlPoints;
        for (unsigned i=0;i<m_controlPoint.size();i++) controlPoints.push_back(getControlPoint(i));

        ConstraintProximityPtr res = getControlPoint();

        if (controlPoints.size() <= 1) return res;

        //check the control points that are necessary
        std::vector<bool> usePoints(controlPoints.size(), true);
        std::vector<double> inc(controlPoints.size(), 0.0);

        int it = 0;

        while (it< maxIt) {
            Vector3 P = res->getPosition();


    //        for (unsigned i=0;i<m_fact.size();i++) {
    //            if (m_fact[i] == 0) usePoints[i] = dot(Q - P,controlPoints[i] - P) > 0;
    //            else usePoints[i] = true;

    //            if (usePoints[i]) normals.push_back((controlPoints[i] - P)*delta);
    //        }

            unsigned JLin = (unsigned) controlPoints.size();

    //        if (JLin == 0) break;

            Eigen::VectorXd e0(JLin);
            Eigen::MatrixXd J = Eigen::MatrixXd::Zero(JLin,JLin);

            Vector3 PQ = Q-P;

            std::vector<Vector3> normals;

            double err=0.0;
            for (unsigned j=0;j<JLin;j++) {
    //            helper::vector<double> inc(controlPoints.size(), 0.0);
    //            inc[j] = delta;
    //            ConstraintProximityPtr sigma = move(inc.ptr());
                normals.push_back(controlPoints[j]->getPosition() - P);
                double e = dot(PQ,normals[j]);
                e0(j) = e;
                err += e*e;
            }

            if (sqrt(err)<tolerance) break;

            for (unsigned j=0;j<JLin;j++) {
                const Vector3 R = P + normals[j] * delta;
                for (unsigned i=0;i<JLin;i++) {
                    const double fxdx = dot(R, normals[i]);
                    J(i,j) = (fxdx - e0(i))/ delta;
                }
            }

            std::cout << "e0=\n" << e0 << std::endl;
            std::cout << "J=\n" << J << std::endl;

            Eigen::MatrixXd invJ = pinv(J);
            Eigen::VectorXd dx = -invJ * e0;

            std::cout << "dx=\n" << dx << std::endl;

//            res->moveToControlPoints(dx.data());

    //        helper::vector<double> Dx;

    //        int k=0;
    //        for (unsigned i=0;i<usePoints.size();i++) {
    //            if (usePoints[i]) Dx.push_back(dx(k++));
    //            else Dx.push_back(0.0);
    //        }

    //        helper::vector<double> prev = m_fact;

            it++;
    //        this->inc(Dx);


    //        double res = 0.0;
    //        for (unsigned i=0;i<m_fact.size();i++) res += std::pow((prev[i] - m_fact[i]),2);
    //        if (sqrt(res) < threshold) break;

    //                std::cout << "DX(" << sqrt(res) << ")=" << Dx << "    |||||| " << normals << std::endl;
        }

        return res;


    //            double sum = 0.0;
    //            for (unsigned i=0;i<m_fact.size();i++) sum += m_fact[i];
    //            std::cout << "NEWTON ITERATIONS " << it << " sum=" << sum << " fact=" << m_fact << std::endl;
    }

protected:
    std::vector<ConstraintProximityPtr> m_controlPoint;
    ConstraintProximityPtr m_centerPoint;
};

typedef ConstraintElement::ConstraintElementPtr ConstraintElementPtr;

} // namespace controller
