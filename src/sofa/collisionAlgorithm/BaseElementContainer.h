#pragma once

#include <sofa/collisionAlgorithm/BaseElementIterator.h>
#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>

#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/BehaviorModel.h>
#include <sofa/core/collision/Pipeline.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <memory>
#include <map>
#include <vector>
#include <qopengl.h>

namespace sofa
{

namespace collisionAlgorithm
{

class BroadPhase;
class BaseGeometry;

class BaseDataElmtContainer {
public:

    // Container sofa : all container of data should inherit from this class
    class  ElementOwner {
    public:
        void init() {
            for (auto it=m_containers.cbegin();it!=m_containers.cend();it++) {
                (*it)->init();
                (*it)->prepareDetection();
            }
        }

        virtual sofa::core::behavior::BaseMechanicalState * getState() const = 0;

        void prepareDetection() {
            for (auto it=m_containers.cbegin();it!=m_containers.cend();it++) {
                (*it)->prepareDetection();
            }
        }

        void registerContainer(BaseDataElmtContainer*c) {
            m_containers.insert(c);
        }

    protected:
        std::set<BaseDataElmtContainer*> m_containers;
    };

    typedef sofa::core::objectmodel::TClass<BaseDataElmtContainer,sofa::core::objectmodel::BaseData> MyClass;
    static const MyClass* GetClass() { return MyClass::get(); }
    virtual const sofa::core::objectmodel::BaseClass* getClass() const
    { return GetClass(); }

    template<class T>
    static void dynamicCast(T*& ptr, sofa::core::objectmodel::Base* b) {
        ptr = dynamic_cast<T*>(b);
    }

    template<class T>
    static std::string className(const T* ptr= NULL) {
        return sofa::core::objectmodel::BaseClass::defaultClassName(ptr);
    }

    template<class T>
    static std::string namespaceName(const T* ptr= NULL) {
        return sofa::core::objectmodel::BaseClass::defaultNamespaceName(ptr);
    }

    template<class T>
    static std::string templateName(const T* ptr= NULL) {
        return sofa::core::objectmodel::BaseClass::BaseClass::defaultTemplateName(ptr);
    }

    template< class T>
    static std::string shortName( const T* ptr = NULL, sofa::core::objectmodel::BaseObjectDescription* = NULL ) {
        std::string shortname = T::className(ptr);
        if( !shortname.empty() )
        {
            *shortname.begin() = ::tolower(*shortname.begin());
        }
        return shortname;
    }

    virtual sofa::core::objectmodel::Base* getOwner() const = 0;

    virtual sofa::core::objectmodel::BaseData* getData() const = 0;

    void setBroadPhase(BroadPhase * d) {
        m_broadPhase = d;
    }

    void unsetBroadPhase(BroadPhase * d) {
        if (m_broadPhase == d) m_broadPhase = NULL;
    }

    BroadPhase * getBroadPhase() const {
        return m_broadPhase;
    }

    virtual BaseElementIterator::UPtr begin(unsigned eid = 0) const  = 0;

    virtual const BaseGeometry * end() const = 0;

    virtual void init() {}

    virtual void prepareDetection() {}

protected:
    BroadPhase * m_broadPhase;
};

template<class ELMT>
class DataElemntContainer : public core::objectmodel::Data<helper::vector<ELMT> >, public BaseDataElmtContainer {
public:

    explicit DataElemntContainer(const typename core::objectmodel::Data<helper::vector<ELMT> >::InitData& init)
    : Data<helper::vector<ELMT>>(init) {
        if (m_owner = dynamic_cast<ElementOwner*>(init.owner)) m_owner->registerContainer(this);
        m_broadPhase = NULL;
    }

    virtual sofa::core::objectmodel::Base* getOwner() const {
        return core::objectmodel::Data<helper::vector<ELMT> >::getOwner();
    }

    virtual sofa::core::objectmodel::BaseData* getData() const {
        return core::objectmodel::Data<helper::vector<ELMT> >::getData();
    }

    virtual BaseElementIterator::UPtr begin(unsigned eid = 0) const {
        return DefaultElementIterator<DataElemntContainer<ELMT> >::create(this, this->getValue().size(), eid);
    }

    virtual const BaseGeometry * end() const {
        return dynamic_cast<BaseGeometry*>(getOwner());
    }

    inline BaseProximity::SPtr project(unsigned pid, const defaulttype::Vector3 & P) const {

    }

    inline BaseProximity::SPtr center(unsigned pid) const {

    }

    inline defaulttype::BoundingBox getBBox(unsigned pid) const {
        const ELMT & elmt = this->getValue()[pid];

        defaulttype::BoundingBox bbox;
        for (unsigned i=0;i<ELMT::size();i++) {
            defaulttype::Vector3 pi(m_owner->getState()->getPX(elmt[i]),
                                    m_owner->getState()->getPY(elmt[i]),
                                    m_owner->getState()->getPZ(elmt[i]));
            bbox.include(pi);
        }

        return bbox;
    }

protected:
    ElementOwner* m_owner;



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

namespace core {

namespace objectmodel {

template<>
class LinkTraitsPtrCasts<collisionAlgorithm::BaseDataElmtContainer>
{
public:
    static sofa::core::objectmodel::Base* getBase(collisionAlgorithm::BaseDataElmtContainer* n) {
        if (!n) return NULL;
        return n->getOwner();
    }

    static sofa::core::objectmodel::BaseData* getData(collisionAlgorithm::BaseDataElmtContainer* n) {
        if (!n) return NULL;
        return n->getData();
    }
};

}

}

}

