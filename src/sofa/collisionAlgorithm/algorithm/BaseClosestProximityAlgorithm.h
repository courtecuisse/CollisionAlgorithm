#pragma once

#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/SubsetElementIterator.h>
#include <sofa/collisionAlgorithm/toolBox/ClosestProximityAlgorithm.h>

namespace sofa {

namespace collisionAlgorithm {

class BaseClosestProximityAlgorithm : public BaseAlgorithm {
public:
    SOFA_CLASS(BaseClosestProximityAlgorithm, BaseAlgorithm);

    Data<unsigned> d_iterations;
    Data<double> d_threshold;

    typedef std::function<bool(const collisionAlgorithm::PairDetection & )> FilterMethod;
    typedef std::function<double(const collisionAlgorithm::PairDetection & )> DistanceMethod;
    typedef std::function<BaseProximity::SPtr(const BaseProximity::SPtr & , BaseGeometry *)> SearchMethod;

    BaseClosestProximityAlgorithm ()
    : d_iterations(initData(&d_iterations,(unsigned) 1,"iterations", "Number of reprojections of between pair of elements"))
    , d_threshold(initData(&d_threshold,0.0000001,"threshold", "Threshold for iterations")) {
        m_distanceMethod = std::bind(&BaseClosestProximityAlgorithm::computeDistance,this,std::placeholders::_1);
        m_searchMethod = std::bind(&BaseClosestProximityAlgorithm::findClosestProximity,this,std::placeholders::_1,std::placeholders::_2);
    }

    PairDetection findClosestPoint(const BaseElementIterator *itfrom, BaseGeometry *geo) {
        BaseProximity::SPtr pfrom = itfrom->createProximity();
        type::Vector3 prevPos = pfrom->getPosition();
        BaseProximity::SPtr pdest = m_searchMethod(pfrom,geo);

        if (pfrom == NULL || pdest == NULL) return PairDetection(pfrom,pdest);

        // Internal iterations to reproject on the from element
        // For linear elements (triangles, edges, ...) it = 1 should be sufficient
        // For points (from) it can be set to 0 since the reprojection will not modify the proximity
        // For non linear elements (bezier, ...) few iterations may be necessary
        for (unsigned it=0;it<d_iterations.getValue();it++) {
            pfrom = itfrom->project(pdest->getPosition()); // reproject on pfrom
            if ((prevPos - pfrom->getPosition()).norm() < d_threshold.getValue()) break;

            pfrom = itfrom->createProximity();
            pdest = m_searchMethod(pfrom,geo);

            if (pfrom == NULL || pdest == NULL) return PairDetection(pfrom,pdest);
        }


        return PairDetection(pfrom,pdest);
    }


    PairDetection findClosestPoint(const BaseProximity::SPtr & pfrom, BaseGeometry *geo) {
        BaseProximity::SPtr pdest = m_searchMethod(pfrom,geo);
        return PairDetection(pfrom,pdest);
    }


    void setDistance(DistanceMethod m) {
        m_distanceMethod = m;
    }

    void setSearchMethod(SearchMethod m) {
        m_searchMethod = m;
    }

    inline DistanceMethod getDistanceMethod() const {
        return m_distanceMethod;
    }

    inline FilterMethod getFilterMethod() const {
        return std::bind(&BaseAlgorithm::acceptFilter,this,std::placeholders::_1);
    }

    inline SearchMethod getSearchMethod() const {
        return m_searchMethod;
    }

protected:
    DistanceMethod m_distanceMethod;
    SearchMethod m_searchMethod;


    double computeDistance(const collisionAlgorithm::PairDetection & d) const {
        return (d.first->getPosition()-d.second->getPosition()).norm() ;
    }

    BaseProximity::SPtr findClosestProximity(const BaseProximity::SPtr & pfrom, BaseGeometry *geo) {
        BaseElementIterator::UPtr begin = geo->begin();
        return toolBox::doFindClosestProximityIt(pfrom,begin,
                                                 getFilterMethod(),
                                                 getDistanceMethod());

        return NULL;
    }
};


}

}
