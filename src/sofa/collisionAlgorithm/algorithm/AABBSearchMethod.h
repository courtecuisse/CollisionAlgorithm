//#pragma once

//#include <sofa/collisionAlgorithm/algorithm/BaseClosestProximityAlgorithm.h>
//#include <sofa/collisionAlgorithm/toolBox/AABBMethods.h>

//namespace sofa {

//namespace collisionAlgorithm {

//class AABBSearchMethod : public sofa::core::objectmodel::BaseObject
//{
//public:
//    SOFA_CLASS(AABBSearchMethod, sofa::core::objectmodel::BaseObject);

//    core::objectmodel::SingleLink<AABBSearchMethod,BaseClosestProximityAlgorithm, BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_algorithm;

//    AABBSearchMethod()
//    : l_algorithm(initLink("algo","Link to the algorithm to optimize")) {
//        l_algorithm.setPath(("@."));
//    }

//    void init() {
//        if (l_algorithm!=NULL) {
//            l_algorithm->setSearchMethod(std::bind(&AABBSearchMethod::findClosestProximity,this,std::placeholders::_1,std::placeholders::_2));
//            l_algorithm->addSlave(this);
//        }
//    }


//    BaseProximity::SPtr findClosestProximity(const BaseProximity::SPtr & pfrom, BaseGeometry *geo) {
//        return toolBox::findClosestProximity(pfrom,geo,l_algorithm->getFilterMethod(),l_algorithm->getDistanceMethod());
//    }

//};


//}

//}
