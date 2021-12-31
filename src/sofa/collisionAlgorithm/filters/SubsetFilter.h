//#pragma once

//#include <sofa/collisionAlgorithm/BaseAlgorithm.h>

//namespace sofa {

//namespace collisionAlgorithm {

///*!
// * \brief The SubsetFilter class
// * accepts proximities which indices are multiples of a certain number
// */
//class SubsetFilter : public BaseAlgorithm::BaseFilter {
//public:
//    SOFA_ABSTRACT_CLASS(BaseFilter, BaseFilter);

//    Data<int> d_multiple;

//    SubsetFilter()
//            : d_multiple(initData(&d_multiple, 1, "multiple", "Indices must be multiples")) {}

//    bool accept(const BaseProximity::SPtr & p1,const BaseProximity::SPtr & /*p2*/) const {
//        int indice = p1->getElementId();

//        return (indice % d_multiple.getValue() == 0);
//    }
//};

//}

//}
