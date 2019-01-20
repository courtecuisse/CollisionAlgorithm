//#pragma once

//#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
//#include <sofa/collisionAlgorithm/proximity/PointProximity.h>
//#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>

//namespace sofa
//{

//namespace collisionAlgorithm
//{

//template<class DataTypes>
//BaseElementIterator::UPtr PointGeometry<DataTypes>::getElementIterator(unsigned eid) const {
//    return BaseElementIterator::UPtr(new DefaultElementIterator<GEOMETRY, PointProximity<GEOMETRY> >(this, this->l_state->getSize(), eid));
//}

//template<class DataTypes>
//void PointGeometry<DataTypes>::draw(const core::visual::VisualParams *vparams) {
//    if (! vparams->displayFlags().getShowCollisionModels())
//        return;

//    if (this->d_color.getValue()[3] == 0.0)
//        return;

//    glDisable(GL_LIGHTING);

////    for(ElementIterator it=elementIterator();!it.end();it.next()) {
////        m_elements[i]->draw(vparams);
////    }
//}

//}

//}
