#pragma once

#include <sofa/helper/AdvancedTimer.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

class AABBDecorator : public core::BehaviorModel {
    friend class AABBElement;
    friend class AABBElementIterator;
public:
    typedef Data<helper::vector<defaulttype::Vector3> > DataVecCoord;

    SOFA_CLASS(AABBDecorator, core::BehaviorModel);

    Data<defaulttype::Vec3i> d_nbox;

    AABBDecorator();

    ~AABBDecorator(){}

    void prepareDetection();

    void updatePosition(SReal ) {
        prepareDetection();
    }

    void init() {
        if (l_geometry != NULL) {
            l_geometry->addSlave(this);
            prepareDetection();
        }
    }

    void draw(const core::visual::VisualParams * /*vparams*/);

    std::set<unsigned> & getIndexedElements(int i,int j,int k) {
        return m_indexedElement[i*m_offset[0] + j * m_offset[1] + k];
    }

protected:
    defaulttype::Vector3 m_Bmin,m_Bmax,m_cellSize;
    defaulttype::Vec3i m_nbox;
    defaulttype::Vec2i m_offset;
    std::vector<std::set<unsigned> >  m_indexedElement;

    core::objectmodel::SingleLink<AABBDecorator,BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;
};

}

}
