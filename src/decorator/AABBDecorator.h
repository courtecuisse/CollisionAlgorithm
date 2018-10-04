#pragma once

#include <BaseGeometry.h>
#include <sofa/helper/AdvancedTimer.h>

namespace sofa {

namespace collisionAlgorithm {

class AABBDecorator : public sofa::core::objectmodel::BaseObject {
    friend class AABBElement;

public:
    typedef Data<helper::vector<defaulttype::Vector3> > DataVecCoord;

    Data<defaulttype::Vec3i> d_nbox;
    DataLink<BaseGeometry> d_geometry;


    AABBDecorator();

    ~AABBDecorator(){}

    void prepareDetection();

    void draw(const core::visual::VisualParams * /*vparams*/);

    std::set<unsigned> & getIndexedElements(int i,int j,int k) {
        return m_indexedElement[i*m_offset[0] + j * m_offset[1] + k];
    }

    virtual void handleEvent(core::objectmodel::Event * e) {
        if (dynamic_cast<simulation::AnimateBeginEvent *>(e)) {
            sofa::helper::AdvancedTimer::stepBegin("AABBDecorator");
            prepareDetection();
            sofa::helper::AdvancedTimer::stepEnd("AABBDecorator");
        }
    }

    template<class T>
    static bool canCreate(T*& /*obj*/, core::objectmodel::BaseContext* /*context*/, core::objectmodel::BaseObjectDescription* /*arg*/) {
        return true;
    }

    defaulttype::Vector3 m_Bmin,m_Bmax,m_cellSize;
    defaulttype::Vec3i m_nbox;
    defaulttype::Vec2i m_offset;
    std::vector<std::set<unsigned> >  m_indexedElement;
};

}

}
