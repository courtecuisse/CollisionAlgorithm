#pragma once

#include <BaseGeometry.h>

namespace collisionAlgorithm {


class AABBDecorator : public BaseObject {
    friend class AABBElement;

public:
    Data<Vec3i> d_nbox;
    Port<BaseGeometry,REQUIRED> p_geometry;

    AABBDecorator();

    ~AABBDecorator(){}

    void prepareDetection();

    void draw(const VisualParams * /*vparams*/);

    std::set<unsigned> & getIndexedElements(int i,int j,int k) {
        return m_indexedElement[i*m_offset[0] + j * m_offset[1] + k];
    }

    virtual void handleEvent(Event * e) {
        if (dynamic_cast<AnimateBeginEvent *>(e)) prepareDetection();
    }

    Vector3 m_Bmin,m_Bmax,m_cellSize;
    Vec3i m_nbox;
    Vec2i m_offset;
    std::vector<std::set<unsigned> >  m_indexedElement;
};

}
