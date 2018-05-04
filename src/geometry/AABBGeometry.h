#pragma once

#include <BaseGeometry.h>

namespace collisionAlgorithm {


class AABBGeometry : public BaseGeometry {
    friend class AABBElement;

public:
    Data<Vec3i> d_nbox;
    Port<BaseGeometry,REQUIRED> p_geometry;

    AABBGeometry();

    ~AABBGeometry(){}

    void init();

    ConstraintProximityPtr project(Vector3 P);

    void prepareDetection();

    void fillElementSet(Vec3i cbox,int d, std::set<int> & selectElements);

    virtual void handleEvent(Event * e) {
        p_geometry->handleEvent(e);
        BaseGeometry::handleEvent(e);
    }

    std::set<unsigned> & getIndexedElements(int i,int j,int k) {
        return m_indexedElement[i*m_offset[0] + j * m_offset[1] + k];
    }

//    inline ReadAccessor<Vector3> read(VecID v) {
//        return p_geometry->read(v);
//    }

    State * getState() {
        return p_geometry->getState();
    }

protected:
    Vector3 m_Bmin,m_Bmax,m_cellSize;
    Vec3i m_nbox;
    Vec2i m_offset;
    std::vector<std::set<unsigned> >  m_indexedElement;

};

}
