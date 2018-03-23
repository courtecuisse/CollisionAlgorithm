#pragma once

#include <BaseGeometry.h>

namespace collisionAlgorithm {

class AABBGeometry : public BaseGeometry {
    friend class AABBElement;

public:
    Data<TVec3i> d_nbox;
    PortOut<BaseGeometry,REQUIRED> d_geometry;

    AABBGeometry();

    void init();

    void prepareDetection();

    void draw(const VisualParams * vparams);

    virtual unsigned getNbElements() const;

    virtual ConstraintElementPtr getElement(unsigned eid) const;

protected:
    Vector3 m_Bmin,m_Bmax,m_cellSize;
    std::vector<std::vector<std::vector<std::vector<unsigned> > > >  m_indexedElement;
    BaseGeometry * m_geo;

};

}
