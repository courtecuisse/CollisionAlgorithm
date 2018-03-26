#pragma once

#include <BaseGeometry.h>

namespace collisionAlgorithm {


class AABBGeometry : public BaseGeometry {
    friend class AABBElement;

public:
    Data<Vec3i> d_nbox;
    PortOut<BaseGeometry,REQUIRED> p_geometry;

    AABBGeometry();

    ~AABBGeometry(){}

    void init();

    void prepareDetection();

    void draw(const VisualParams * vparams);

protected:
    Vector3 m_Bmin,m_Bmax,m_cellSize;
    std::vector<std::vector<std::vector<std::vector<unsigned> > > >  m_indexedElement;
};

class AABBElement : public ConstraintElement {
public:
    AABBElement(AABBGeometry *geo);

    ConstraintProximityPtr getControlPoint(const int i);

    ConstraintProximityPtr project(Vector3 /*P*/);

    inline AABBGeometry * geometry() const {
        return (AABBGeometry*) m_geometry;
    }

private:
    void fillElementSet(Vec3i cbox,int d, std::set<int> & selectElements);
};

}
