#pragma once

#include <sofa/collisionAlgorithm/geometry/PhongTriangleGeometry.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
class BezierTriangleProximity;

template<class DataTypes>
class BezierTriangleGeometry : public PhongTriangleGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef PhongTriangleGeometry<DataTypes> Inherit;
    typedef BezierTriangleGeometry<DataTypes> GEOMETRY;
    typedef sofa::core::topology::BaseMeshTopology::Triangle Triangle;
    typedef typename Inherit::TriangleInfo TriangleInfo;
    typedef helper::vector<Triangle> VecTriangles;
    typedef Data<helper::vector<defaulttype::Vector3> > DataVecCoord;
    typedef sofa::core::behavior::MechanicalState<DataTypes> State;

    friend class BezierTriangleProximity<GEOMETRY>;

    SOFA_CLASS(GEOMETRY,Inherit);

    Data <unsigned> d_nonlin_max_it;
    Data <double> d_nonlin_tolerance;
    Data <double> d_nonlin_threshold;
    Data <unsigned> d_draw_tesselation;

    BezierTriangleGeometry();

    virtual void prepareDetection() override;

    virtual BaseElementIterator::UPtr begin(unsigned eid) const;

    typedef struct
    {
        defaulttype::Vector3 p210,p120,p021,p012,p102,p201,p111;
        defaulttype::Vector3 n110,n011,n101;
    } BezierTriangleInfo;

    std::vector<BezierTriangleInfo> m_beziertriangle_info;

protected:
    virtual void projectBezier(unsigned eid, const defaulttype::Vector3 & P, core::topology::BaseMeshTopology::Triangle & triangle, defaulttype::Vector3 & factor) const;
};

}

}
