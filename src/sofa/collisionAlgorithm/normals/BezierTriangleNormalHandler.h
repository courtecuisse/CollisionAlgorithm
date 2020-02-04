#pragma once

#include <sofa/collisionAlgorithm/BaseNormalHandler.h>

namespace sofa {

namespace collisionAlgorithm {


template<class GEOMETRY>
class BezierTriangleNormalHandler : public BaseNormalHandler<GEOMETRY> {
public:
    typedef typename GEOMETRY::VecCoord VecCoord;
    typedef typename GEOMETRY::DataVecCoord DataVecCoord;
    typedef typename GEOMETRY::PROXIMITYDATA PROXIMITYDATA;
    typedef typename GEOMETRY::VecTriangles VecTriangles;
    typedef BaseNormalHandler<GEOMETRY> Inherit;

    SOFA_CLASS(SOFA_TEMPLATE(BezierTriangleNormalHandler,GEOMETRY), Inherit);

    inline defaulttype::Vector3 getNormal(const TriangleProximity & data) const {
        auto tbinfo = this->l_geometry->getBezierInfo(data.m_eid);

        const defaulttype::Vector3 &n200 = tbinfo.n200;
        const defaulttype::Vector3 &n020 = tbinfo.n020;
        const defaulttype::Vector3 &n002 = tbinfo.n002;

        double fact_w = data.m_f2;
        double fact_u = data.m_f1;
        double fact_v = data.m_f0;

        defaulttype::Vector3 normal = n200 * fact_w*fact_w +
                                      n020 * fact_u*fact_u +
                                      n002 * fact_v*fact_v +
                                      tbinfo.n110 * fact_w*fact_u +
                                      tbinfo.n011 * fact_u*fact_v +
                                      tbinfo.n101 * fact_w*fact_v;

        return normal.normalized();
    }

};


}

}
