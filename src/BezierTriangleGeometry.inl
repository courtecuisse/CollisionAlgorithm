#ifndef SOFA_COMPONENT_BEZIERTRIANGLEGEOMETRY_INL
#define SOFA_COMPONENT_BEZIERTRIANGLEGEOMETRY_INL

#include "BezierTriangleGeometry.h"
#include "ConstraintProximity.h"

namespace sofa {

namespace core {

namespace behavior {

BezierTriangleGeometry::BezierTriangleGeometry()
: Inherit()
{}

void BezierTriangleGeometry::prepareDetection() {
    Inherit::prepareDetection();

    const helper::ReadAccessor<Data <VecCoord> >& x = *this->getMstate()->read(core::VecCoordId::position());

    m_beziertriangle_info.resize(this->getTopology()->getNbTriangles());
    for (int t=0;t<this->getTopology()->getNbTriangles();t++) {
        BezierTriangleInfo & tbinfo = this->m_beziertriangle_info[t];
        const sofa::core::topology::BaseMeshTopology::Triangle trpids = this->getTopology()->getTriangle(t);

        const Vector3 & p300 = x[trpids[2]];
        const Vector3 & p030 = x[trpids[1]];
        const Vector3 & p003 = x[trpids[0]];

        const Vector3 & n200 = this->m_pointNormal[trpids[2]];
        const Vector3 & n020 = this->m_pointNormal[trpids[1]];
        const Vector3 & n002 = this->m_pointNormal[trpids[0]];

        double w12 = dot(p030 - p300,n200);
        double w21 = dot(p300 - p030,n020);
        double w23 = dot(p003 - p030,n020);
        double w32 = dot(p030 - p003,n002);
        double w31 = dot(p300 - p003,n002);
        double w13 = dot(p003 - p300,n200);

        tbinfo.p210 = (p300*2.0 + p030 - n200 * w12) / 3.0;
        tbinfo.p120 = (p030*2.0 + p300 - n020 * w21) / 3.0;

        tbinfo.p021 = (p030*2.0 + p003 - n020 * w23) / 3.0;
        tbinfo.p012 = (p003*2.0 + p030 - n002 * w32) / 3.0;

        tbinfo.p102 = (p003*2.0 + p300 - n002 * w31) / 3.0;
        tbinfo.p201 = (p300*2.0 + p003 - n200 * w13) / 3.0;

        Vector3 E = (tbinfo.p210+tbinfo.p120+tbinfo.p102+tbinfo.p201+tbinfo.p021+tbinfo.p012) / 6.0;
        Vector3 V = (p300+p030+p003) / 3.0;
        tbinfo.p111 =  E + (E-V) / 2.0;

        //Compute Bezier Normals
        double v12 = 2 * dot(p030-p300,n200+n020) / dot(p030-p300,p030-p300);
        double v23 = 2 * dot(p003-p030,n020+n002) / dot(p003-p030,p003-p030);
        double v31 = 2 * dot(p300-p003,n002+n200) / dot(p300-p003,p300-p003);

        Vector3 h110 = n200 + n020 - (p030-p300) * v12;
        Vector3 h011 = n020 + n002 - (p003-p030) * v23;
        Vector3 h101 = n002 + n200 - (p300-p003) * v31;

        tbinfo.n110 = h110 / h110.norm();
        tbinfo.n011 = h011 / h011.norm();
        tbinfo.n101 = h101 / h101.norm();
    }
}

ConstraintProximityPtr BezierTriangleGeometry::getNonLinearTriangleProximity(unsigned eid, unsigned p1,double f1,unsigned p2, double f2, unsigned p3, double f3) const {
    return ConstraintProximityPtr (new BezierConstraintProximity(this, eid, p1,f1,p2,f2,p3,f3));
}

} //behavior

} //core

}//sofa

#endif
