///******************************************************************************
//*             Private SOFA components, (c) 2012 INRIA                         *
//* CONFIDENTIAL SOURCE CODE. This file is the property of INRIA and should not *
//* be redistributed. Commercial use is prohibited without a specific license.  *
//******************************************************************************/
//#ifndef SOFA_COMPONENT_INTERSECTIONCONTOURGEOMETRY_INL
//#define SOFA_COMPONENT_INTERSECTIONCONTOURGEOMETRY_INL

//#include "IntersectionContourGeometry.h"
//#include <sofa/core/visual/VisualParams.h>
//#include <sofa/core/behavior/Constraint.inl>

//#include <sofa/defaulttype/Vec.h>
//#include <sofa/helper/gl/template.h>
//#include <sofa/defaulttype/SolidTypes.h>

//#include <SofaConstraint/BilateralInteractionConstraint.h>
//#include "ConstraintProximity.h"

//namespace sofa {

//namespace core {

//namespace behavior {

//IntersectionContourGeometry::IntersectionContourGeometry()
//: d_planePos(initData(&d_planePos, "planePos","Plane Position"))
//{
//    this->f_listening.setValue(true);
//}

////void IntersectionContourGeometry::init() {
//////    this->getContext()->get(m_tiranglegeometry);
//////    if (m_tiranglegeometry == NULL) serr << "Error " << this->getName() << " cannot find the geometry" << std::endl;
////}

//ConstraintProximityPtr IntersectionContourGeometry::getPointProximity(unsigned iid) const {
//    return ConstraintProximityPtr(new IntersectionContourProximity(this, m_intersection[iid].p1,m_intersection[iid].alpha,m_intersection[iid].p2,1.0 - m_intersection[iid].alpha));
//}

//int IntersectionContourGeometry::getNbElements() const {
//    return m_intersection.size();
//}

//void IntersectionContourGeometry::prepareDetection() {
//    m_intersection.clear();
//    helper::ReadAccessor<Data <VecCoord> > x = *this->getMstate()->read(core::VecCoordId::position());

//    defaulttype::Vector3 Z(0,0,1);
//    Vector3 pointOnPlane = d_planePos.getValue().getCenter();
//    Vector3 planeNormal = d_planePos.getValue().getOrientation().rotate(Z);

//    double d=dot(planeNormal,pointOnPlane);

//    //inspect the plane edge intersection
//    for(int i=0;i<this->getTopology()->getNbEdges();i++) {
//        const core::topology::BaseMeshTopology::Edge &e = this->getTopology()->getEdge(i);

//        Vector3 p1 = x[e[0]];
//        Vector3 p2 = x[e[1]];

//        double alpha=1.0 - (d-dot(planeNormal,p1))/(dot(planeNormal,p2-p1));

//        if (alpha>=0 && alpha<=1) m_intersection.push_back(IntersectionEdgeInfo(e[0],e[1],alpha));
//    }
//}

//} // namespace helpMeSee

//} // namespace component

//} // namespace sofa

//#endif // SOFA_COMPONENT_HELPMESEEPLUGIN_IntersectionContourGeometry_INL
