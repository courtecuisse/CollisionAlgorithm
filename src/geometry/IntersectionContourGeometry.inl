/******************************************************************************
*             Private SOFA components, (c) 2012 INRIA                         *
* CONFIDENTIAL SOURCE CODE. This file is the property of INRIA and should not *
* be redistributed. Commercial use is prohibited without a specific license.  *
******************************************************************************/
#ifndef SOFA_COMPONENT_INTERSECTIONCONTOURGEOMETRY_INL
#define SOFA_COMPONENT_INTERSECTIONCONTOURGEOMETRY_INL

#include "IntersectionContourGeometry.h"
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/behavior/Constraint.inl>

#include <sofa/defaulttype/Vec.h>
#include <sofa/helper/gl/template.h>
#include <sofa/defaulttype/SolidTypes.h>

#include <SofaConstraint/BilateralInteractionConstraint.h>

namespace sofa {

namespace core {

namespace behavior {

/**************************************************************************/
/******************************PROXIMITY***********************************/
/**************************************************************************/

class IntersectionProximity : public BaseGeometry::ConstraintProximity {
public :
    IntersectionProximity(const IntersectionContourGeometry * geo,unsigned pid) {
        m_geo = geo;
        m_pid = pid;
    }

    defaulttype::Vector3 getPosition(core::VecCoordId vid) const {
        IntersectionContourGeometry::IntersectionEdgeInfo ie = m_geo->m_intersection[m_pid];
        return m_geo->getPos(ie.p1,vid) * ie.alpha + m_geo->getPos(ie.p2,vid) * (1.0 - ie.alpha);
    }

    void buildConstraintMatrix(const ConstraintParams* /*cParams*/, core::MultiMatrixDerivId cId, unsigned cline,const defaulttype::Vector3 & N) {
        DataMatrixDeriv & c_d = *cId[m_geo->getMstate()].write();
        MatrixDeriv & c = *c_d.beginEdit();
        MatrixDerivRowIterator c_it1 = c.writeLine(cline);
        c_it1.addCol(m_pid,N);
        c_d.endEdit();
    }

    const IntersectionContourGeometry * m_geo;
    unsigned m_pid;
};

/**************************************************************************/
/******************************ELEMENT*************************************/
/**************************************************************************/

class IntersectionElement : public BaseGeometry::ConstraintElement {
public:

    IntersectionElement(const IntersectionContourGeometry * geo,unsigned pid) {
        m_pid = pid;
        m_geo = geo;
    }

    ConstraintProximityPtr getDefaultProximity() {
        return std::make_shared<IntersectionProximity>(m_geo,m_pid);
    }

    //this function returns a vector with all the control points of the element
    helper::vector<ConstraintProximityPtr> getConstrolPoints() {
        helper::vector<ConstraintProximityPtr> res;
        res.push_back(getDefaultProximity());
        return res;
    }

    //this function project the point P on the element and return the corresponding proximity
    virtual ConstraintProximityPtr project(defaulttype::Vector3 /*P*/) {
        return getDefaultProximity();
    }


protected:
    unsigned m_pid;
    const IntersectionContourGeometry * m_geo;
};

/**************************************************************************/
/******************************GEOMETRY************************************/
/**************************************************************************/

IntersectionContourGeometry::IntersectionContourGeometry()
: d_planePos(initData(&d_planePos, "planePos","Plane Position"))
{}

unsigned IntersectionContourGeometry::getNbElements() const {
    return m_intersection.size();
}

ConstraintElementPtr IntersectionContourGeometry::getElement(unsigned i) const {
    return std::make_shared<IntersectionElement>(this,i);
}

void IntersectionContourGeometry::prepareDetection() {
    m_intersection.clear();
    helper::ReadAccessor<Data <VecCoord> > x = *this->getMstate()->read(core::VecCoordId::position());

    defaulttype::Vector3 Z(0,0,1);
    Vector3 pointOnPlane = d_planePos.getValue().getCenter();
    Vector3 planeNormal = d_planePos.getValue().getOrientation().rotate(Z);

    double d=dot(planeNormal,pointOnPlane);

    //inspect the plane edge intersection
    for(int i=0;i<this->getTopology()->getNbEdges();i++) {
        const core::topology::BaseMeshTopology::Edge &e = this->getTopology()->getEdge(i);

        Vector3 p1 = x[e[0]];
        Vector3 p2 = x[e[1]];

        double alpha=1.0 - (d-dot(planeNormal,p1))/(dot(planeNormal,p2-p1));

        if (alpha>=0 && alpha<=1) m_intersection.push_back(IntersectionEdgeInfo(e[0],e[1],alpha));
    }
}

void IntersectionContourGeometry::draw(const core::visual::VisualParams * vparams) {
    if (!vparams->displayFlags().getShowCollisionModels()) return;

    double norm = (this->f_bbox.getValue().maxBBox() - this->f_bbox.getValue().minBBox()).norm();

    glDisable(GL_LIGHTING);
    glColor4f(d_color.getValue()[0],d_color.getValue()[1],d_color.getValue()[2],d_color.getValue()[3]);
    for (unsigned i=0;i<this->getNbElements();i++) {
        vparams->drawTool()->drawSphere(this->getElement(i)->getDefaultProximity()->getPosition(),norm*0.01);
    }
}

} // namespace helpMeSee

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_HELPMESEEPLUGIN_IntersectionContourGeometry_INL
