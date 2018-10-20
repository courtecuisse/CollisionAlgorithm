#pragma once

#include <geometry/TriangleGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

class TriangleElement : public ConstraintElement {
    friend class TriangleProximity;
    friend class TriangleGeometry;

public:
    TriangleElement(TriangleGeometry * geo,unsigned eid) : ConstraintElement(geo,3) {
        m_eid = eid;

        const std::vector<core::topology::BaseMeshTopology::Triangle> & triangles = geometry()->d_topology->getTriangles();

        m_pid[0] = triangles[eid][0];
        m_pid[1] = triangles[eid][1];
        m_pid[2] = triangles[eid][2];
    }

    static ConstraintElement::UPtr createElement(TriangleGeometry * geo,unsigned eid) {
        return std::unique_ptr<TriangleElement>(new TriangleElement(geo,eid));
    }

    ConstraintProximity::SPtr getControlPoint(int cid) const {
        if (cid == 0) return TriangleGeometry::createProximity(this,1,0,0);
        else if (cid == 1) return TriangleGeometry::createProximity(this,0,1,0);
        else if (cid == 2) return TriangleGeometry::createProximity(this,0,0,1);
        return TriangleGeometry::createProximity(this,1.0/3.0,1.0/3.0,1.0/3.0);
    }

    //proj_P must be on the plane
    void computeBaryCoords(const defaulttype::Vector3 & proj_P,const TriangleGeometry::TriangleInfo & tinfo, const defaulttype::Vector3 & p0, double & fact_u,double & fact_v, double & fact_w) const {
        defaulttype::Vector3 v2 = proj_P - p0;

        double d20 = dot(v2,tinfo.v0);
        double d21 = dot(v2,tinfo.v1);

        fact_v = (tinfo.d11 * d20 - tinfo.d01 * d21) * tinfo.invDenom;
        fact_w = (tinfo.d00 * d21 - tinfo.d01 * d20) * tinfo.invDenom;
        fact_u = 1.0 - fact_v  - fact_w;
    }

    //Barycentric coordinates are computed according to
    //http://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates

    ConstraintProximity::SPtr project(defaulttype::Vector3 P) const {
        const helper::ReadAccessor<DataVecCoord> & pos = geometry()->getState()->read(core::VecCoordId::position());

        defaulttype::Vector3 P0 = pos[m_pid[0]];
        defaulttype::Vector3 P1 = pos[m_pid[1]];
        defaulttype::Vector3 P2 = pos[m_pid[2]];

        defaulttype::Vector3 x1x2 = P - P0;

        const TriangleGeometry::TriangleInfo & tinfo = geometry()->m_triangle_info[m_eid];

        //corrdinate on the plane
        double c0 = dot(x1x2,tinfo.ax1);
        double c1 = dot(x1x2,tinfo.ax2);
        defaulttype::Vector3 proj_P = P0 + tinfo.ax1 * c0 + tinfo.ax2 * c1;

        double fact_u,fact_v,fact_w;

        computeBaryCoords(proj_P, tinfo, P0, fact_u,fact_v,fact_w);

        if (fact_u<0) {
            defaulttype::Vector3 v3 = P1 - P2;
            defaulttype::Vector3 v4 = proj_P - P2;
            double alpha = dot(v4,v3) / dot(v3,v3);

            if (alpha<0) alpha = 0;
            else if (alpha>1) alpha = 1;

            fact_u = 0;
            fact_v = alpha;
            fact_w = 1.0 - alpha;
        } else if (fact_v<0) {
            defaulttype::Vector3 v3 = P0 - P2;
            defaulttype::Vector3 v4 = proj_P - P2;
            double alpha = dot(v4,v3) / dot(v3,v3);

            if (alpha<0) alpha = 0;
            else if (alpha>1) alpha = 1;

            fact_u = alpha;
            fact_v = 0;
            fact_w = 1.0 - alpha;
        } else if (fact_w<0) {
            defaulttype::Vector3 v3 = P1 - P0;
            defaulttype::Vector3 v4 = proj_P - P0;
            double alpha = dot(v4,v3) / dot(v3,v3);

            if (alpha<0) alpha = 0;
            else if (alpha>1) alpha = 1;

            fact_u = 1.0 - alpha;
            fact_v = alpha;
            fact_w = 0;
        }

        return TriangleGeometry::createProximity(this,fact_u,fact_v,fact_w);
    }

    inline TriangleGeometry * geometry() const {
        return (TriangleGeometry*) m_geometry;
    }

    void drawTriangle(const core::visual::VisualParams * /*vparams*/,const defaulttype::Vector3 & A,const defaulttype::Vector3 & B, const defaulttype::Vector3 & C) const {
        double delta = 0.1;
        defaulttype::Vector4 color = geometry()->d_color.getValue();

        glBegin(GL_TRIANGLES);
            glColor4f(fabs(color[0]-delta),color[1],color[2],color[3]);
            glVertex3dv(A.data());
            glColor4f(color[0],fabs(color[1]-delta),color[2],color[3]);
            glVertex3dv(B.data()); // A<->B
            glColor4f(color[0],color[1],fabs(color[2]-delta),color[3]);
            glVertex3dv(C.data());
        glEnd();
    }

    virtual void draw(const core::visual::VisualParams *vparams) const {
        drawTriangle(vparams,getControlPoint(0)->getPosition(),
                             getControlPoint(1)->getPosition(),
                             getControlPoint(2)->getPosition());
    }

protected:
    unsigned m_pid[3];
    unsigned m_eid;
};

}

}
