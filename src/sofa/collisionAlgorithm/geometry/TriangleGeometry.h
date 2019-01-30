#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class GEOMETRY>
class TriangleElement : public BaseElement {
public:
    typedef GEOMETRY TGeometry;
    typedef typename GEOMETRY::TDataTypes DataTypes;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;

    TriangleElement(unsigned id,const GEOMETRY * geo) : m_tid(id), m_geo(geo) {}

    inline BaseProximity::SPtr project(const defaulttype::Vector3 & P) const {
        core::topology::BaseMeshTopology::Triangle triangle;
        defaulttype::Vector3 factor;
        m_geo->project(m_tid, P, triangle, factor);

        return BaseProximity::create<TriangleProximity<GEOMETRY> >(m_geo, m_tid,triangle[0],triangle[1],triangle[2],factor[0],factor[1],factor[2]);
    }

    inline BaseProximity::SPtr center() const {
        const core::topology::BaseMeshTopology::Triangle & triangle = m_geo->getTriangles()[m_tid];
        return BaseProximity::create<TriangleProximity<GEOMETRY> >(m_geo, m_tid,triangle[0],triangle[1],triangle[2],0.3333,0.3333,0.3333);
    }

    inline defaulttype::BoundingBox getBBox() const {
        const core::topology::BaseMeshTopology::Triangle & triangle = m_geo->getTriangles()[m_tid];
        const helper::ReadAccessor<Data <VecCoord> >& x = m_geo->getState()->read(core::VecCoordId::position());
        defaulttype::BoundingBox bbox;
        bbox.include(x[triangle[0]]);
        bbox.include(x[triangle[1]]);
        bbox.include(x[triangle[2]]);
        return bbox;
    }

protected:
    unsigned m_tid;
    const GEOMETRY * m_geo;
};


template<class DataTypes>
class TriangleGeometry : public TBaseGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef TriangleGeometry<DataTypes> GEOMETRY;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;
    typedef sofa::core::topology::BaseMeshTopology::Triangle Triangle;
    typedef size_t TriangleID; // to remove once TriangleID has been changed to size_t in BaseMeshTopology
    typedef helper::vector<Triangle> VecTriangles;

    SOFA_CLASS(GEOMETRY,Inherit);

    Data<VecTriangles> d_triangles;

    TriangleGeometry()
    : d_triangles(initData(&d_triangles, "triangles", "Vector of Triangles")){}

    virtual ~TriangleGeometry() override {}

    virtual BaseElementIterator::UPtr begin(unsigned eid = 0) const {
        return DefaultElementIterator<TriangleElement<GEOMETRY> >::create(this, this->d_triangles.getValue().size(), eid);
    }

    void init() {
        ///To remove if we think every input has to be explicit
        if(d_triangles.getValue().empty())
        {
            msg_warning(this) << "Triangles are not set (data is empty). Will set from topology if present in the same context";
            sofa::core::topology::BaseMeshTopology* topology{nullptr};
            this->getContext()->get(topology);
            if(!topology)
            {
                msg_error(this) << "No topology to work with ; giving up.";
            }
            else
            {
                if(topology->getTriangles().empty())
                {
                    msg_error(this) << "No topology with triangles to work with ; giving up.";
                }
                else
                {
                    d_triangles.setParent(topology->findData("triangles"));
                }
            }
        }
    }

    virtual void draw(const core::visual::VisualParams * vparams) {
        Inherit::draw(vparams);

        if (! vparams->displayFlags().getShowCollisionModels())
            return;

        if (this->d_color.getValue()[3] == 0.0)
            return;

        glDisable(GL_LIGHTING);

        double delta = 0.2;
        defaulttype::Vector4 color = this->d_color.getValue();
        const helper::ReadAccessor<DataVecCoord> & pos = this->l_state->read(core::VecCoordId::position());

        if (! vparams->displayFlags().getShowWireFrame()) glBegin(GL_TRIANGLES);
        else glBegin(GL_LINES);

        for (unsigned i=0;i<d_triangles.getValue().size();i++) {
            const Triangle& tri = this->d_triangles.getValue()[i];

            glColor4f(fabs(color[0]-delta),color[1],color[2],color[3]);
            glVertex3dv(pos[tri[0]].data());
            if (vparams->displayFlags().getShowWireFrame()) glVertex3dv(pos[tri[1]].data());

            glColor4f(color[0],fabs(color[1]-delta),color[2],color[3]);
            glVertex3dv(pos[tri[1]].data());
            if (vparams->displayFlags().getShowWireFrame()) glVertex3dv(pos[tri[2]].data());

            glColor4f(color[0],color[1],fabs(color[2]-delta),color[3]);
            glVertex3dv(pos[tri[2]].data());            
            if (vparams->displayFlags().getShowWireFrame()) glVertex3dv(pos[tri[0]].data());
        }
        glEnd();
    }

    virtual void prepareDetection() override {
        const VecTriangles& triangles = d_triangles.getValue();

        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());

        m_triangle_info.resize(triangles.size());
        m_triangle_normals.resize(triangles.size());

        for (size_t t=0 ; t<triangles.size() ; t++)
        {
            const Triangle& tri = triangles[t];

            //Compute Bezier Positions
            const defaulttype::Vector3 & p0 = pos[tri[0]];
            const defaulttype::Vector3 & p1 = pos[tri[1]];
            const defaulttype::Vector3 & p2 = pos[tri[2]];

            TriangleInfo & tinfo = m_triangle_info[t];
            tinfo.v0 = p1 - p0;
            tinfo.v1 = p2 - p0;

            tinfo.d00 = dot(tinfo.v0,tinfo.v0);
            tinfo.d01 = dot(tinfo.v0,tinfo.v1);
            tinfo.d11 = dot(tinfo.v1,tinfo.v1);

            tinfo.invDenom = 1.0 / (tinfo.d00 * tinfo.d11 - tinfo.d01 * tinfo.d01);

            tinfo.ax1 = tinfo.v0;
            m_triangle_normals[t] = tinfo.v0.cross(tinfo.v1);
            tinfo.ax2 = tinfo.v0.cross(m_triangle_normals[t]);

            tinfo.ax1.normalize();
            m_triangle_normals[t].normalize();
            tinfo.ax2.normalize();
        }
    }

    inline const VecTriangles & getTriangles() const {
        return d_triangles.getValue();
    }

    inline const helper::vector<defaulttype::Vector3> & triangleNormals() const {
        return m_triangle_normals;
    }

    //Barycentric coordinates are computed according to
    //http://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
    void project(unsigned eid, const defaulttype::Vector3 & P, core::topology::BaseMeshTopology::Triangle & triangle, defaulttype::Vector3 & factor) const {
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());

        const TriangleInfo & tinfo = m_triangle_info[eid];
        triangle = d_triangles.getValue()[eid];

        defaulttype::Vector3 P0 = pos[triangle[0]];
        defaulttype::Vector3 P1 = pos[triangle[1]];
        defaulttype::Vector3 P2 = pos[triangle[2]];

        defaulttype::Vector3 x1x2 = P - P0;

        //corrdinate on the plane
        double c0 = dot(x1x2,tinfo.ax1);
        double c1 = dot(x1x2,tinfo.ax2);
        defaulttype::Vector3 proj_P = P0 + tinfo.ax1 * c0 + tinfo.ax2 * c1;

        double fact_u,fact_v,fact_w;

        computeBaryCoords(proj_P, tinfo, P0, fact_u,fact_v,fact_w);

        if (fact_u<0)
        {
            defaulttype::Vector3 v3 = P1 - P2;
            defaulttype::Vector3 v4 = proj_P - P2;
            double alpha = dot(v4,v3) / dot(v3,v3);

            if (alpha<0) alpha = 0;
            else if (alpha>1) alpha = 1;

            fact_u = 0;
            fact_v = alpha;
            fact_w = 1.0 - alpha;
        }
        else if (fact_v<0)
        {
            defaulttype::Vector3 v3 = P0 - P2;
            defaulttype::Vector3 v4 = proj_P - P2;
            double alpha = dot(v4,v3) / dot(v3,v3);

            if (alpha<0) alpha = 0;
            else if (alpha>1) alpha = 1;

            fact_u = alpha;
            fact_v = 0;
            fact_w = 1.0 - alpha;
        }
        else if (fact_w<0)
        {
            defaulttype::Vector3 v3 = P1 - P0;
            defaulttype::Vector3 v4 = proj_P - P0;
            double alpha = dot(v4,v3) / dot(v3,v3);

            if (alpha<0) alpha = 0;
            else if (alpha>1) alpha = 1;

            fact_u = 1.0 - alpha;
            fact_v = alpha;
            fact_w = 0;
        }

        factor[0] = fact_u;
        factor[1] = fact_v;
        factor[2] = fact_w;
    }

protected:
    typedef struct
    {
        defaulttype::Vector3 v0,v1;
        double d00;
        double d01;
        double d11;
        double invDenom;

        defaulttype::Vector3 ax1,ax2;
    } TriangleInfo;

    //proj_P must be on the plane
    void computeBaryCoords(const defaulttype::Vector3 & proj_P,const TriangleInfo & tinfo, const defaulttype::Vector3 & p0, double & fact_u,double & fact_v, double & fact_w) const
    {
        defaulttype::Vector3 v2 = proj_P - p0;

        double d20 = dot(v2,tinfo.v0);
        double d21 = dot(v2,tinfo.v1);

        fact_v = (tinfo.d11 * d20 - tinfo.d01 * d21) * tinfo.invDenom;
        fact_w = (tinfo.d00 * d21 - tinfo.d01 * d20) * tinfo.invDenom;
        fact_u = 1.0 - fact_v  - fact_w;
    }

    std::vector<TriangleInfo> m_triangle_info;
    helper::vector<defaulttype::Vector3> m_triangle_normals;
};


}

}
