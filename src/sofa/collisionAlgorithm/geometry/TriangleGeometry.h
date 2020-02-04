#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>

namespace sofa {

namespace collisionAlgorithm {




template<class DataTypes>
class TriangleGeometry : public TBaseGeometry<DataTypes,TriangleProximity> {
public:
    typedef DataTypes TDataTypes;
    typedef TriangleGeometry<DataTypes> GEOMETRY;
    typedef TBaseGeometry<DataTypes,TriangleProximity> Inherit;
    typedef typename Inherit::PROXIMITYDATA PROXIMITYDATA;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;

    typedef size_t TriangleID;
    typedef sofa::core::topology::BaseMeshTopology::Triangle Triangle;
    typedef helper::vector<Triangle> VecTriangles;

    SOFA_CLASS(GEOMETRY,Inherit);

    core::objectmodel::SingleLink<GEOMETRY,core::topology::BaseMeshTopology,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_topology;

    TriangleGeometry()
        : l_topology(initLink("topology", "link to topology")) {
        l_topology.setPath("@.");
    }

    inline BaseElementIterator::UPtr begin(unsigned eid = 0) const override {
        return DefaultElementIterator<PROXIMITYDATA>::create(this, this->l_topology->getTriangles(), eid);
    }

    void draw(const core::visual::VisualParams * vparams) {
        this->drawNormals(vparams);

        //        if (! vparams->displayFlags().getShowCollisionModels()) return;
        if (! vparams->displayFlags().getShowCollisionModels()) return ;
        const defaulttype::Vector4 & color = this->d_color.getValue();
        if (color[3] == 0.0) return;

        glDisable(GL_LIGHTING);

        double delta = 0.2;
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());

        if (! vparams->displayFlags().getShowWireFrame()) glBegin(GL_TRIANGLES);
        else glBegin(GL_LINES);

        for (auto it=this->begin();it != this->end(); it++) {
            const Triangle& tri = this->l_topology->getTriangle(it->id());

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
        const VecTriangles& triangles = this->l_topology->getTriangles();

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


            tinfo = toolBox::computeTriangleInfo(p0, p1, p2);

            m_triangle_normals[t] = tinfo.ax2.cross(tinfo.ax1);

        }
    }

    inline const sofa::core::topology::BaseMeshTopology::Triangle getTriangle(unsigned eid) const {
        return this->l_topology->getTriangle(eid);
    }

    ////Bezier triangle are computed according to :
    ////http://www.gamasutra.com/view/feature/131389/b%C3%A9zier_triangles_and_npatches.php?print=1
    inline defaulttype::Vector3 getPosition(const PROXIMITYDATA & data, core::VecCoordId v = core::VecCoordId::position()) const {
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(v);

        return pos[data.m_p0] * data.m_f0 +
                pos[data.m_p1] * data.m_f1 +
                pos[data.m_p2] * data.m_f2;
    }

    PROXIMITYDATA center(unsigned eid,const Triangle & triangle) const {
        return PROXIMITYDATA(eid, triangle[0], triangle[1], triangle[2], 0.3333, 0.3333, 0.3333);
    }

    defaulttype::BoundingBox getBBox(const Triangle & triangle) const {
        const helper::ReadAccessor<Data <VecCoord> >& x = this->getState()->read(core::VecCoordId::position());

        defaulttype::BoundingBox bbox;
        bbox.include(x[triangle[0]]);
        bbox.include(x[triangle[1]]);
        bbox.include(x[triangle[2]]);
        return bbox;
    }

    //Barycentric coordinates are computed according to
    //http://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
    inline PROXIMITYDATA project(unsigned eid, const Triangle & triangle, const defaulttype::Vector3 & P) const {
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());

        TriangleInfo  tinfo = m_triangle_info[eid];

        defaulttype::Vector3 P0 = pos[triangle[0]];
        defaulttype::Vector3 P1 = pos[triangle[1]];
        defaulttype::Vector3 P2 = pos[triangle[2]];

        double fact_u,fact_v,fact_w;

        toolBox::projectOnTriangle(P,P0,P1,P2,tinfo,fact_u,fact_v,fact_w);

        return PROXIMITYDATA(eid, triangle[0], triangle[1], triangle[2],fact_u,fact_v,fact_w);

    }


protected:

    std::vector<TriangleInfo> m_triangle_info;
    helper::vector<defaulttype::Vector3> m_triangle_normals;

};


}

}
