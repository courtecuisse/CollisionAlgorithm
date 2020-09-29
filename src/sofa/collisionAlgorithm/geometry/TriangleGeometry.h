#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>
#include <sofa/collisionAlgorithm/toolBox/TriangleToolBox.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class TriangleGeometry : public TBaseGeometry<DataTypes,TriangleProximity> {
public:
    typedef DataTypes TDataTypes;
    typedef TriangleGeometry<DataTypes> GEOMETRY;
    typedef TBaseGeometry<DataTypes,TriangleProximity> Inherit;
    typedef BaseProximity::index_type index_type;
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

    inline BaseElementIterator::UPtr begin(index_type eid = 0) const override {
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
        m_triangle_info.clear();
    }

    inline const sofa::core::topology::BaseMeshTopology::Triangle getTriangle(index_type eid) const {
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

    PROXIMITYDATA createProximity(index_type eid, CONTROL_POINT pid = CONTROL_DEFAULT) const {
        return PROXIMITYDATA::create(eid,getTriangle(eid),pid);
    }

    //Barycentric coordinates are computed according to
    //http://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
    inline PROXIMITYDATA project(const defaulttype::Vector3 & P, index_type eid) const {
        TriangleInfo  tinfo = getTriangleInfo()[eid];
        auto triangle = getTriangle(eid);

        double fact_u,fact_v,fact_w;
        toolBox::projectOnTriangle(P,tinfo,fact_u,fact_v,fact_w);

        return PROXIMITYDATA(eid, triangle[0], triangle[1], triangle[2],fact_u,fact_v,fact_w);
    }

    virtual defaulttype::Vector3 computeNormal(const PROXIMITYDATA & data) const override {
        auto tinfo = getTriangleInfo()[data.m_eid];
        return cross(tinfo.ax2,tinfo.ax1);
    }


    inline const std::vector<TriangleInfo> & getTriangleInfo(core::VecCoordId v = core::VecCoordId::position()) const {
        if (m_triangle_info[v.getIndex()].empty()) computeTriangleInfo(v);
        return m_triangle_info[v.getIndex()];
    }


protected:
    mutable std::map<int,std::vector<TriangleInfo> > m_triangle_info;

    void computeTriangleInfo(core::VecCoordId v = core::VecCoordId::position()) const {
        std::vector<TriangleInfo> & vecInfo = m_triangle_info[v.getIndex()];
        const VecTriangles& triangles = this->l_topology->getTriangles();
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(v);

        vecInfo.clear();
        for (size_t t=0 ; t<triangles.size() ; t++) {
            const Triangle& tri = triangles[t];

            //Compute Positions
            const defaulttype::Vector3 & p0 = pos[tri[0]];
            const defaulttype::Vector3 & p1 = pos[tri[1]];
            const defaulttype::Vector3 & p2 = pos[tri[2]];

            vecInfo.push_back(toolBox::computeTriangleInfo(p0, p1, p2));
        }
    }
};


}

}
