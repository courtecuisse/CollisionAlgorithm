#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/TetrahedronProximity.h>
#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/collisionAlgorithm/toolBox/TetraToolBox.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class TetrahedronGeometry : public TBaseGeometry<DataTypes,TetrahedronProximity> {
public:
    typedef DataTypes TDataTypes;
    typedef TetrahedronGeometry<DataTypes> GEOMETRY;
    typedef TBaseGeometry<DataTypes,TetrahedronProximity> Inherit;
    typedef BaseProximity::Index Index;
    typedef typename Inherit::PROXIMITYDATA PROXIMITYDATA;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;

    typedef size_t TetraID;
    typedef sofa::core::topology::BaseMeshTopology::Tetrahedron Tetrahedron;
    typedef sofa::type::vector<Tetrahedron> VecTetrahedron;

    SOFA_CLASS(GEOMETRY,Inherit);

    core::objectmodel::SingleLink<GEOMETRY,core::topology::BaseMeshTopology,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_topology;

    TetrahedronGeometry()
    : l_topology(initLink("topology", "link to topology")) {
        l_topology.setPath("@.");
    }

    inline BaseElementIterator::UPtr begin(Index eid = 0) const override {
        return DefaultElementIterator<PROXIMITYDATA>::create(this, this->l_topology->getTetrahedra(), eid);
    }

    void draw(const core::visual::VisualParams * vparams) {
        //TO DO
//        this->drawNormals(vparams);

////        if (! vparams->displayFlags().getShowCollisionModels()) return;
//        if (! this->drawCollision.getValue() && ! vparams->displayFlags().getShowCollisionModels()) return ;
//        const sofa::type::RGBAColor & color = this->d_color.getValue();
//        if (color[3] == 0.0) return;

//        glDisable(GL_LIGHTING);

//        double delta = 0.2;
//        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());

//        if (! vparams->displayFlags().getShowWireFrame()) glBegin(GL_TRIANGLES);
//        else glBegin(GL_LINES);

//        for (auto it=this->begin();it != this->end(); it++) {
//            const Triangle& tri = this->l_topology->getTriangle(it->id());

//            glColor4f(fabs(color[0]-delta),color[1],color[2],color[3]);
//            glVertex3dv(pos[tri[0]].data());
//            if (vparams->displayFlags().getShowWireFrame()) glVertex3dv(pos[tri[1]].data());

//            glColor4f(color[0],fabs(color[1]-delta),color[2],color[3]);
//            glVertex3dv(pos[tri[1]].data());
//            if (vparams->displayFlags().getShowWireFrame()) glVertex3dv(pos[tri[2]].data());

//            glColor4f(color[0],color[1],fabs(color[2]-delta),color[3]);
//            glVertex3dv(pos[tri[2]].data());
//            if (vparams->displayFlags().getShowWireFrame()) glVertex3dv(pos[tri[0]].data());
//        }
//        glEnd();
    }

    virtual void prepareDetection() override {
        m_tetra_info.clear();
    }

    inline const sofa::core::topology::BaseMeshTopology::Tetrahedron getTetrahedron(Index eid) const {
        return this->l_topology->getTetrahedron(eid);
    }

    inline type::Vector3 getPosition(const PROXIMITYDATA & data, core::VecCoordId v = core::VecCoordId::position()) const {
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(v);

        return pos[data.m_p0] * data.m_f0 +
               pos[data.m_p1] * data.m_f1 +
               pos[data.m_p2] * data.m_f2 +
               pos[data.m_p3] * data.m_f3;
    }

    virtual type::Vector3 computeNormal(const PROXIMITYDATA & data) const override {
        return type::Vector3(0,0,0);
    }

    PROXIMITYDATA createProximity(Index eid,CONTROL_POINT pid = CONTROL_DEFAULT) const {
        return PROXIMITYDATA::create(eid, getTetrahedron(eid), pid);
    }

    inline PROXIMITYDATA createProximity(Index eid,double & fact_u,double & fact_v, double & fact_w, double & fact_x) {
        const Tetrahedron & tetrahedron = l_topology->getTetrahedron(eid);
        return PROXIMITYDATA(eid, tetrahedron[0], tetrahedron[1], tetrahedron[2], tetrahedron[3], fact_u,fact_v,fact_w,fact_x);
    }

    inline PROXIMITYDATA project(const type::Vector3 & P, Index eid) const {
        auto tetrahedron = getTetrahedron(eid);
        const TetraInfo & tinfo = getTetraInfo()[eid];

        double fact[4];
        toolBox::projectOnTetra( P, tinfo,fact[0],fact[1],fact[2],fact[3]);

        return PROXIMITYDATA(eid, tetrahedron[0], tetrahedron[1], tetrahedron[2], tetrahedron[3], fact[0],fact[1],fact[2],fact[3]);
    }

    //proj_P must be on the plane

    const std::vector<TetraInfo> & getTetraInfo(core::VecCoordId v = core::VecCoordId::position()) const {
        if (m_tetra_info[v.getIndex()].empty()) computeTetraInfo(v);
        return m_tetra_info[v.getIndex()];
    }

protected:
    mutable std::map<int,std::vector<TetraInfo> > m_tetra_info;

    void computeTetraInfo(core::VecCoordId v = core::VecCoordId::position()) const {
        std::vector<TetraInfo> & vecInfo = m_tetra_info[v.getIndex()];
        const VecTetrahedron& tetrahedra = this->l_topology->getTetrahedra();

        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());

        vecInfo.clear();
        for (size_t t=0 ; t<tetrahedra.size() ; t++) {
            const Tetrahedron& tri = tetrahedra[t];

            const sofa::core::topology::BaseMeshTopology::TrianglesInTetrahedron& TIT = this->l_topology->getTrianglesInTetrahedron(t);

            const type::Vector3 & p0 = pos[tri[0]];
            const type::Vector3 & p1 = pos[tri[1]];
            const type::Vector3 & p2 = pos[tri[2]];
            const type::Vector3 & p3 = pos[tri[3]];

            vecInfo.push_back(toolBox::computeTetraInfo(p0,p1,p2,p3));
        }
    }
};


}

}
