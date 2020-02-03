#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/TetrahedronProximity.h>
#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>
#include <sofa/core/topology/BaseMeshTopology.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class TetrahedronGeometry : public TBaseGeometry<DataTypes,TetrahedronProximity> {
public:
    typedef DataTypes TDataTypes;
    typedef TetrahedronGeometry<DataTypes> GEOMETRY;
    typedef TBaseGeometry<DataTypes,TetrahedronProximity> Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;

    typedef size_t TetraID;
    typedef sofa::core::topology::BaseMeshTopology::Tetrahedron Tetrahedron;
    typedef helper::vector<Tetrahedron> VecTetrahedron;

    SOFA_CLASS(GEOMETRY,Inherit);

    core::objectmodel::SingleLink<GEOMETRY,core::topology::BaseMeshTopology,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_topology;

    TetrahedronGeometry()
    : l_topology(initLink("topology", "link to topology")) {
        l_topology.setPath("@.");
    }

    inline BaseElementIterator::UPtr begin(unsigned eid = 0) const override {
        return DefaultElementIterator<TetrahedronProximity>::create(this, this->l_topology->getTetrahedra(), eid);
    }

    void draw(const core::visual::VisualParams * vparams) {
        //TO DO
//        this->drawNormals(vparams);

////        if (! vparams->displayFlags().getShowCollisionModels()) return;
//        if (! this->drawCollision.getValue() && ! vparams->displayFlags().getShowCollisionModels()) return ;
//        const defaulttype::Vector4 & color = this->d_color.getValue();
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

        const VecTetrahedron& tetrahedra = this->l_topology->getTetrahedra();

        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());

        m_tetra_info.resize(tetrahedra.size());

        for (size_t t=0 ; t<tetrahedra.size() ; t++)
        {
            const Tetrahedron& tri = tetrahedra[t];

            const sofa::core::topology::BaseMeshTopology::TrianglesInTetrahedron& TIT = this->l_topology->getTrianglesInTetrahedron(t);

            const defaulttype::Vector3 & p0 = pos[tri[0]];
            const defaulttype::Vector3 & p1 = pos[tri[1]];
            const defaulttype::Vector3 & p2 = pos[tri[2]];
            const defaulttype::Vector3 & p3 = pos[tri[3]];
            defaulttype::Vector3 bary = (p0 + p1 + p2 + p3)/4.0;


            TetraInfo & tinfo = m_tetra_info[t];

            tinfo.ax1 = p1 - p0;
            tinfo.ax2 = p2 - p0;
            tinfo.ax3 = p3 - p0;
            tinfo.p0 = p0;
            tinfo.ax2Cax3 = tinfo.ax2.cross(tinfo.ax3);
            tinfo.V0 = 1.0/6.0 * dot(tinfo.ax1,tinfo.ax2Cax3);

//            tinfo.trianglesInfo.resize(4);

            //Triangle part
//            for(unsigned i=0;i<4;i++)
//            {
//                const defaulttype::Vector3 & P0 = pos[tri[(i+1)%4]];
//                const defaulttype::Vector3 & P1 = pos[tri[(i+2)%4]];
//                const defaulttype::Vector3 & P2 = pos[tri[(i+3)%4]];
//                tinfo.trianglesInfo[i].v0 = P1 - P0;
//                tinfo.trianglesInfo[i].v1 = P2 - P0;


//                defaulttype::Vector3 normal = tinfo.trianglesInfo[i].v0.cross(tinfo.trianglesInfo[i].v1);
//                if(dot((P0 + P1 + P2)/3-bary,normal)<0)
//                {
//                    normal = -normal;
//                    defaulttype::Vector3 temp = tinfo.trianglesInfo[i].v0;
//                    tinfo.trianglesInfo[i].v0 = tinfo.trianglesInfo[i].v1;
//                    tinfo.trianglesInfo[i].v1 = temp;


//                }
//                tinfo.trianglesInfo[i].d00 = dot(tinfo.trianglesInfo[i].v0,tinfo.trianglesInfo[i].v0);
//                tinfo.trianglesInfo[i].d01 = dot(tinfo.trianglesInfo[i].v0,tinfo.trianglesInfo[i].v1);
//                tinfo.trianglesInfo[i].d11 = dot(tinfo.trianglesInfo[i].v1,tinfo.trianglesInfo[i].v1);

//                tinfo.trianglesInfo[i].invDenom = 1.0 /
//                        (tinfo.trianglesInfo[i].d00 * tinfo.trianglesInfo[i].d11
//                         - tinfo.trianglesInfo[i].d01 * tinfo.trianglesInfo[i].d01);

//                tinfo.trianglesInfo[i].ax1 = tinfo.trianglesInfo[i].v0;
//                tinfo.trianglesInfo[i].ax2 = tinfo.trianglesInfo[i].v0.cross(normal);

//                tinfo.trianglesInfo[i].ax1.normalize();
//                tinfo.trianglesInfo[i].ax2.normalize();
//            }

        }
    }

    inline const sofa::core::topology::BaseMeshTopology::Tetrahedron getTetrahedron(unsigned eid) const {
        return this->l_topology->getTetrahedra(eid);
    }

    inline defaulttype::Vector3 getPosition(const TetrahedronProximity & data, core::VecCoordId v = core::VecCoordId::position()) const {
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(v);

        return pos[data.m_p0] * data.m_f0 +
               pos[data.m_p1] * data.m_f1 +
               pos[data.m_p2] * data.m_f2 +
               pos[data.m_p3] * data.m_f3;
    }

    inline defaulttype::Vector3 getNormal(const TetrahedronProximity & data) const {
        return defaulttype::Vector3(0,0,0);
    }

    TetrahedronProximity center(unsigned eid,const Tetrahedron & tetrahedron) const {
        return TetrahedronProximity(eid, tetrahedron[0], tetrahedron[1], tetrahedron[2], tetrahedron[3], 0.25, 0.25, 0.25, 0.25);
    }

    defaulttype::BoundingBox getBBox(const Tetrahedron & tetrahedron) const {
        const helper::ReadAccessor<Data <VecCoord> >& x = this->getState()->read(core::VecCoordId::position());

        defaulttype::BoundingBox bbox;
        bbox.include(x[tetrahedron[0]]);
        bbox.include(x[tetrahedron[1]]);
        bbox.include(x[tetrahedron[2]]);
        bbox.include(x[tetrahedron[3]]);
        return bbox;
    }

    inline TetrahedronProximity createProximity(unsigned eid,double & fact_u,double & fact_v, double & fact_w, double & fact_x) {
        const Tetrahedron & tetrahedron = l_topology->getTetrahedron(eid);
        return TetrahedronProximity(eid, tetrahedron[0], tetrahedron[1], tetrahedron[2], tetrahedron[3], fact_u,fact_v,fact_w,fact_x);
    }

    inline TetrahedronProximity project(unsigned eid, const Tetrahedron & tetrahedron, const defaulttype::Vector3 & P) const {
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());

        const TetraInfo & tinfo = m_tetra_info[eid];

        double fact[4];

        computeBaryCoords(P, tinfo, fact[0],fact[1],fact[2],fact[3]);

        //Verify if we are out of the tetra
        double acc = 0;

        for(unsigned i = 0; i<4; i++)
        {
            fact[i] = (fact[i]>=0)*fact[i];
            acc += fact[i];
        }
        for(unsigned i = 0; i<4; i++)
        {
            fact[i] /= acc;
        }

        return TetrahedronProximity(eid, tetrahedron[0], tetrahedron[1], tetrahedron[2], tetrahedron[3], fact[0],fact[1],fact[2],fact[3]);
    }

    typedef struct
    {
        double V0;
        defaulttype::Vector3 p0;
        defaulttype::Vector3 ax1,ax2,ax3,ax2Cax3;
//        std::vector<typename TriangleGeometry<DataTypes>::TriangleInfo> trianglesInfo ;

    } TetraInfo;

    void computeBaryCoords(const defaulttype::Vector3 & P,unsigned eid, double & fact_u,double & fact_v, double & fact_w, double & fact_x)
    {
        computeBaryCoords(P,m_tetra_info[eid],fact_u, fact_v,  fact_w,  fact_x);
    }

    //proj_P must be on the plane
    static void computeBaryCoords(const defaulttype::Vector3 & P,const TetraInfo & tinfo, double & fact_u,double & fact_v, double & fact_w, double & fact_x)
    {
        defaulttype::Vector3 e = P - tinfo.p0;

        double Va = 1.0/6.0 * dot(e,tinfo.ax2Cax3);
        double Vb = 1.0/6.0 * dot(tinfo.ax1,e.cross(tinfo.ax3));
        double Vc = 1.0/6.0 * dot(tinfo.ax1,tinfo.ax2.cross(e));

        fact_v = Va/tinfo.V0;
        fact_w = Vb/tinfo.V0;
        fact_x = Vc/tinfo.V0;
        fact_u = 1.0 - (fact_v + fact_w + fact_x);

    }

protected:

    std::vector<TetraInfo> m_tetra_info;

};


}

}
