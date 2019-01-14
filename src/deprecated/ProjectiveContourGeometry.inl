#pragma once

#include "ProjectiveContourGeometry.h"

namespace collisionAlgorithm {

ProjectiveContourGeometry::ProjectiveContourGeometry()
: d_projectionMatrix("projectionMatrix", Mat3x4(Vector4(1.0,0.0,0.0,0.0),
                                                Vector4(0.0,1.0,0.0,0.0),
                                                Vector4(0.0,0.0,1.0,0.0)),this)
, d_epsilon("epsilon",1e-6, this) {}

void ProjectiveContourGeometry::fillTriangleSet(AABBDecorator * decorator, int d,const Vec3i & cbox,std::set<unsigned> & triangleSet) {
    for (int i=-d;i<=d;i++) {
        if (cbox[0]+i < 0 || cbox[0]+i > decorator->d_nbox.getValue()[0]) continue;

        for (int j=-d;j<=d;j++) {
            if (cbox[1]+j < 0 || cbox[1]+j > decorator->d_nbox.getValue()[1]) continue;

            for (int k=-d;k<=d;k++) {
                if (cbox[2]+k < 0 || cbox[2]+k > decorator->d_nbox.getValue()[2]) continue;

                if (sqrt(i * i + j*j + k*k) > d) continue;

                const helper::vector<unsigned> & triangles = decorator->m_triangleboxes[cbox[0] + i][cbox[1] + j][cbox[2] + k];

                for (unsigned t=0;t<triangles.size();t++) triangleSet.insert(triangles[t]);
            }
        }
    }
}

void ProjectiveContourGeometry::filterTriangleFunctionWithAABB(AABBDecorator * decorator,const unsigned p, const Vector3 & A) {

    bool hasActiveTriangle = false;
    const Topology::TrianglesAroundVertex & tav = this->getTopology()->getTrianglesAroundVertex(p);
    for (unsigned t=0;!hasActiveTriangle && t<tav.size();t++) {
        if (m_filteredTriangle[tav[t]]) hasActiveTriangle = true;
    }

    //if there is no active triangle around the vertex we don't ckech the line
    if (! hasActiveTriangle) return;


    helper::ReadAccessor<Data <VecCoord> > x2 = *this->getMstate()->read(core::VecCoordId::position());
    Vector3 P = x2[p];

    defaulttype::Vec3i cbox_min;
    cbox_min[0] = floor(decorator->m_Bmin[0]/decorator->m_cellSize[0]);
    cbox_min[1] = floor(decorator->m_Bmin[1]/decorator->m_cellSize[1]);
    cbox_min[2] = floor(decorator->m_Bmin[2]/decorator->m_cellSize[2]);


    defaulttype::Vec3i cbox_max;
    cbox_max[0] = floor(decorator->m_Bmax[0]/decorator->m_cellSize[0]);
    cbox_max[1] = floor(decorator->m_Bmax[1]/decorator->m_cellSize[1]);
    cbox_max[2] = floor(decorator->m_Bmax[2]/decorator->m_cellSize[2]);

    defaulttype::Vec3i cbox_A;
    cbox_A[0] = floor(A[0]/decorator->m_cellSize[0]);
    cbox_A[1] = floor(A[1]/decorator->m_cellSize[1]);
    cbox_A[2] = floor(A[2]/decorator->m_cellSize[2]);


    defaulttype::Vec3i cbox_P;
    cbox_P[0] = floor(P[0]/decorator->m_cellSize[0]);
    cbox_P[1] = floor(P[1]/decorator->m_cellSize[1]);
    cbox_P[2] = floor(P[2]/decorator->m_cellSize[2]);

    defaulttype::Vec3i cordA;
    cordA[0] = std::min(std::max(cbox_A[0],cbox_min[0]),cbox_max[0]) - cbox_min[0];
    cordA[1] = std::min(std::max(cbox_A[1],cbox_min[1]),cbox_max[1]) - cbox_min[1];
    cordA[2] = std::min(std::max(cbox_A[2],cbox_min[2]),cbox_max[2]) - cbox_min[2];

    defaulttype::Vec3i cordP;
    cordP[0] = std::min(std::max(cbox_P[0],cbox_min[0]),cbox_max[0]) - cbox_min[0];
    cordP[1] = std::min(std::max(cbox_P[1],cbox_min[1]),cbox_max[1]) - cbox_min[1];
    cordP[2] = std::min(std::max(cbox_P[2],cbox_min[2]),cbox_max[2]) - cbox_min[2];


    defaulttype::Vec3i cordFrom;
    cordFrom[0] = std::min(cordA[0],cordP[0]);
    cordFrom[1] = std::min(cordA[1],cordP[1]);
    cordFrom[2] = std::min(cordA[2],cordP[2]);

    defaulttype::Vec3i cordTo;
    cordTo[0] = std::max(cordA[0],cordP[0]);
    cordTo[1] = std::max(cordA[1],cordP[1]);
    cordTo[2] = std::max(cordA[2],cordP[2]);

    std::set<unsigned> triangleSet;
    for (int i=cordFrom[0];i<=cordTo[0];i++) {
        for (int j=cordFrom[1];j<=cordTo[1];j++) {
            for (int k=cordFrom[2];k<=cordTo[2];k++) {
                const helper::vector<unsigned> & triangles = decorator->m_triangleboxes[i][j][k];

                for (unsigned t=0;t<triangles.size();t++) triangleSet.insert(triangles[t]);
            }
        }
    }

    for(std::set<unsigned>::iterator it=triangleSet.begin();it!=triangleSet.end();++it) {
        unsigned ta = *it;

        if (!m_filteredTriangle[ta]) continue;

        const core::topology::BaseMeshTopology::Triangle tri = this->getTopology()->getTriangle(ta);

        Vector3 e1 = x2[tri[1]] - x2[tri[0]];
        Vector3 e2 = x2[tri[2]] - x2[tri[0]];
        Vector3 dir = A - P; //D
        dir.normalize();

        Vector3 N = cross(dir,e2); //P
        double  det = dot(e1,N);

        if(fabs(det) < d_epsilon.getValue()) continue ;

        double invDet = 1.0/det;

        Vector3 T = P-x2[tri[0]] ;
        double u  = dot(T,N) * invDet;

        //The intersection lies outside of the triangle
        if(u < 0.0 || u > 1.0) continue ;

        Vector3 Q = cross(T,e1) ;
        double v = dot(dir,Q) * invDet;

        //The intersection lies outside of the triangle
        if(v < 0.0 || u + v  > 1.0) continue ;

        if ((dot(e2,Q) * invDet) > d_epsilon.getValue() ){
            for (unsigned t=0;t<tav.size();t++) m_filteredTriangle[tav[t]] = false;
            return;
        }
    }
}

BaseDecorator * ProjectiveContourGeometry::getDecorator() {
    return NULL;
}

void ProjectiveContourGeometry::filterTriangleFunction(const unsigned p, const Vector3 & A) {

    helper::ReadAccessor<Data <VecCoord> > x2 = *this->getMstate()->read(core::VecCoordId::position());
    Vector3 P = x2[p];

    bool hasActiveTriangle = false;
    const Topology::TrianglesAroundVertex & tav = this->getTopology()->getTrianglesAroundVertex(p);
    for (unsigned t=0;!hasActiveTriangle && t<tav.size();t++) {
        if (m_filteredTriangle[tav[t]]) hasActiveTriangle = true;
    }

    //if there is no active triangle around the vertex we don't ckech the line
    if (! hasActiveTriangle) return;

    for (int ta=0;ta<this->getTopology()->getNbTriangles();ta++) {
        if (!m_filteredTriangle[ta]) continue;

        const core::topology::BaseMeshTopology::Triangle tri = this->getTopology()->getTriangle(ta);

        Vector3 e1 = x2[tri[1]] - x2[tri[0]];
        Vector3 e2 = x2[tri[2]] - x2[tri[0]];
        Vector3 dir = A - P; //D
        dir.normalize();

        Vector3 N = cross(dir,e2); //P
        double  det = dot(e1,N);

        if(fabs(det) < d_epsilon.getValue()) continue ;

        double invDet = 1.0/det;

        Vector3 T = P-x2[tri[0]] ;
        double u  = dot(T,N) * invDet;

        //The intersection lies outside of the triangle
        if(u < 0.0 || u > 1.0) continue ;

        Vector3 Q = cross(T,e1) ;
        double v = dot(dir,Q) * invDet;

        //The intersection lies outside of the triangle
        if(v < 0.0 || u + v  > 1.0) continue ;

        if ((dot(e2,Q) * invDet) > d_epsilon.getValue() ){
            for (unsigned t=0;t<tav.size();t++) m_filteredTriangle[tav[t]] = false;
            return;
        }
    }
}

void ProjectiveContourGeometry::prepareDetection() {
    Inherit::prepareDetection();

    Vector3 T;
    Matrix3 C;
    for (unsigned j=0;j<3;j++) {
        for (unsigned i=0;i<3;i++) {
            C[j][i] = d_projectionMatrix.getValue()[j][i];
        }
        T[j] = d_projectionMatrix.getValue()[j][3];
    }

    m_iC.invert(C);
    m_A = -m_iC * T; //camera position

    Vector3 Z(d_projectionMatrix.getValue()[2][0],
              d_projectionMatrix.getValue()[2][1],
              d_projectionMatrix.getValue()[2][2]);
    Z.normalize();

    //select only triangles frontFace triangle
    m_filteredTriangle.resize(this->getTopology()->getNbTriangles());
    for (int t=0;t<this->getTopology()->getNbTriangles();t++) {
        m_filteredTriangle[t] = dot(Z,this->m_triangle_info[t].tn) < 0;
    }

    AABBDecorator * aabb_decorator;
    this->getContext()->get(aabb_decorator);

    // compute the projection and check that there is no triangle in fron of the point
//    m_xproj.resize(this->getTopology()->getNbPoints());
    for (int p=0;p<this->getTopology()->getNbPoints();p++) {
        if (aabb_decorator) filterTriangleFunctionWithAABB(aabb_decorator,p,m_A) ;
        else filterTriangleFunction(p,m_A) ;
//        m_xproj[p] = get3DFrom2DPosition(get2DFrom3DPosition(x2[p]));
    }

    m_filteredEdge.clear();
    for (int e=0;e<getTopology()->getNbEdges();e++) {
        const core::topology::BaseMeshTopology::TrianglesAroundEdge & tae = this->getTopology()->getTrianglesAroundEdge(e);

        if (tae.size() == 1) {
            m_filteredEdge.push_back(e);
        } else if (tae.size() == 2) {
            if (( m_filteredTriangle[tae[0]] && !m_filteredTriangle[tae[1]]) ||
                    (!m_filteredTriangle[tae[0]] &&  m_filteredTriangle[tae[1]])) {
                m_filteredEdge.push_back(e);
            }
        }
    }
}

int ProjectiveContourGeometry::getNbElements() {
    return m_filteredEdge.size();
}

defaulttype::Vector2 ProjectiveContourGeometry::project3d(const defaulttype::Vector3 & p) {
    const Mat3x4d & P = d_projectionMatrix.getValue();

    double rx = P[0][0] * p[0] + P[0][1] * p[1] + P[0][2] * p[2] + P[0][3];
    double ry = P[1][0] * p[0] + P[1][1] * p[1] + P[1][2] * p[2] + P[1][3];
    double rz = P[2][0] * p[0] + P[2][1] * p[1] + P[2][2] * p[2] + P[2][3];

    return Vector2 (rx,ry) * 1.0/rz;
}

double ProjectiveContourGeometry::projectPoint(unsigned eid,const defaulttype::Vector3 & s,BaseProximity & pinfo) {
    defaulttype::Vector2 P = project3d(s);

    const helper::ReadAccessor<Data <VecCoord> >& x = *this->getMstate()->read(core::VecCoordId::position());

    sofa::core::topology::BaseMeshTopology::Edge edge = this->getTopology()->getEdge(m_filteredEdge[eid]);

    double fact_u;
    double fact_v;

    Vector2 x0 = project3d(x[edge[0]]);
    Vector2 x1 = project3d(x[edge[1]]);

    Vector2 v = x1 - x0;
    fact_v = dot (P - x0,v) / dot (v,v);

    if (fact_v<0.0) fact_v = 0.0;
    else if (fact_v>1.0) fact_v = 1.0;

    fact_u = 1.0-fact_v;

    pinfo = getEdgeProximity(m_filteredEdge[eid],fact_u,fact_v);

    return (project3d(pinfo.getPosition()) - P).norm();
}

defaulttype::Vector3 ProjectiveContourGeometry::getNormal(const BaseProximity & pinfo) {
    Vector3 P = pinfo.getPosition();

    Vector3 N1g = this->m_pointNormal[pinfo.m_pid[0]] * pinfo.m_fact[0] + this->m_pointNormal[pinfo.m_pid[1]] * pinfo.m_fact[1];

    Vector3 Y = cross (P-m_A,N1g);
    Y.normalize();

    Vector3 N1 = cross(m_A-P,Y);
    N1.normalize();

    return N1;
}

void ProjectiveContourGeometry::draw(const core::visual::VisualParams* /*vparams*/)
{
    //if (!vparams->displayFlags().getShowCollisionModels()) return;

    helper::ReadAccessor<Data <VecCoord> > x = *this->getMstate()->read(core::VecCoordId::position());
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glColor3f(0.0,0.0,1.0);
    glLineWidth(9);
    for (unsigned e=0;e<m_filteredEdge.size();e++) {
        sofa::core::topology::BaseMeshTopology::Edge edge = this->getTopology()->getEdge(m_filteredEdge[e]);

        helper::gl::glVertexT(x[edge[0]]); helper::gl::glVertexT(x[edge[1]]);
    }
    glLineWidth(1);
    glEnd();
}

}
