#ifndef SOFA_COMPONENT_AABBDECORATOR_INL
#define SOFA_COMPONENT_AABBDECORATOR_INL

#include "AABBDecorator.h"
#include <sofa/helper/Quater.h>
#include <sofa/core/visual/VisualParams.h>
#include <SofaOpenglVisual/OglModel.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <SofaConstraint/BilateralInteractionConstraint.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <sofa/core/visual/VisualParams.h>
#include <SofaOpenglVisual/OglModel.h>

namespace sofa {

namespace core {

namespace behavior {

#define min3(a,b,c) std::min(std::min(a,b),c)
#define max3(a,b,c) std::max(std::max(a,b),c)


AABBDecorator::AABBDecorator()
: d_nbox(initData(&d_nbox, defaulttype::Vec3i(8,8,8),"nbox","Number of bbox subdivision"))
, d_drawBbox(initData(&d_drawBbox, false,"drawBbox","Draw Bbox"))
{
    this->f_listening.setValue(true);
}

BaseConstraintIteratorPtr AABBDecorator::getIterator(const ConstraintProximityPtr & P) {
    return BaseConstraintIteratorPtr(new AABBIterator(this,P));
}

void AABBDecorator::prepareDetection() {
    sofa::core::behavior::MechanicalState<DataTypes> * state;
    topology::TopologyContainer * topology;

    this->getContext()->get(topology);
    this->getContext()->get(state);

    if (topology == NULL) return;
    if (state == NULL) return;

    helper::ReadAccessor<Data <VecCoord> > x = state->read(core::VecCoordId::position());

    m_Bmin = x[0];
    m_Bmax = x[0];
    for (unsigned p=0;p<x.size();p++) {
        m_Bmin[0] = std::min(m_Bmin[0],x[p][0]);
        m_Bmin[1] = std::min(m_Bmin[1],x[p][1]);
        m_Bmin[2] = std::min(m_Bmin[2],x[p][2]);

        m_Bmax[0] = std::max(m_Bmax[0],x[p][0]);
        m_Bmax[1] = std::max(m_Bmax[1],x[p][1]);
        m_Bmax[2] = std::max(m_Bmax[2],x[p][2]);
    }

    m_cellSize[0] = (m_Bmax[0] - m_Bmin[0]) / d_nbox.getValue()[0];
    m_cellSize[1] = (m_Bmax[1] - m_Bmin[1]) / d_nbox.getValue()[1];
    m_cellSize[2] = (m_Bmax[2] - m_Bmin[2]) / d_nbox.getValue()[2];

    m_Bmin -= m_cellSize * 0.5;
    m_Bmax -= m_cellSize * 0.5;

    for (unsigned i=0;i<m_triangleboxes.size();i++) {
        for (unsigned j=0;j<m_triangleboxes[i].size();j++) {
            for (unsigned k=0;k<m_triangleboxes[i][j].size();k++) {
                m_triangleboxes[i][j][k].clear();
            }
        }
    }


    for (int t=0;t<topology->getNbTriangles();t++) {
        const topology::Triangle tri = topology->getTriangle(t);

        //Compute Bezier Positions
        defaulttype::Vector3 p0 = x[tri[0]];
        defaulttype::Vector3 p1 = x[tri[1]];
        defaulttype::Vector3 p2 = x[tri[2]];

        //add the triangle in boxes
        defaulttype::Vector3 minbox(min3(p0[0], p1[0], p2[0]),min3(p0[1], p1[1], p2[1]),min3(p0[2], p1[2], p2[2]));
        defaulttype::Vector3 maxbox(max3(p0[0], p1[0], p2[0]),max3(p0[1], p1[1], p2[1]),max3(p0[2], p1[2], p2[2]));

        defaulttype::Vec3i cminbox;
        defaulttype::Vec3i cmaxbox;

        cminbox[0] = floor((minbox[0] - m_Bmin[0])/m_cellSize[0]);
        cminbox[1] = floor((minbox[1] - m_Bmin[1])/m_cellSize[1]);
        cminbox[2] = floor((minbox[2] - m_Bmin[2])/m_cellSize[2]);

        cmaxbox[0] = ceil((maxbox[0] - m_Bmin[0])/m_cellSize[0]);
        cmaxbox[1] = ceil((maxbox[1] - m_Bmin[1])/m_cellSize[1]);
        cmaxbox[2] = ceil((maxbox[2] - m_Bmin[2])/m_cellSize[2]);

        for (int i=cminbox[0];i<cmaxbox[0];i++) {
            for (int j=cminbox[1];j<cmaxbox[1];j++) {
                for (int k=cminbox[2];k<cmaxbox[2];k++) {
                    m_triangleboxes[i][j][k].push_back(t);
                }
            }
        }
    }
}

void AABBDecorator::init() {
    reinit();
}

void AABBDecorator::reinit() {
    m_triangleboxes.resize(d_nbox.getValue()[0]+1);
    for (unsigned i=0;i<m_triangleboxes.size();i++) {
        m_triangleboxes[i].resize(d_nbox.getValue()[1]+1);
        for (unsigned j=0;j<m_triangleboxes[i].size();j++) {
            m_triangleboxes[i][j].resize(d_nbox.getValue()[2]+1);
        }
    }
}

void AABBDecorator::draw(const core::visual::VisualParams * /*vparams*/) {
    if (this->d_drawBbox.getValue()) {
        for (int i=0;i<=d_nbox.getValue()[0];i++) {
            for (int j=0;j<=d_nbox.getValue()[1];j++) {
                for (int k=0;k<=d_nbox.getValue()[2];k++) {
                    if (m_triangleboxes[i][j][k].empty()) continue;

                    defaulttype::Vector3 points[8];
                    points[0] = m_Bmin + defaulttype::Vector3((i  ) * m_cellSize[0],(j  ) * m_cellSize[1],(k  ) * m_cellSize[2]) ;
                    points[1] = m_Bmin + defaulttype::Vector3((i+1) * m_cellSize[0],(j  ) * m_cellSize[1],(k  ) * m_cellSize[2]) ;
                    points[2] = m_Bmin + defaulttype::Vector3((i  ) * m_cellSize[0],(j+1) * m_cellSize[1],(k  ) * m_cellSize[2]) ;
                    points[3] = m_Bmin + defaulttype::Vector3((i+1) * m_cellSize[0],(j+1) * m_cellSize[1],(k  ) * m_cellSize[2]) ;
                    points[4] = m_Bmin + defaulttype::Vector3((i  ) * m_cellSize[0],(j  ) * m_cellSize[1],(k+1) * m_cellSize[2]) ;
                    points[5] = m_Bmin + defaulttype::Vector3((i+1) * m_cellSize[0],(j  ) * m_cellSize[1],(k+1) * m_cellSize[2]) ;
                    points[6] = m_Bmin + defaulttype::Vector3((i  ) * m_cellSize[0],(j+1) * m_cellSize[1],(k+1) * m_cellSize[2]) ;
                    points[7] = m_Bmin + defaulttype::Vector3((i+1) * m_cellSize[0],(j+1) * m_cellSize[1],(k+1) * m_cellSize[2]) ;

                    glColor4f(1,1,1,1);
                    glBegin(GL_LINES);
                        helper::gl::glVertexT(points[0]);helper::gl::glVertexT(points[1]);
                        helper::gl::glVertexT(points[3]);helper::gl::glVertexT(points[2]);
                        helper::gl::glVertexT(points[7]);helper::gl::glVertexT(points[6]);
                        helper::gl::glVertexT(points[4]);helper::gl::glVertexT(points[5]);

                        helper::gl::glVertexT(points[0]);helper::gl::glVertexT(points[2]);
                        helper::gl::glVertexT(points[1]);helper::gl::glVertexT(points[3]);
                        helper::gl::glVertexT(points[4]);helper::gl::glVertexT(points[6]);
                        helper::gl::glVertexT(points[5]);helper::gl::glVertexT(points[7]);

                        helper::gl::glVertexT(points[0]);helper::gl::glVertexT(points[4]);
                        helper::gl::glVertexT(points[1]);helper::gl::glVertexT(points[5]);
                        helper::gl::glVertexT(points[2]);helper::gl::glVertexT(points[6]);
                        helper::gl::glVertexT(points[3]);helper::gl::glVertexT(points[7]);
                    glEnd();
                }
            }
        }
    }
}

} //controller

} //component

}//Sofa

#endif
