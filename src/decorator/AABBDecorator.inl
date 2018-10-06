#pragma once

#include <decorator/AABBDecorator.h>

namespace sofa {

namespace collisionAlgorithm {

AABBDecorator::AABBDecorator()
: d_nbox(initData(&d_nbox, defaulttype::Vec3i(8,8,8),"nbox", "number of bbox"))
, d_geometry("geometry", this) {}

void AABBDecorator::prepareDetection() {
    const helper::ReadAccessor<DataVecCoord> & pos = d_geometry->getState()->read(core::VecCoordId::position());
    if (pos.empty()) return;

    m_Bmin = pos[0];
    m_Bmax = pos[0];
    for (unsigned i=1;i<pos.size();i++) {
        if (pos[i][0]<m_Bmin[0]) m_Bmin[0] = pos[i][0];
        if (pos[i][1]<m_Bmin[1]) m_Bmin[1] = pos[i][1];
        if (pos[i][2]<m_Bmin[2]) m_Bmin[2] = pos[i][2];

        if (pos[i][0]>m_Bmax[0]) m_Bmax[0] = pos[i][0];
        if (pos[i][1]>m_Bmax[1]) m_Bmax[1] = pos[i][1];
        if (pos[i][2]>m_Bmax[2]) m_Bmax[2] = pos[i][2];
    }

    m_cellSize[0] = (m_Bmax[0] - m_Bmin[0]) / d_nbox.getValue()[0];
    m_cellSize[1] = (m_Bmax[1] - m_Bmin[1]) / d_nbox.getValue()[1];
    m_cellSize[2] = (m_Bmax[2] - m_Bmin[2]) / d_nbox.getValue()[2];

    if (m_cellSize[0] == 0) {
        m_cellSize[0] = (m_cellSize[1]+m_cellSize[2])*0.5;
        m_nbox[0] = 1;
    } else m_nbox[0] = d_nbox.getValue()[0] + 1;

    if (m_cellSize[1] == 0) {
        m_cellSize[1] = (m_cellSize[0]+m_cellSize[2])*0.5;
        m_nbox[1] = 1;
    } else m_nbox[1] = d_nbox.getValue()[1] + 1;

    if (m_cellSize[2] == 0) {
        m_cellSize[2] = (m_cellSize[0]+m_cellSize[1])*0.5;
        m_nbox[2] = 1;
    } else m_nbox[2] = d_nbox.getValue()[2] + 1;

    m_indexedElement.resize(m_nbox[0]*m_nbox[1]*m_nbox[2]);
    m_offset[0] = m_nbox[1]*m_nbox[2];
    m_offset[1] = m_nbox[2];

    for (unsigned i=0;i<m_indexedElement.size();i++) {
        m_indexedElement[i].clear();
    }

    // center in -0.5 cellwidth
    m_Bmin -= m_cellSize * 0.5;
    m_Bmax -= m_cellSize * 0.5;

    for (unsigned itE = 0; itE < d_geometry->getNbElements(); itE++) {
        ConstraintElementPtr elmt = d_geometry->getElement(itE);
        if (elmt->getNbControlPoints() == 0) continue;

        defaulttype::Vector3 minbox = elmt->getControlPoint(0)->getPosition();
        defaulttype::Vector3 maxbox = elmt->getControlPoint(0)->getPosition();
        for (unsigned p=1;p<elmt->getNbControlPoints();p++) {
            defaulttype::Vector3 P = elmt->getControlPoint(p)->getPosition();

            minbox[0] = std::min(minbox[0],P[0]);
            minbox[1] = std::min(minbox[1],P[1]);
            minbox[2] = std::min(minbox[2],P[2]);

            maxbox[0] = std::max(maxbox[0],P[0]);
            maxbox[1] = std::max(maxbox[1],P[1]);
            maxbox[2] = std::max(maxbox[2],P[2]);
        }

        defaulttype::Vec3i cminbox(0,0,0);
        defaulttype::Vec3i cmaxbox(0,0,0);

        cminbox[0] = floor((minbox[0] - m_Bmin[0])/m_cellSize[0]);
        cminbox[1] = floor((minbox[1] - m_Bmin[1])/m_cellSize[1]);
        cminbox[2] = floor((minbox[2] - m_Bmin[2])/m_cellSize[2]);

        cmaxbox[0] = ceil((maxbox[0] - m_Bmin[0])/m_cellSize[0]);
        cmaxbox[1] = ceil((maxbox[1] - m_Bmin[1])/m_cellSize[1]);
        cmaxbox[2] = ceil((maxbox[2] - m_Bmin[2])/m_cellSize[2]);

        for (int i=cminbox[0];i<cmaxbox[0];i++) {
            for (int j=cminbox[1];j<cmaxbox[1];j++) {
                for (int k=cminbox[2];k<cmaxbox[2];k++) {
                    getIndexedElements(i,j,k).insert(itE);
                }
            }
        }
    }
}

void AABBDecorator::draw(const core::visual::VisualParams * vparams) {
    if (! vparams->displayFlags().getShowCollisionModels()) return;

    for (int i=0;i<m_nbox[0];i++) {
        for (int j=0;j<m_nbox[1];j++) {
            for (int k=0;k<m_nbox[2];k++) {
                if (getIndexedElements(i,j,k).empty()) continue;

                defaulttype::Vector3 points[8];

                points[0] = m_Bmin + defaulttype::Vector3((i  ) * m_cellSize[0],(j  ) * m_cellSize[1],(k  ) * m_cellSize[2]) ;
                points[1] = m_Bmin + defaulttype::Vector3((i+1) * m_cellSize[0],(j  ) * m_cellSize[1],(k  ) * m_cellSize[2]) ;
                points[2] = m_Bmin + defaulttype::Vector3((i  ) * m_cellSize[0],(j+1) * m_cellSize[1],(k  ) * m_cellSize[2]) ;
                points[3] = m_Bmin + defaulttype::Vector3((i+1) * m_cellSize[0],(j+1) * m_cellSize[1],(k  ) * m_cellSize[2]) ;
                points[4] = m_Bmin + defaulttype::Vector3((i  ) * m_cellSize[0],(j  ) * m_cellSize[1],(k+1) * m_cellSize[2]) ;
                points[5] = m_Bmin + defaulttype::Vector3((i+1) * m_cellSize[0],(j  ) * m_cellSize[1],(k+1) * m_cellSize[2]) ;
                points[6] = m_Bmin + defaulttype::Vector3((i  ) * m_cellSize[0],(j+1) * m_cellSize[1],(k+1) * m_cellSize[2]) ;
                points[7] = m_Bmin + defaulttype::Vector3((i+1) * m_cellSize[0],(j+1) * m_cellSize[1],(k+1) * m_cellSize[2]) ;

                glColor4f(1,0,0,1);
                glBegin(GL_LINES);
                    glVertex3dv(points[0].data());glVertex3dv(points[1].data());
                    glVertex3dv(points[3].data());glVertex3dv(points[2].data());
                    glVertex3dv(points[7].data());glVertex3dv(points[6].data());
                    glVertex3dv(points[4].data());glVertex3dv(points[5].data());

                    glVertex3dv(points[0].data());glVertex3dv(points[2].data());
                    glVertex3dv(points[1].data());glVertex3dv(points[3].data());
                    glVertex3dv(points[4].data());glVertex3dv(points[6].data());
                    glVertex3dv(points[5].data());glVertex3dv(points[7].data());

                    glVertex3dv(points[0].data());glVertex3dv(points[4].data());
                    glVertex3dv(points[1].data());glVertex3dv(points[5].data());
                    glVertex3dv(points[2].data());glVertex3dv(points[6].data());
                    glVertex3dv(points[3].data());glVertex3dv(points[7].data());
                glEnd();
            }
        }
    }


}

}

}
