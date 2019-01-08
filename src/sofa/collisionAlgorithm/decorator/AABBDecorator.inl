#pragma once

#include <sofa/collisionAlgorithm/decorator/AABBDecorator.h>

namespace sofa
{

namespace collisionAlgorithm
{

AABBDecorator::AABBDecorator(BaseGeometry* geometry)
: BaseElementFilter(geometry)
, d_nbox(initData(&d_nbox, defaulttype::Vec3i(8,8,8),"nbox", "number of bbox"))
{
}

void AABBDecorator::prepareDetection()
{
    const helper::ReadAccessor<DataVecCoord> & pos = l_geometry->getState()->read(core::VecCoordId::position());
    if (pos.empty()) return;

    m_Bmin = pos[0];
    m_Bmax = pos[0];
    for (unsigned i=1;i<pos.size();i++)
    {
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
    }
    else
        m_nbox[0] = d_nbox.getValue()[0] + 1;

    if (m_cellSize[1] == 0)
    {
        m_cellSize[1] = (m_cellSize[0]+m_cellSize[2])*0.5;
        m_nbox[1] = 1;
    }
    else
        m_nbox[1] = d_nbox.getValue()[1] + 1;

    if (m_cellSize[2] == 0)
    {
        m_cellSize[2] = (m_cellSize[0]+m_cellSize[1])*0.5;
        m_nbox[2] = 1;
    }
    else
        m_nbox[2] = d_nbox.getValue()[2] + 1;

    m_indexedElement.resize(m_nbox[0]*m_nbox[1]*m_nbox[2]);
    m_offset[0] = m_nbox[1]*m_nbox[2];
    m_offset[1] = m_nbox[2];

    for (unsigned i=0;i<m_indexedElement.size();i++)
    {
        m_indexedElement[i].clear();
    }

    // center in -0.5 cellwidth
    m_Bmin -= m_cellSize * 0.5;
    m_Bmax -= m_cellSize * 0.5;

    for (unsigned itE = 0; itE < l_geometry->getNbElements(); itE++)
    {
        const BaseElement * elmt = l_geometry->getElement(itE);
        if (elmt->getNbControlPoints() == 0) continue;

        defaulttype::Vector3 minbox = elmt->getControlPoint(0)->getPosition();
        defaulttype::Vector3 maxbox = elmt->getControlPoint(0)->getPosition();
        for (unsigned p=1;p<elmt->getNbControlPoints();p++)
        {
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

        for (int i=cminbox[0];i<cmaxbox[0];i++)
        {
            for (int j=cminbox[1];j<cmaxbox[1];j++)
            {
                for (int k=cminbox[2];k<cmaxbox[2];k++)
                {
                    getIndexedElements(i,j,k).insert(itE);
                }
            }
        }
    }
}

void AABBDecorator::draw(const core::visual::VisualParams * vparams) {
    if (! vparams->displayFlags().getShowCollisionModels()) return;

    for (int i=0;i<m_nbox[0];i++)
    {
        for (int j=0;j<m_nbox[1];j++)
        {
            for (int k=0;k<m_nbox[2];k++)
            {
                if (getIndexedElements(i,j,k).empty())
                    continue;

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

AABBElementIterator::AABBElementIterator(const BaseElement *from, const AABBDecorator * aabb)
    : BaseElementFilterIterator(from, aabb)
    , m_aabb(aabb)
{
    defaulttype::Vector3 P = from->getControlPoint()->getPosition();

    //compute the box where is P
    defaulttype::Vec3i cbox;
    cbox[0] = floor((P[0] - m_aabb->m_Bmin[0])/m_aabb->m_cellSize[0]);
    cbox[1] = floor((P[1] - m_aabb->m_Bmin[1])/m_aabb->m_cellSize[1]);
    cbox[2] = floor((P[2] - m_aabb->m_Bmin[2])/m_aabb->m_cellSize[2]);

    //project the box in the bounding box of the object
    //search with the closest box in bbox
    for (unsigned int i=0;i<3;i++)
    {
        if (cbox[i] < 0)
            cbox[i] = 0;
        else
        {
            if (cbox[i] > m_aabb->m_nbox[i])
                cbox[i] = m_aabb->m_nbox[i];
        }
    }

    int d = 0;
    int max = std::max(std::max(m_aabb->m_nbox[0],m_aabb->m_nbox[1]),m_aabb->m_nbox[2]);
    std::set<int> selectElements;
    while (selectElements.empty() && d<max)
    {
        fillElementSet(cbox,d,selectElements);
        d++;// we look for boxed located at d+1
    }

    m_selectElements.clear();
    for (std::set<int>::iterator it = selectElements.begin();it != selectElements.end(); it++) m_selectElements.push_back(*it);
}

void AABBElementIterator::fillElementSet(defaulttype::Vec3i cbox, int d, std::set<int> & selectElements)
{
    {
        int i=-d;
        if (cbox[0]+i >= 0 && cbox[0]+i < m_aabb->m_nbox[0])
        {
            for (int j=-d;j<=d;j++)
            {
                if (cbox[1]+j < 0 || cbox[1]+j >= m_aabb->m_nbox[1])
                    continue;
                for (int k=-d;k<=d;k++)
                {
                    if (cbox[2]+k < 0 || cbox[2]+k >= m_aabb->m_nbox[2])
                        continue;

                    const std::set<unsigned> & elemntsID = m_aabb->getConstIndexedElements(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                    selectElements.insert(elemntsID.begin(),elemntsID.end());
                }
            }
        }
    }

    {
        int i=d;
        if (cbox[0]+i >= 0 && cbox[0]+i < m_aabb->m_nbox[0])
        {
            for (int j=-d;j<=d;j++)
            {
                if (cbox[1]+j < 0 || cbox[1]+j >= m_aabb->m_nbox[1])
                    continue;

                for (int k=-d;k<=d;k++)
                {
                    if (cbox[2]+k < 0 || cbox[2]+k >= m_aabb->m_nbox[2])
                        continue;

                    const std::set<unsigned> & elemntsID = m_aabb->getConstIndexedElements(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                    selectElements.insert(elemntsID.begin(),elemntsID.end());
                }
            }
        }
    }


    {
        int j=-d;
        if (cbox[1]+j >= 0 && cbox[1]+j < m_aabb->m_nbox[1])
        {
            for (int i=-d+1;i<d;i++)
            {
                if (cbox[0]+i < 0 || cbox[0]+i >= m_aabb->m_nbox[0])
                    continue;

                for (int k=-d;k<=d;k++)
                {
                    if (cbox[2]+k < 0 || cbox[2]+k >= m_aabb->m_nbox[2])
                        continue;

                    const std::set<unsigned> & elemntsID = m_aabb->getConstIndexedElements(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                    selectElements.insert(elemntsID.begin(),elemntsID.end());
                }
            }
        }
    }

    {
        int j=d;
        if (cbox[1]+j >= 0 && cbox[1]+j < m_aabb->m_nbox[1])
        {
            for (int i=-d+1;i<d;i++)
            {
                if (cbox[0]+i < 0 || cbox[0]+i >= m_aabb->m_nbox[0])
                    continue;

                for (int k=-d;k<=d;k++)
                {
                    if (cbox[2]+k < 0 || cbox[2]+k >= m_aabb->m_nbox[2])
                        continue;

                    const std::set<unsigned> & elemntsID = m_aabb->getConstIndexedElements(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                    selectElements.insert(elemntsID.begin(),elemntsID.end());
                }
            }
        }
    }

    {
        int k=-d;
        if (cbox[2]+k >= 0 && cbox[2]+k < m_aabb->m_nbox[2])
        {
            for (int i=-d+1;i<d;i++)
            {
                if (cbox[0]+i < 0 || cbox[0]+i >= m_aabb->m_nbox[0])
                    continue;

                for (int j=-d+1;j<d;j++)
                {
                    if (cbox[1]+j < 0 || cbox[1]+j >= m_aabb->m_nbox[1])
                        continue;

                    const std::set<unsigned> & elemntsID = m_aabb->getConstIndexedElements(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                    selectElements.insert(elemntsID.begin(),elemntsID.end());
                }
            }
        }
    }

    {
        int k=d;
        if (cbox[2]+k >= 0 && cbox[2]+k < m_aabb->m_nbox[2])
        {
            for (int i=-d+1;i<d;i++)
            {
                if (cbox[0]+i < 0 || cbox[0]+i >= m_aabb->m_nbox[0])
                    continue;

                for (int j=-d+1;j<d;j++)
                {
                    if (cbox[1]+j < 0 || cbox[1]+j >= m_aabb->m_nbox[1])
                        continue;

                    const std::set<unsigned> & elemntsID = m_aabb->getConstIndexedElements(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                    selectElements.insert(elemntsID.begin(),elemntsID.end());
                }
            }
        }
    }
}


}

}
