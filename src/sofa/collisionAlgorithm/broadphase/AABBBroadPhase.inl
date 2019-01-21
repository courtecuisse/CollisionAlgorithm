#pragma once

#include <sofa/collisionAlgorithm/broadphase/AABBBroadPhase.h>
#include <sofa/collisionAlgorithm/proximity/FixedProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

AABBBroadPhase::AABBBroadPhase()
: d_nbox(initData(&d_nbox, defaulttype::Vec3i(8,8,8),"nbox", "number of bbox"))
, d_refineBBox(initData(&d_refineBBox, true,"refine", "Optimization to project center of box in order to find the minimal set of intersecting boxes")){
}

defaulttype::BoundingBox AABBBroadPhase::getBBox() const {
    return defaulttype::BoundingBox(m_Bmin,m_Bmax);
}

bool AABBBroadPhase::selectElement(const defaulttype::Vector3 & P,std::set<unsigned> & selectElements, unsigned d) const {
    if ((int) d>std::max(std::max(m_nbox[0],m_nbox[1]),m_nbox[2])) return false; // check that distance is still in the box

    //compute the box where is P
    defaulttype::Vec3i cbox;
    cbox[0] = floor((P[0] - m_Bmin[0])/m_cellSize[0]);
    cbox[1] = floor((P[1] - m_Bmin[1])/m_cellSize[1]);
    cbox[2] = floor((P[2] - m_Bmin[2])/m_cellSize[2]);

    //project the box in the bounding box of the object
    //search with the closest box in bbox
    for (unsigned int i=0;i<3;i++)
    {
        if (cbox[i] < 0) return false;
        else if (cbox[i] > m_nbox[i]) return false;
    }

    fillElementSet(cbox,selectElements,d);

    return true;
}

void AABBBroadPhase::computeCollisionReset() {
    const BaseGeometry * geo = l_elements->end();
    if (geo == NULL) return;

    sofa::core::behavior::BaseMechanicalState * mstate = geo->getState();

    m_Bmin = defaulttype::Vector3(mstate->getPX(0),mstate->getPY(0),mstate->getPZ(0));
    m_Bmax = m_Bmin;
    for (unsigned i=1;i<mstate->getSize();i++) {
        defaulttype::Vector3 pos(mstate->getPX(i),mstate->getPY(i),mstate->getPZ(i));

        if (pos[0]<m_Bmin[0]) m_Bmin[0] = pos[0];
        if (pos[1]<m_Bmin[1]) m_Bmin[1] = pos[1];
        if (pos[2]<m_Bmin[2]) m_Bmin[2] = pos[2];

        if (pos[0]>m_Bmax[0]) m_Bmax[0] = pos[0];
        if (pos[1]>m_Bmax[1]) m_Bmax[1] = pos[1];
        if (pos[2]>m_Bmax[2]) m_Bmax[2] = pos[2];
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

    m_indexedElement.clear();
    m_offset[0] = m_nbox[1]*m_nbox[2];
    m_offset[1] = m_nbox[2];

    // center in -0.5 cellwidth
    m_Bmin -= m_cellSize * 0.5;
    m_Bmax -= m_cellSize * 0.5;

    for (auto it = l_elements->begin(); it != l_elements->end(); it++)
    {
        defaulttype::BoundingBox bbox = (*it)->getBBox();

        const defaulttype::Vector3 & minbox = bbox.minBBox();
        const defaulttype::Vector3 & maxbox = bbox.maxBBox();

        defaulttype::Vec3i cminbox(0,0,0);
        defaulttype::Vec3i cmaxbox(0,0,0);

        cminbox[0] = floor((minbox[0] - m_Bmin[0])/m_cellSize[0]);
        cminbox[1] = floor((minbox[1] - m_Bmin[1])/m_cellSize[1]);
        cminbox[2] = floor((minbox[2] - m_Bmin[2])/m_cellSize[2]);

        cmaxbox[0] = ceil((maxbox[0] - m_Bmin[0])/m_cellSize[0]);
        cmaxbox[1] = ceil((maxbox[1] - m_Bmin[1])/m_cellSize[1]);
        cmaxbox[2] = ceil((maxbox[2] - m_Bmin[2])/m_cellSize[2]);

        const bool refine = d_refineBBox.getValue();

        for (int i=cminbox[0];i<cmaxbox[0];i++)
        {
            for (int j=cminbox[1];j<cmaxbox[1];j++)
            {
                for (int k=cminbox[2];k<cmaxbox[2];k++)
                {
                    if (refine) { // project the point on the element in order to know if the box is empty
                        defaulttype::Vector3 P = m_Bmin + m_cellSize*0.5;

                        P[0] += i*m_cellSize[0];
                        P[1] += j*m_cellSize[1];
                        P[2] += k*m_cellSize[2];

                        defaulttype::Vector3 D = P - (*it)->project(P)->getPosition();

                        if ((fabs(D[0])<=m_cellSize[0]*0.5) &&
                            (fabs(D[1])<=m_cellSize[1]*0.5) &&
                            (fabs(D[2])<=m_cellSize[2]*0.5))
                            m_indexedElement[getKey(i,j,k)].insert(it->id());
                    } else {
                        m_indexedElement[getKey(i,j,k)].insert(it->id());
                    }
                }
            }
        }
    }
}

void AABBBroadPhase::draw(const core::visual::VisualParams * vparams) {
    if (! vparams->displayFlags().getShowBoundingCollisionModels()) return;

    if (this->d_color.getValue()[3] == 0.0)
        return;

    glDisable(GL_LIGHTING);

    glColor4dv(this->d_color.getValue().data());

    for (auto it = m_indexedElement.begin(); it != m_indexedElement.end(); it++) {
        unsigned eid = it->first;

        unsigned i = (eid) / m_offset[0];
        unsigned j = (eid - i*m_offset[0]) / m_offset[1];
        unsigned k = (eid - i*m_offset[0] - j*m_offset[1]);

        defaulttype::Vector3 min = m_Bmin + defaulttype::Vector3((i  ) * m_cellSize[0],(j  ) * m_cellSize[1],(k  ) * m_cellSize[2]) ;
        defaulttype::Vector3 max = m_Bmin + defaulttype::Vector3((i+1) * m_cellSize[0],(j+1) * m_cellSize[1],(k+1) * m_cellSize[2]) ;
        defaulttype::BoundingBox bbox(min,max);

        defaulttype::Vector3 points[8];

        points[0] = defaulttype::Vector3(bbox.minBBox()[0], bbox.minBBox()[1], bbox.minBBox()[2]);
        points[1] = defaulttype::Vector3(bbox.maxBBox()[0], bbox.minBBox()[1], bbox.minBBox()[2]);
        points[2] = defaulttype::Vector3(bbox.minBBox()[0], bbox.maxBBox()[1], bbox.minBBox()[2]);
        points[3] = defaulttype::Vector3(bbox.maxBBox()[0], bbox.maxBBox()[1], bbox.minBBox()[2]);
        points[4] = defaulttype::Vector3(bbox.minBBox()[0], bbox.minBBox()[1], bbox.maxBBox()[2]);
        points[5] = defaulttype::Vector3(bbox.maxBBox()[0], bbox.minBBox()[1], bbox.maxBBox()[2]);
        points[6] = defaulttype::Vector3(bbox.minBBox()[0], bbox.maxBBox()[1], bbox.maxBBox()[2]);
        points[7] = defaulttype::Vector3(bbox.maxBBox()[0], bbox.maxBBox()[1], bbox.maxBBox()[2]);


//        if (vparams->displayFlags().getShowWireFrame()) {
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
//        } else {
//            glBegin(GL_QUADS);
//                glColor3dv((this->d_color.getValue()*0.8).data());
//                glVertex3dv(points[0].data());glVertex3dv(points[1].data());glVertex3dv(points[3].data());glVertex3dv(points[2].data());

//                glColor3dv((this->d_color.getValue()*0.7).data());
//                glVertex3dv(points[4].data());glVertex3dv(points[5].data());glVertex3dv(points[7].data());glVertex3dv(points[6].data());

//                glColor3dv((this->d_color.getValue()*0.6).data());
//                glVertex3dv(points[2].data());glVertex3dv(points[3].data());glVertex3dv(points[7].data());glVertex3dv(points[6].data());

//                glColor3dv((this->d_color.getValue()*0.5).data());
//                glVertex3dv(points[0].data());glVertex3dv(points[1].data());glVertex3dv(points[5].data());glVertex3dv(points[4].data());

//                glColor3dv((this->d_color.getValue()*0.4).data());
//                glVertex3dv(points[3].data());glVertex3dv(points[1].data());glVertex3dv(points[5].data());glVertex3dv(points[7].data());

//                glColor3dv((this->d_color.getValue()*0.3).data());
//                glVertex3dv(points[2].data());glVertex3dv(points[0].data());glVertex3dv(points[4].data());glVertex3dv(points[6].data());
//            glEnd();
//        }

    }
}

void AABBBroadPhase::fillElementSet(defaulttype::Vec3i cbox, std::set<unsigned> & selectElements, int d) const
{
    {
        int i=-d;
        if (cbox[0]+i >= 0 && cbox[0]+i < m_nbox[0])
        {
            for (int j=-d;j<=d;j++)
            {
                if (cbox[1]+j < 0 || cbox[1]+j >= m_nbox[1])
                    continue;
                for (int k=-d;k<=d;k++)
                {
                    if (cbox[2]+k < 0 || cbox[2]+k >= m_nbox[2])
                        continue;

                    auto it = m_indexedElement.find(getKey(cbox[0] + i,cbox[1] + j,cbox[2] + k));
                    if (it != m_indexedElement.cend()) {
                        const std::set<unsigned> & elemntsID = it->second;
                        selectElements.insert(elemntsID.begin(),elemntsID.end());
                    }
                }
            }
        }
    }

    {
        int i=d;
        if (cbox[0]+i >= 0 && cbox[0]+i < m_nbox[0])
        {
            for (int j=-d;j<=d;j++)
            {
                if (cbox[1]+j < 0 || cbox[1]+j >= m_nbox[1])
                    continue;

                for (int k=-d;k<=d;k++)
                {
                    if (cbox[2]+k < 0 || cbox[2]+k >= m_nbox[2])
                        continue;

                    auto it = m_indexedElement.find(getKey(cbox[0] + i,cbox[1] + j,cbox[2] + k));
                    if (it != m_indexedElement.end()) {
                        const std::set<unsigned> & elemntsID = it->second;
                        selectElements.insert(elemntsID.begin(),elemntsID.end());
                    }
                }
            }
        }
    }


    {
        int j=-d;
        if (cbox[1]+j >= 0 && cbox[1]+j < m_nbox[1])
        {
            for (int i=-d+1;i<d;i++)
            {
                if (cbox[0]+i < 0 || cbox[0]+i >= m_nbox[0])
                    continue;

                for (int k=-d;k<=d;k++)
                {
                    if (cbox[2]+k < 0 || cbox[2]+k >= m_nbox[2])
                        continue;

                    auto it = m_indexedElement.find(getKey(cbox[0] + i,cbox[1] + j,cbox[2] + k));
                    if (it != m_indexedElement.end()) {
                        const std::set<unsigned> & elemntsID = it->second;
                        selectElements.insert(elemntsID.begin(),elemntsID.end());
                    }
                }
            }
        }
    }

    {
        int j=d;
        if (cbox[1]+j >= 0 && cbox[1]+j < m_nbox[1])
        {
            for (int i=-d+1;i<d;i++)
            {
                if (cbox[0]+i < 0 || cbox[0]+i >= m_nbox[0])
                    continue;

                for (int k=-d;k<=d;k++)
                {
                    if (cbox[2]+k < 0 || cbox[2]+k >= m_nbox[2])
                        continue;

                    auto it = m_indexedElement.find(getKey(cbox[0] + i,cbox[1] + j,cbox[2] + k));
                    if (it != m_indexedElement.end()) {
                        const std::set<unsigned> & elemntsID = it->second;
                        selectElements.insert(elemntsID.begin(),elemntsID.end());
                    }
                }
            }
        }
    }

    {
        int k=-d;
        if (cbox[2]+k >= 0 && cbox[2]+k < m_nbox[2])
        {
            for (int i=-d+1;i<d;i++)
            {
                if (cbox[0]+i < 0 || cbox[0]+i >= m_nbox[0])
                    continue;

                for (int j=-d+1;j<d;j++)
                {
                    if (cbox[1]+j < 0 || cbox[1]+j >= m_nbox[1])
                        continue;

                    auto it = m_indexedElement.find(getKey(cbox[0] + i,cbox[1] + j,cbox[2] + k));
                    if (it != m_indexedElement.end()) {
                        const std::set<unsigned> & elemntsID = it->second;
                        selectElements.insert(elemntsID.begin(),elemntsID.end());
                    }
                }
            }
        }
    }

    {
        int k=d;
        if (cbox[2]+k >= 0 && cbox[2]+k < m_nbox[2])
        {
            for (int i=-d+1;i<d;i++)
            {
                if (cbox[0]+i < 0 || cbox[0]+i >= m_nbox[0])
                    continue;

                for (int j=-d+1;j<d;j++)
                {
                    if (cbox[1]+j < 0 || cbox[1]+j >= m_nbox[1])
                        continue;

                    auto it = m_indexedElement.find(getKey(cbox[0] + i,cbox[1] + j,cbox[2] + k));
                    if (it != m_indexedElement.end()) {
                        const std::set<unsigned> & elemntsID = it->second;
                        selectElements.insert(elemntsID.begin(),elemntsID.end());
                    }
                }
            }
        }
    }
}


}

}
