#pragma once

#include <sofa/collisionAlgorithm/geometry/AABBGeometry.h>
#include <sofa/collisionAlgorithm/proximity/FixedProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

//Internal iterator of elements
class AABBElement : public BaseElement {
public:
    AABBElement(const AABBGeometry * geometry) : m_geometry(geometry) {
        m_iterator = geometry->m_indexedElement.cbegin();
    }

    BaseProximity::SPtr project(const defaulttype::Vector3 & P) const {

    }

    virtual BaseProximity::SPtr center() const {
        const std::set<unsigned> & selected = m_iterator->second;
        unsigned eid = *(selected.begin());

        return m_geometry->l_geometry->begin(eid)->center();
        defaulttype::BoundingBox bbox = getBBox();
        return BaseProximity::SPtr(new FixedProximity((bbox.minBBox() + bbox.maxBBox())*0.5));
    }

    void next() {
        m_iterator++;
    }

    unsigned id() const {
        return m_iterator->first;
    }

    bool end(const BaseGeometry * /*geo*/) const{
        return m_iterator!=m_geometry->m_indexedElement.end();
    }

    virtual defaulttype::BoundingBox getBBox() const {
        unsigned eid = m_iterator->first;

        unsigned i = (eid) / m_geometry->m_offset[0];
        unsigned j = (eid - i*m_geometry->m_offset[0]) / m_geometry->m_offset[1];
        unsigned k = (eid - i*m_geometry->m_offset[0] - j*m_geometry->m_offset[1]);

        defaulttype::Vector3 min = m_geometry->m_Bmin + defaulttype::Vector3((i  ) * m_geometry->m_cellSize[0],(j  ) * m_geometry->m_cellSize[1],(k  ) * m_geometry->m_cellSize[2]) ;
        defaulttype::Vector3 max = m_geometry->m_Bmin + defaulttype::Vector3((i+1) * m_geometry->m_cellSize[0],(j+1) * m_geometry->m_cellSize[1],(k+1) * m_geometry->m_cellSize[2]) ;

        return defaulttype::BoundingBox(min,max);
    }

    const AABBGeometry * m_geometry;
    std::set<int> m_selectElements;
    std::map<unsigned, std::set<unsigned> >::const_iterator m_iterator;
};

AABBGeometry::AABBGeometry()
: d_nbox(initData(&d_nbox, defaulttype::Vec3i(8,8,8),"nbox", "number of bbox"))
, l_geometry(initLink("geometry", "link to state")) {
    l_geometry.setPath("@.");
}

BaseElement::Iterator AABBGeometry::begin(unsigned /*eid*/) const {
    return BaseElement::Iterator(new AABBElement(this));
}

sofa::core::behavior::BaseMechanicalState * AABBGeometry::getState() const {
    return l_geometry->getState();
}

void AABBGeometry::prepareDetection()
{
    sofa::core::behavior::BaseMechanicalState * mstate = l_geometry->getState();
    l_geometry->bwdInit();

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

    for (auto it = l_geometry->begin(); it != l_geometry->end(); it++)
    {
        defaulttype::BoundingBox bbox = it->getBBox();

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

        for (int i=cminbox[0];i<cmaxbox[0];i++)
        {
            for (int j=cminbox[1];j<cmaxbox[1];j++)
            {
                for (int k=cminbox[2];k<cmaxbox[2];k++)
                {
                    defaulttype::Vector3 P = m_Bmin + m_cellSize*0.5;

                    P[0] += i*m_cellSize[0];
                    P[1] += j*m_cellSize[1];
                    P[2] += k*m_cellSize[2];

                    defaulttype::Vector3 D = P - it->project(P)->getPosition();

                    if ((fabs(D[0])<m_cellSize[0]*0.5) &&
                        (fabs(D[1])<m_cellSize[1]*0.5) &&
                        (fabs(D[2])<m_cellSize[2]*0.5)) m_indexedElement[getKey(i,j,k)].insert(it->id());
                }
            }
        }
    }
}

void AABBGeometry::draw(const core::visual::VisualParams * vparams) {
    if (! vparams->displayFlags().getShowCollisionModels()) return;

    if (this->d_color.getValue()[3] == 0.0)
        return;

    glDisable(GL_LIGHTING);

    glColor3dv(this->d_color.getValue().data());

    for (auto it = begin(); it != end() ; it++) {
        defaulttype::BoundingBox bbox = it->getBBox();

        defaulttype::Vector3 points[8];

        points[0] = defaulttype::Vector3(bbox.minBBox()[0], bbox.minBBox()[1], bbox.minBBox()[2]);
        points[1] = defaulttype::Vector3(bbox.maxBBox()[0], bbox.minBBox()[1], bbox.minBBox()[2]);
        points[2] = defaulttype::Vector3(bbox.minBBox()[0], bbox.maxBBox()[1], bbox.minBBox()[2]);
        points[3] = defaulttype::Vector3(bbox.maxBBox()[0], bbox.maxBBox()[1], bbox.minBBox()[2]);
        points[4] = defaulttype::Vector3(bbox.minBBox()[0], bbox.minBBox()[1], bbox.maxBBox()[2]);
        points[5] = defaulttype::Vector3(bbox.maxBBox()[0], bbox.minBBox()[1], bbox.maxBBox()[2]);
        points[6] = defaulttype::Vector3(bbox.minBBox()[0], bbox.maxBBox()[1], bbox.maxBBox()[2]);
        points[7] = defaulttype::Vector3(bbox.maxBBox()[0], bbox.maxBBox()[1], bbox.maxBBox()[2]);

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

void AABBGeometry::selectElements(const defaulttype::Vector3 & P, std::set<unsigned> & selectElements) const {
    //compute the box where is P
    defaulttype::Vec3i cbox;
    cbox[0] = floor((P[0] - m_Bmin[0])/m_cellSize[0]);
    cbox[1] = floor((P[1] - m_Bmin[1])/m_cellSize[1]);
    cbox[2] = floor((P[2] - m_Bmin[2])/m_cellSize[2]);

    //project the box in the bounding box of the object
    //search with the closest box in bbox
    for (unsigned int i=0;i<3;i++)
    {
        if (cbox[i] < 0)
            cbox[i] = 0;
        else
        {
            if (cbox[i] > m_nbox[i])
                cbox[i] = m_nbox[i];
        }
    }

    int d = 0;
    int max = std::max(std::max(m_nbox[0],m_nbox[1]),m_nbox[2]);

    while (selectElements.empty() && d<max)
    {
        fillElementSet(cbox,d,selectElements);
        d++;// we look for boxed located at d+1
    }
}

void AABBGeometry::fillElementSet(defaulttype::Vec3i cbox, int d, std::set<unsigned> & selectElements) const
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
