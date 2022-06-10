#pragma once

#include <sofa/helper/AdvancedTimer.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/operations/Project.h>
#include <sofa/collisionAlgorithm/BaseElement.h>

namespace sofa::collisionAlgorithm {

class AABBBroadPhase : public BaseGeometry::BroadPhase {
public:

    SOFA_CLASS(AABBBroadPhase,BaseGeometry::BroadPhase);

    Data<type::Vec3i> d_nbox;
    Data<bool> d_static;

    core::objectmodel::DataCallback c_nbox;

    AABBBroadPhase()
    : d_nbox(initData(&d_nbox, type::Vec3i(8,8,8),"nbox", "number of bbox"))
    , d_static(initData(&d_static, false,"isStatic", "Optimization: object is not moving in the scene")) {
        c_nbox.addInputs({&d_nbox});
        c_nbox.addCallback(std::bind(&AABBBroadPhase::updateBroadPhase,this));
    }

    type::BoundingBox getBBox() const {
        return type::BoundingBox(m_Bmin,m_Bmax);
    }

    const std::set<BaseElement::SPtr> & getElementSet(unsigned i, unsigned j, unsigned k) const override {
        auto it = m_indexedElement.find(getKey(i,j,k));
        if (it == m_indexedElement.end()) {
            static std::set<BaseElement::SPtr> empty;
            return empty;
        } else {
            return it->second;
        }
    }

    void initBroadPhase() override {
        doUpdate();
    }

    void updateBroadPhase() override {
        if(d_static.getValue()) return;
        doUpdate();
    }

    inline void doUpdate() {
        m_Bmin = l_geometry->getPosition(0);
        m_Bmax = m_Bmin;

        //updates bounding box area
        for (unsigned j=1;j<l_geometry->getSize();j++) {
            type::Vector3 pos = l_geometry->getPosition(j);

            for (int i = 0 ; i < 3 ; i++) {
                if (pos[i] > m_Bmax[i])
                    m_Bmax[i] = pos[i] ;
                if (pos[i] < m_Bmin[i])
                    m_Bmin[i] = pos[i] ;
            }

    //        if (pos[0]<m_Bmin[0]) m_Bmin[0] = pos[0];
    //        if (pos[1]<m_Bmin[1]) m_Bmin[1] = pos[1];
    //        if (pos[2]<m_Bmin[2]) m_Bmin[2] = pos[2];

    //        if (pos[0]>m_Bmax[0]) m_Bmax[0] = pos[0];
    //        if (pos[1]>m_Bmax[1]) m_Bmax[1] = pos[1];
    //        if (pos[2]>m_Bmax[2]) m_Bmax[2] = pos[2];
        }

        //fixes cell size
        for (int i = 0 ; i < 3 ; i++) {
            m_cellSize[i] = (m_Bmax[i] - m_Bmin[i]) / d_nbox.getValue()[i];
        }
    //    m_cellSize[0] = (m_Bmax[0] - m_Bmin[0]) / d_nbox.getValue()[0];
    //    m_cellSize[1] = (m_Bmax[1] - m_Bmin[1]) / d_nbox.getValue()[1];
    //    m_cellSize[2] = (m_Bmax[2] - m_Bmin[2]) / d_nbox.getValue()[2];



        for (int i = 0 ; i < 3 ; i++) {
            if (m_cellSize[i] == 0) {
                int a = (i == 0) ? 1 : 0 ;
                int b = (i == 2) ? 1 : 2 ;
                m_cellSize[i] = (m_cellSize[a]+m_cellSize[b])*0.5;
                m_nbox[i] = 1;
            } else {
                m_nbox[i] = d_nbox.getValue()[i] + 1;
            }
        }

    //    if (m_cellSize[0] == 0) {
    //        m_cellSize[0] = (m_cellSize[1]+m_cellSize[2])*0.5;
    //        m_nbox[0] = 1;
    //    }
    //    else {
    //        m_nbox[0] = d_nbox.getValue()[0] + 1;
    //    }

    //    if (m_cellSize[1] == 0) {
    //        m_cellSize[1] = (m_cellSize[0]+m_cellSize[2])*0.5;
    //        m_nbox[1] = 1;
    //    }
    //    else
    //        m_nbox[1] = d_nbox.getValue()[1] + 1;

    //    if (m_cellSize[2] == 0)
    //    {
    //        m_cellSize[2] = (m_cellSize[0]+m_cellSize[1])*0.5;
    //        m_nbox[2] = 1;
    //    }
    //    else
    //        m_nbox[2] = d_nbox.getValue()[2] + 1;

        m_indexedElement.clear();

        m_offset[0] = m_nbox[1]*m_nbox[2];
        m_offset[1] = m_nbox[2];

        // center in -0.5 cellwidth
        m_Bmin -= m_cellSize * 0.5;
        m_Bmax -= m_cellSize * 0.5;

        auto projectOp = Operations::Project::Operation::get(l_geometry);

        for (auto it = l_geometry->begin(); it != l_geometry->end(); it++)
        {
            BaseElement::SPtr elmt = it->element();

            //std::cout << ++i << std::endl;
            type::BoundingBox bbox;
//            std::cout << "size v_prox : " << v_prox.size() << std::endl;

            for (auto it = elmt->pointElements().cbegin(); it!= elmt->pointElements().cend(); it++) {
                bbox.include((*it)->getP0()->getPosition());
            }

            const type::Vector3 & minbox = bbox.minBBox();
            const type::Vector3 & maxbox = bbox.maxBBox();

            type::Vec3i cminbox(0,0,0);
            type::Vec3i cmaxbox(0,0,0);

            for (int i = 0 ; i < 3 ; i++) {
                cmaxbox[i] = ceil((maxbox[i] - m_Bmin[i])/m_cellSize[i]);
                cminbox[i] = floor((minbox[i] - m_Bmin[i])/m_cellSize[i]); //second m_Bmax was Bmin => bug ?
            }

            for (int i=cminbox[0];i<cmaxbox[0];i++)
            {
                unsigned key_i = i*m_offset[0];

                for (int j=cminbox[1];j<cmaxbox[1];j++)
                {
                    unsigned key_j = j*m_offset[1];

                    for (int k=cminbox[2];k<cmaxbox[2];k++)
                    {
                        unsigned key = key_i + key_j + k;

                        type::Vector3 P = m_Bmin + m_cellSize*0.5;

                        P[0] += i*m_cellSize[0];
                        P[1] += j*m_cellSize[1];
                        P[2] += k*m_cellSize[2];

                        BaseProximity::SPtr prox = projectOp(P,elmt).prox;
                        if (prox == NULL) continue;

                        prox->normalize();
                        type::Vector3 D = prox->getPosition()-P;

                        if ((fabs(D[0])<=m_cellSize[0]*0.5) &&
                            (fabs(D[1])<=m_cellSize[1]*0.5) &&
                            (fabs(D[2])<=m_cellSize[2]*0.5)) {

                            m_indexedElement[key].insert(elmt);
                        }
                    }
                }
            }
        }
    }

    void draw(const core::visual::VisualParams * vparams) {
        if (! vparams->displayFlags().getShowBoundingCollisionModels()) return;

        type::RGBAColor color(1,0,0,1);

        glDisable(GL_LIGHTING);
        if (color[3] == 0.0) return;

        glColor4f(color[0],color[1],color[2],color[3]);
        for (auto it = m_indexedElement.cbegin();it != m_indexedElement.cend(); it++) {
            unsigned i = getIKey(it->first);
            unsigned j = getJKey(it->first);
            unsigned k = getKKey(it->first);

            type::Vector3 min = m_Bmin + type::Vector3((i  ) * m_cellSize[0],(j  ) * m_cellSize[1],(k  ) * m_cellSize[2]) ;
            type::Vector3 max = m_Bmin + type::Vector3((i+1) * m_cellSize[0],(j+1) * m_cellSize[1],(k+1) * m_cellSize[2]) ;
            type::BoundingBox bbox(min,max);

            type::Vector3 points[8];

            points[0] = type::Vector3(bbox.minBBox()[0], bbox.minBBox()[1], bbox.minBBox()[2]);
            points[1] = type::Vector3(bbox.maxBBox()[0], bbox.minBBox()[1], bbox.minBBox()[2]);
            points[2] = type::Vector3(bbox.minBBox()[0], bbox.maxBBox()[1], bbox.minBBox()[2]);
            points[3] = type::Vector3(bbox.maxBBox()[0], bbox.maxBBox()[1], bbox.minBBox()[2]);
            points[4] = type::Vector3(bbox.minBBox()[0], bbox.minBBox()[1], bbox.maxBBox()[2]);
            points[5] = type::Vector3(bbox.maxBBox()[0], bbox.minBBox()[1], bbox.maxBBox()[2]);
            points[6] = type::Vector3(bbox.minBBox()[0], bbox.maxBBox()[1], bbox.maxBBox()[2]);
            points[7] = type::Vector3(bbox.maxBBox()[0], bbox.maxBBox()[1], bbox.maxBBox()[2]);


            if (vparams->displayFlags().getShowWireFrame()) {
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
            } else {
                glBegin(GL_QUADS);
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

    inline Index getKey(size_t i,size_t j,size_t k) const {
        return i*m_offset[0] + j * m_offset[1] + k;
    }

    inline const type::Vector3 & getMin() const {
        return m_Bmin;
    }

    inline const type::Vector3 & getMax() const {
        return m_Bmax;
    }

    inline const type::Vector3 & getCellSize() const {
        return m_cellSize;
    }

    //compute the box where is P
    type::Vec3i getBoxCoord(const type::Vector3 & P) const override {
        return type::Vec3i((P[0] - m_Bmin[0])/m_cellSize[0],
                           (P[1] - m_Bmin[1])/m_cellSize[1],
                           (P[2] - m_Bmin[2])/m_cellSize[2]);
    }

    inline unsigned getIKey(unsigned key) {
        return key/m_offset[0];
    }

    inline unsigned getJKey(unsigned key) {
        return (key - getIKey(key) * m_offset[0])/m_offset[1];
    }

    inline unsigned getKKey(unsigned key) {
        return key - getIKey(key) * m_offset[0] - getJKey(key) * m_offset[1];
    }

    type::Vec3i getNbox() override {
        return d_nbox.getValue();
    }

protected:
    type::Vector3 m_Bmin,m_Bmax,m_cellSize;
    type::Vec3i m_nbox;
    type::Vec<2, size_t> m_offset;
    std::map<unsigned, std::set<BaseElement::SPtr> > m_indexedElement;
};


}
