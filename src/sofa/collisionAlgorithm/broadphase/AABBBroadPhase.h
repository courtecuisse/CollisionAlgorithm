#pragma once

#include <sofa/helper/AdvancedTimer.h>
#include <sofa/collisionAlgorithm/BroadPhase.h>
#include <sofa/collisionAlgorithm/BaseAlgorithm.h>

namespace sofa
{

namespace collisionAlgorithm
{

/*!
 * \brief The AABBBroadPhase class
 * Implementation of broad phase collision detection using bounding boxes
 */
class AABBBroadPhase : public BroadPhase {
    friend class AABBElement;

public:

    SOFA_CLASS(AABBBroadPhase,BroadPhase);

    Data<defaulttype::Vec3i> d_nbox;
    Data<bool> d_refineBBox;
    Data<bool> d_static;

    AABBBroadPhase();

    virtual ~AABBBroadPhase() override {}

    virtual void prepareDetection() override;

    virtual defaulttype::BoundingBox getBBox() const;

    void draw(const core::visual::VisualParams * /*vparams*/) override;

    inline unsigned getKey(size_t i,size_t j,size_t k) const {
        return i*m_offset[0] + j * m_offset[1] + k;
    }

    inline defaulttype::Vec3i getCoord(const defaulttype::Vector3 & P) const {
        defaulttype::Vec3i cbox;
        for (int i = 0 ; i < 3 ; i++) {
            cbox[i] = floor(P[i]/m_cellSize[i]);
        }
//        cbox[0] = floor(P[0]/m_cellSize[0]);
//        cbox[1] = floor(P[1]/m_cellSize[1]);
//        cbox[2] = floor(P[2]/m_cellSize[2]);
        return cbox;
    }

    inline const defaulttype::Vector3 & getMin() const {
        return m_Bmin;
    }

    inline const defaulttype::Vector3 & getMax() const {
        return m_Bmax;
    }

    virtual defaulttype::Vec3i getBoxSize() const {
        return m_nbox;
    }

    virtual defaulttype::Vec3i getBoxCoord(const defaulttype::Vector3 & P) const {
        //compute the box where is P
        defaulttype::Vec3i cbox;
        for (int i = 0 ; i < 3 ; i++) {
            cbox[i] = floor((P[i] - m_Bmin[i])/m_cellSize[i]);
        }
//        cbox[0] = floor((P[0] - m_Bmin[0])/m_cellSize[0]);
//        cbox[1] = floor((P[1] - m_Bmin[1])/m_cellSize[1]);
//        cbox[2] = floor((P[2] - m_Bmin[2])/m_cellSize[2]);

        //project the box in the bounding box of the object
        //search with the closest box in bbox
        for (unsigned int i=0;i<3;i++)
        {
            if (cbox[i] < 0) {
                cbox[i] = 0;
            } else if (cbox[i] >= m_nbox[i]) {
                cbox[i]=m_nbox[i]-1;
            }
        }

        return cbox;
    }

    virtual void getElementSet(unsigned cx,unsigned cy, unsigned cz, std::set<unsigned> & selectElements) const {
        auto it = m_indexedElement.find(getKey(cx,cy,cz));
        if (it != m_indexedElement.end()) {
            const std::set<unsigned> & elemntsID = it->second;
            selectElements.insert(elemntsID.begin(),elemntsID.end());
        }
    }

protected:
    defaulttype::Vector3 m_Bmin,m_Bmax,m_cellSize;
    defaulttype::Vec3i m_nbox;
    defaulttype::Vec<2, size_t> m_offset;
    std::map<unsigned, std::set<unsigned> > m_indexedElement;
    bool m_staticInitDone;
};


}

}
