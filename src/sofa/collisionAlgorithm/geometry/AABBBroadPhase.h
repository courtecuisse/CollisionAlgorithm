#pragma once

#include <sofa/helper/AdvancedTimer.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseAlgorithm.h>

namespace sofa
{

namespace collisionAlgorithm
{

/*!
 * \brief The AABBBroadPhase class
 * Implementation of broad phase collision detection using bounding boxes
 */
class AABBBroadPhase : public BaseGeometry {
    friend class AABBElement;

public:

    SOFA_CLASS(AABBBroadPhase,BaseGeometry);

    typedef BaseGeometry::BaseGeometry::Index Index;

    Data<type::Vec3i> d_nbox;
    Data<bool> d_refineBBox;
    Data<bool> d_static;

    core::objectmodel::SingleLink<AABBBroadPhase, BaseGeometry, BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;

    AABBBroadPhase();

    virtual ~AABBBroadPhase() override {}

    virtual void init();

    virtual void prepareDetection() override;

    virtual BaseElementIterator::SPtr begin(Index eid = 0) const override;

    virtual sofa::core::behavior::BaseMechanicalState * getState() const override;

    virtual type::BoundingBox getBBox() const;

    void draw(const core::visual::VisualParams * /*vparams*/) override;

    inline Index getKey(size_t i,size_t j,size_t k) const {
        return i*m_offset[0] + j * m_offset[1] + k;
    }

    inline type::Vec3i getCoord(const type::Vector3 & P) const {
        type::Vec3i cbox;
        for (int i = 0 ; i < 3 ; i++) {
            cbox[i] = floor(P[i]/m_cellSize[i]);
        }
//        cbox[0] = floor(P[0]/m_cellSize[0]);
//        cbox[1] = floor(P[1]/m_cellSize[1]);
//        cbox[2] = floor(P[2]/m_cellSize[2]);
        return cbox;
    }

    inline const type::Vector3 & getMin() const {
        return m_Bmin;
    }

    inline const type::Vector3 & getMax() const {
        return m_Bmax;
    }

    virtual type::Vec3i getBoxSize() const {
        return m_nbox;
    }

    virtual type::Vec3i getBoxCoord(const type::Vector3 & P) const {
        //compute the box where is P
        type::Vec3i cbox;
        for (int i = 0 ; i < 3 ; i++) {
            cbox[i] = floor((P[i] - m_Bmin[i])/m_cellSize[i]);
        }
//        cbox[0] = floor((P[0] - m_Bmin[0])/m_cellSize[0]);
//        cbox[1] = floor((P[1] - m_Bmin[1])/m_cellSize[1]);
//        cbox[2] = floor((P[2] - m_Bmin[2])/m_cellSize[2]);

        //project the box in the bounding box of the object
        //search with the closest box in bbox
        for (Index i=0;i<3;i++)
        {
            if (cbox[i] < 0) {
                cbox[i] = 0;
            } else if (cbox[i] >= m_nbox[i]) {
                cbox[i]=m_nbox[i]-1;
            }
        }

        return cbox;
    }

    virtual void getElementSet(type::Vec3i c, std::set<Index> & selectElements) const {
        auto it = m_indexedElement.find(getKey(c[0],c[1],c[2]));
        if (it != m_indexedElement.end()) {
            const std::set<Index> & elemntsID = it->second;
            selectElements.insert(elemntsID.begin(),elemntsID.end());
        }
    }

    virtual void recomputeNormals() {}

protected:
    type::Vector3 m_Bmin,m_Bmax,m_cellSize;
    type::Vec3i m_nbox;
    type::Vec<2, size_t> m_offset;
    std::map<Index, std::set<Index> > m_indexedElement;
    bool m_staticInitDone;
};


}

}
