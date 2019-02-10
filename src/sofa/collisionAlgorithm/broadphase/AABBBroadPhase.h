#pragma once

#include <sofa/helper/AdvancedTimer.h>
#include <sofa/collisionAlgorithm/BroadPhase.h>
#include <sofa/collisionAlgorithm/BaseGeometryAlgorithm.h>

namespace sofa
{

namespace collisionAlgorithm
{

class AABBBroadPhase : public BroadPhase {
    friend class AABBElement;

public:

    SOFA_CLASS(AABBBroadPhase,BroadPhase);

    Data<defaulttype::Vec3i> d_nbox;
    Data<bool> d_refineBBox;

    AABBBroadPhase();

    virtual ~AABBBroadPhase() override {}

    virtual void computeCollisionReset() override;

    virtual defaulttype::BoundingBox getBBox() const;

    virtual bool selectElement(const defaulttype::Vector3 & P,std::set<unsigned> & eid, unsigned d = 0) const;

    void draw(const core::visual::VisualParams * /*vparams*/) override;

    inline unsigned getKey(size_t i,size_t j,size_t k) const {
        return i*m_offset[0] + j * m_offset[1] + k;
    }

    inline const std::set<unsigned> & getIndexedElements(unsigned i,unsigned j, unsigned k) const {
        auto it = m_indexedElement.find(getKey(i,j,k));
        if (it == m_indexedElement.end()) return m_empty;
        else return it->second;
    }

    inline defaulttype::Vec3i getCoord(const defaulttype::Vector3 & P) const {
        defaulttype::Vec3i cbox;
        cbox[0] = floor(P[0]/m_cellSize[0]);
        cbox[1] = floor(P[1]/m_cellSize[1]);
        cbox[2] = floor(P[2]/m_cellSize[2]);
        return cbox;
    }

    inline const defaulttype::Vector3 & getMin() const {
        return m_Bmin;
    }

    inline const defaulttype::Vector3 & getMax() const {
        return m_Bmax;
    }

protected:
    defaulttype::Vector3 m_Bmin,m_Bmax,m_cellSize;
    defaulttype::Vec3i m_nbox;
    defaulttype::Vec<2, size_t> m_offset;
    std::map<unsigned, std::set<unsigned> > m_indexedElement;
    std::set<unsigned> m_empty;

    void fillElementSet(defaulttype::Vec3i cbox, std::set<unsigned> & selectElements, int b) const;
};


}

}
