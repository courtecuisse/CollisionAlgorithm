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

    virtual void prepareDetection() override;

    virtual defaulttype::BoundingBox getBBox() const;

    virtual bool selectElement(const defaulttype::Vector3 & P,std::set<unsigned> & eid, unsigned d = 0) const;

    void draw(const core::visual::VisualParams * /*vparams*/) override;


protected:
    defaulttype::Vector3 m_Bmin,m_Bmax,m_cellSize;
    defaulttype::Vec3i m_nbox;
    defaulttype::Vec<2, size_t> m_offset;
    std::map<unsigned, std::set<unsigned> > m_indexedElement;

    unsigned getKey(size_t i,size_t j,size_t k) const
    {
        return i*m_offset[0] + j * m_offset[1] + k;
    }

    void fillElementSet(defaulttype::Vec3i cbox, std::set<unsigned> & selectElements, int b) const;
};


}

}
