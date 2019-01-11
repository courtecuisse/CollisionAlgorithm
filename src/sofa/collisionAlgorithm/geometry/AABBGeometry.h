#pragma once

#include <sofa/helper/AdvancedTimer.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseCollisionAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseElementFilter.h>

namespace sofa
{

namespace collisionAlgorithm
{

class AABBGeometry : public BaseGeometry {
    friend class AABBElement;

public:

    SOFA_CLASS(AABBGeometry,BaseGeometry);

    Data<defaulttype::Vec3i> d_nbox;

    AABBGeometry();

    virtual ~AABBGeometry() override {}

    void prepareDetection() override;

    virtual BaseElement::Iterator begin(unsigned eid = 0) const;

    virtual sofa::core::behavior::BaseMechanicalState * getState() const;

    void draw(const core::visual::VisualParams * /*vparams*/) override;

    core::objectmodel::SingleLink<AABBGeometry,BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;

    void selectElements(const defaulttype::Vector3 & P, std::set<unsigned> & elmt) const;

protected:
    defaulttype::Vector3 m_Bmin,m_Bmax,m_cellSize;
    defaulttype::Vec3i m_nbox;
    defaulttype::Vec<2, size_t> m_offset;
    std::map<unsigned, std::set<unsigned> > m_indexedElement;

    unsigned getKey(size_t i,size_t j,size_t k) const
    {
        return i*m_offset[0] + j * m_offset[1] + k;
    }

    void fillElementSet(defaulttype::Vec3i cbox, int d, std::set<unsigned> & selectElements) const;
};


}

}
