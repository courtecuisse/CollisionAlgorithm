#pragma once

#include <sofa/helper/AdvancedTimer.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseCollisionAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseElementFilter.h>

namespace sofa
{

namespace collisionAlgorithm
{

class AABBDecorator;

class AABBElementIterator : public BaseElementFilterIterator
{
public :
    AABBElementIterator(const ConstraintElement *from, const AABBDecorator * aabb);

    virtual ~AABBElementIterator() override {}


    size_t size() const override
    {
        return m_selectElements.size();
    }

    inline const ConstraintElement* element(size_t i) const override
    {
        return m_geo->getElement(m_selectElements[i]);
    }
private:
    void fillElementSet(defaulttype::Vec3i cbox, int d, std::set<int> & selectElements);

    BaseGeometry * m_geo;
    const AABBDecorator * m_aabb;
    std::vector<int> m_selectElements;
};

class AABBDecorator : public BaseElementFilter
{
    friend class AABBElement;
    friend class AABBElementIterator;
public:
    typedef Data<helper::vector<defaulttype::Vector3> > DataVecCoord;

    SOFA_CLASS(AABBDecorator, core::BehaviorModel);

    Data<defaulttype::Vec3i> d_nbox;

    AABBDecorator(BaseGeometry* geometry = nullptr);

    virtual ~AABBDecorator() override {}

    void prepareDetection() override;

    //replace with a factory ?
    std::unique_ptr<BaseElementFilterIterator> iterator(const ConstraintElement *from) override
    {
        return std::unique_ptr<BaseElementFilterIterator>(new AABBElementIterator(from, this));
    }

    void draw(const core::visual::VisualParams * /*vparams*/) override;

    const std::set<unsigned> & getConstIndexedElements(size_t i,size_t j,size_t k) const
    {
        return m_indexedElement[i*m_offset[0] + j * m_offset[1] + k];
    }

    std::set<unsigned> &getIndexedElements(size_t i,size_t j,size_t k)
    {
        return m_indexedElement[i*m_offset[0] + j * m_offset[1] + k];
    }

protected:
    defaulttype::Vector3 m_Bmin,m_Bmax,m_cellSize;
    defaulttype::Vec3i m_nbox;
    defaulttype::Vec<2, size_t> m_offset;
    std::vector<std::set<unsigned> >  m_indexedElement;
};


}

}
