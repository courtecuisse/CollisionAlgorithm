#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/core/BehaviorModel.h>

namespace sofa
{

namespace collisionAlgorithm
{

class BaseElementFilter;

class BaseElementFilterIterator
{
public:
    BaseElementFilterIterator(const ConstraintElement * from, const BaseElementFilter * filter)
        : m_from(from)
        , m_filter(filter)
    {

    }
    virtual ~BaseElementFilterIterator() {}

    virtual inline const ConstraintElement* element(size_t i) const = 0;
    virtual size_t size() const = 0;

    inline const ConstraintElement* getFrom() { return m_from; }

protected:
    const ConstraintElement * m_from;
    const BaseElementFilter* m_filter;
};

class BaseElementFilter : public core::BehaviorModel
{
public:
    SOFA_ABSTRACT_CLASS(BaseElementFilter, core::BehaviorModel);

    BaseElementFilter(BaseGeometry* geometry = nullptr)
        :l_geometry(initLink("geometry", "link to the filtered geometry"))
    {
        if(geometry)
            l_geometry.set(geometry);
        else
            if(!l_geometry.get())
                l_geometry.setPath("@.");
    }

    virtual ~BaseElementFilter() override {}

    void updatePosition(SReal ) override
    {
        prepareDetection();
    }

    void init() override
    {
        if (l_geometry != nullptr)
        {
            l_geometry->addSlave(this);
            prepareDetection();
        }
    }

    virtual void prepareDetection() = 0;
    virtual std::unique_ptr<BaseElementFilterIterator> iterator(const ConstraintElement *from) = 0;

    inline const BaseGeometry* geometry() const { return l_geometry.get(); }


protected:
    core::objectmodel::SingleLink<BaseElementFilter,BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;
};


class DefaultElementIterator : public BaseElementFilterIterator
{
public:
    DefaultElementIterator(const ConstraintElement * from, const BaseElementFilter * filter)
        : BaseElementFilterIterator(from, filter)
    {
    }

    virtual ~DefaultElementIterator() override {}

    size_t size() const override
    {
        return m_filter->geometry()->getNbElements();
    }

    inline const ConstraintElement* element(size_t i) const override
    {
        return m_filter->geometry()->getElement(i);
    }
protected:
};


class DefaultElementFilter : public BaseElementFilter
{
public:
    SOFA_CLASS(DefaultElementFilter, core::BehaviorModel);

    Data<defaulttype::Vec3i> d_nbox;

    DefaultElementFilter(BaseGeometry* geometry = nullptr)
        : BaseElementFilter(geometry)
    {}

    virtual ~DefaultElementFilter() override {}

    void prepareDetection() override {}

    //replace with a factory ?
    std::unique_ptr<BaseElementFilterIterator> iterator(const ConstraintElement *from) override
    {
        return std::unique_ptr<BaseElementFilterIterator>(new DefaultElementIterator(from, this));
    }

protected:
};


}

}
