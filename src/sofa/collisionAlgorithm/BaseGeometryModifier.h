#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/core/BehaviorModel.h>
#include <memory>

namespace sofa
{

namespace collisionAlgorithm
{

class BaseNormalHandler
{
public:
    virtual defaulttype::Vector3 getNormal(const BaseGeometry*,const unsigned, const double* fact) const = 0;

};

template<class TElement>
class TNormalHandler : public BaseNormalHandler
{
public:
    virtual defaulttype::Vector3 getNormal(const BaseGeometry* geometry, const unsigned id, const double* fact) const
    {
        return internalGetNormal(geometry->getElement(id), fact);
    }
protected:
    virtual defaulttype::Vector3 internalGetNormal(const TElement*, const double* fact) const = 0;

};

template<typename TElement>
class FlatNormalHandler : public TNormalHandler<TElement>
{
protected:
    defaulttype::Vector3 internalGetNormal(const TElement*, const double* fact) const ;
};


class SofaBaseNormalHandler : public sofa::core::objectmodel::BaseObject
{
public:
    SOFA_CLASS(SofaBaseNormalHandler, sofa::core::objectmodel::BaseObject);

    SofaBaseNormalHandler()
        : sofa::core::objectmodel::BaseObject()
        , m_impl(nullptr)
    {}
    virtual ~SofaBaseNormalHandler() override
    {}

    void init() override
    {
        if(!l_geometry.get())
        {
            msg_warning(this) << "NO geometry given, finding one...";
            l_geometry.setPath("@.");
        }
        if(!l_geometry.get())
        {
            msg_warning(this) << "No geometry found, giving up";
            return;
        }

        l_geometry->setNormalHandler(this);
    }

    defaulttype::Vector3 getNormal(const unsigned elementID, const double* fact) const
    {
        return m_impl->getNormal(l_geometry.get(), elementID, fact);
    }

protected:
    core::objectmodel::SingleLink<SofaBaseNormalHandler,BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;
    BaseNormalHandler* m_impl;
};

class SofaFlatNormalHandler : public SofaBaseNormalHandler
{
public:
    SOFA_CLASS(SofaFlatNormalHandler, SofaBaseNormalHandler);

    SofaFlatNormalHandler();


};

}

}
