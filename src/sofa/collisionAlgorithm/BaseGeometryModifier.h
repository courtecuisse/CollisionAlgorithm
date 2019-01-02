#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/core/BehaviorModel.h>
#include <memory>

namespace sofa
{

namespace collisionAlgorithm
{

class BaseGeometry;

//class BaseNormalHandler
//{
//public:
//    virtual defaulttype::Vector3 getNormal(const BaseGeometry*,const size_t, const double* fact) const = 0;

//};

//template<class TGeometry>
//class TNormalHandler : public BaseNormalHandler
//{
//public:
//    virtual defaulttype::Vector3 getNormal(const BaseGeometry* geometry, const size_t id, const double* fact) const
//    {
//        return internalGetNormal(geometry, id, fact);
//    }
//protected:
//    virtual defaulttype::Vector3 internalGetNormal(const BaseGeometry*, const size_t id, const double* fact) const = 0;

//};

//template<typename TGeometry>
//class FlatNormalHandler : public TNormalHandler<TGeometry>
//{
//protected:
//    defaulttype::Vector3 internalGetNormal(const BaseGeometry*, const size_t, const double* fact) const ;
//};


class SofaBaseNormalHandler : public sofa::core::objectmodel::BaseObject
{
public:
    typedef sofa::core::objectmodel::BaseObject Inherit;

    SOFA_CLASS(SofaBaseNormalHandler, Inherit);

    SofaBaseNormalHandler(const BaseGeometry* geometry)
        : Inherit()
        , m_geometry(geometry)
//        , m_impl(nullptr)
    {}
    virtual ~SofaBaseNormalHandler() override
    {}

    void init() override
    {

    }

    virtual defaulttype::Vector3 getNormal(const size_t elementID, const double* fact) const
    {
        SOFA_UNUSED(elementID);
        SOFA_UNUSED(fact);

        ///TODO: set up a warning ??
        ///
        return defaulttype::Vector3();
    }

    void draw(const sofa::core::visual::VisualParams* vparams) override;

    void setGeometry(const BaseGeometry* geometry)
    {
        m_geometry = geometry;
    }
//    void setHandler(BaseNormalHandler* impl)
//    {
//        m_impl = impl;
//    }

protected:
    const BaseGeometry* m_geometry;
    //BaseNormalHandler* m_impl;
};

template <typename TGeometry>
class SofaFlatNormalHandler : public SofaBaseNormalHandler
{
public:
    SOFA_CLASS(SofaFlatNormalHandler, SofaBaseNormalHandler);

    SofaFlatNormalHandler(const BaseGeometry* geometry)
        : SofaBaseNormalHandler(geometry)
    {

    }

    virtual ~SofaFlatNormalHandler() override {}

    template<class T>
    static bool canCreate(T*& obj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg)
    {
        BaseGeometry* geometry = nullptr;
        context->get(geometry);
        if (!geometry)
            return false;
        if(!static_cast<TGeometry*>(geometry))
            return false;

        return BaseObject::canCreate(obj, context, arg);
    }

    template<class T>
    static typename T::SPtr create(T*, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg)
    {
        BaseGeometry* geometry = nullptr;
        context->get(geometry);

        typename T::SPtr obj = sofa::core::objectmodel::New<T>(geometry);

        if (context)
            context->addObject(obj);
        if (arg)
            obj->parse(arg);

        return obj;
    }
    defaulttype::Vector3 getNormal(const size_t elementID, const double* fact) const
    {
        SOFA_UNUSED(elementID);
        SOFA_UNUSED(fact);

        msg_error(this) << "Geometry does not implement Flat Normal";
        return defaulttype::Vector3();
    }

protected:

};

}

}
