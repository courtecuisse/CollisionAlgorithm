#pragma once

#include <sofa/collisionAlgorithm/BaseGeometryModifier.h>

namespace sofa
{

namespace collisionAlgorithm
{


template <typename TGeometry>
class SofaPhongNormalHandler : public SofaBaseNormalHandler
{
public:
    SOFA_CLASS(SofaPhongNormalHandler, SofaBaseNormalHandler);

    SofaPhongNormalHandler(const BaseGeometry* geometry)
        : SofaBaseNormalHandler(geometry)
    {

    }

    virtual ~SofaPhongNormalHandler() override {}

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

        msg_error(this) << "Geometry does not implement Phong Normal";
        return defaulttype::Vector3();
    }

protected:

};

}

}

