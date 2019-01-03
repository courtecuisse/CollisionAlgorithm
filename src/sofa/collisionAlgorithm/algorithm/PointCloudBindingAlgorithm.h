#pragma once

#include <sofa/collisionAlgorithm/BaseCollisionAlgorithm.h>
#include <sofa/collisionAlgorithm/decorator/AABBDecorator.h>

namespace sofa
{

namespace collisionAlgorithm
{

class PointCloudBindingAlgorithm : public BaseCollisionAlgorithm
{
public:
    typedef sofa::defaulttype::Vector3 Vector3;
    SOFA_CLASS(PointCloudBindingAlgorithm, BaseCollisionAlgorithm);

    Data<double> d_maxDist;

    PointCloudBindingAlgorithm();

    void processAlgorithm() override;

    void getState(std::set<sofa::core::behavior::MechanicalState<defaulttype::Vec3dTypes>* > & list_state) override
    {
        list_state.insert(l_from->getState());
        list_state.insert(l_dest->getState());
    }

private:
    core::objectmodel::SingleLink<PointCloudBindingAlgorithm,BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_from;
    core::objectmodel::SingleLink<PointCloudBindingAlgorithm,BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_dest;
};

}

}
//    void init();

//    PariProximityVector processAlgorithm(BaseGeometry * from,BaseGeometry * dest);

//};

//} // namespace controller

//} // namespace component

//} // namespace sofa

//#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
