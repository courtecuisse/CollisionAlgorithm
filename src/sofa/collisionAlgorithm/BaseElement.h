#pragma once

#include <memory>
#include <map>
#include <vector>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/VecId.h>
#include <sofa/core/behavior/MechanicalState.h>

namespace sofa
{

namespace collisionAlgorithm
{

class ConstraintElement;
class BaseGeometry;

class ConstraintProximity
{
public :
    typedef std::shared_ptr<ConstraintProximity> SPtr;
    typedef Data<helper::vector<defaulttype::Vector3> > DataVecCoord;

    ConstraintProximity(const ConstraintElement *elmt);

    virtual ~ConstraintProximity() {}

    virtual inline const ConstraintElement* element() const = 0;

    virtual defaulttype::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const = 0;

    virtual defaulttype::Vector3 getNormal() const = 0;

    virtual std::map<unsigned,double> getContributions() const = 0;

    inline sofa::core::behavior::MechanicalState<defaulttype::Vec3dTypes> * getState() const
    {
        return m_state;
    }

protected:
//    const ConstraintElement * m_element;
    sofa::core::behavior::MechanicalState<defaulttype::Vec3dTypes> * m_state;
};

class ConstraintElement
{
    friend class ConstraintProximity;
public:
    typedef std::unique_ptr<ConstraintElement> UPtr;
    typedef Data<helper::vector<defaulttype::Vector3> > DataVecCoord;

    ConstraintElement()
    {}

    virtual ~ConstraintElement() {}

    //this function returns a vector with all the control points of the element
    //if an id is not >=0 and <getNbControlPoints() this function should return the gravity center of the element
    virtual ConstraintProximity::SPtr getControlPoint(int i = -1) const = 0;

    //this function project the point P on the element and return the corresponding proximity
    virtual ConstraintProximity::SPtr project(defaulttype::Vector3 P) const = 0;

    //return the geometry // See covariant return type !
    virtual inline const BaseGeometry* geometry() const = 0;

    // return the number of control points
    virtual inline size_t getNbControlPoints() const = 0;

    virtual void draw(const core::visual::VisualParams *vparams) const = 0;

};

} // namespace controller

}
