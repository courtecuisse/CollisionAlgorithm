#pragma once

#include <sofa/core/collision/Pipeline.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

/*!
 * \brief The BroadPhase class defines an interface for the collision detection broad phase
 */
class BroadPhase : public core::BehaviorModel
{
public:
    SOFA_ABSTRACT_CLASS(BroadPhase,core::BehaviorModel);

    Data<defaulttype::Vector4> d_color;
    core::objectmodel::SingleLink<BroadPhase,BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;
//    sofa::core::objectmodel::_datacallback_::DataCallback c_update;

    /*!
     * \brief BroadPhase Constructor
     */
    BroadPhase()
    : d_color(initData(&d_color, defaulttype::Vector4(1,0,1,1), "color", "Color of the collision model"))
    , l_geometry(initLink("geometry", "link to state")) {
//        c_update.addCallback(std::bind(&BroadPhase::prepareDetection,this));
    }

    /*!
     * \brief ~BroadPhase destructor
     */
    virtual ~BroadPhase() {
        if (l_geometry != NULL) l_geometry->setBroadPhase(NULL);
    }

//    void init( ) override {
//        if (l_geometry != NULL) {
//            core::objectmodel::BaseData * data = l_geometry->getState()->findData("position");
//            if (data) {
//                l_geometry->setBroadPhase(this);
//                c_update.addInput(data);
//            }
//        }
//    }

    /*!
     * \brief prepareDetection virtual method to implement
     * detection pre-processing
     */
    virtual void prepareDetection() = 0;

    /*!
     * \brief getBoxSize
     * \return bounding box size in a vec3i
     */
    virtual defaulttype::Vec3i getBoxSize() const = 0;

    /*!
     * \brief getBoxCoord
     * \param P : point in space
     * \return the box's coordinates (vec3i) containing point P
     */
    virtual defaulttype::Vec3i getBoxCoord(const defaulttype::Vector3 & P) const = 0;

    virtual void getElementSet(unsigned cx,unsigned cy, unsigned cz, std::set<unsigned> & selectElements) const = 0;

protected:
    /// Computation of a new simulation step.
    virtual void updatePosition(SReal ) {
        prepareDetection();
    }
};


}

}
